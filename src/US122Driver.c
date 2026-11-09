// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Oliver Frommel

// US122Driver — AudioServerPlugIn exposing the TASCAM US-122MKII to Core Audio.
//
// Object tree:  PlugIn (1) -> Device (2) -> Input stream (3), Output stream (4)
// The device is only listed while the hardware is connected. All USB work is
// done by US122Engine; this file is the Core Audio property/IO glue.

#include <CoreAudio/AudioServerPlugIn.h>
#include <CoreFoundation/CoreFoundation.h>
#include <mach/mach_time.h>
#include <os/log.h>
#include <xpc/xpc.h>
#include <pthread.h>
#include <stdatomic.h>
#include <math.h>
#include "US122Engine.h"

enum {
    kObjectID_PlugIn = kAudioObjectPlugInObject,
    kObjectID_Device = 2,
    kObjectID_StreamIn = 3,
    kObjectID_StreamOut = 4,
    kObjectID_Volume = 5, // output volume (software gain)
    kObjectID_Mute = 6,   // output mute
};

#define kDeviceUID "US122MKII-0644-8021"
#define kModelUID "US122MKII"
#define kZeroTimeStampPeriod 16384

static const Float64 kRates[] = {44100.0, 48000.0, 88200.0, 96000.0};
#define kNumRates (sizeof kRates / sizeof kRates[0])

static AudioServerPlugInHostRef gHost;
static pthread_mutex_t gLock = PTHREAD_MUTEX_INITIALIZER;
static _Atomic UInt32 gRefCount = 1;
static Float64 gSampleRate = 48000.0;
static UInt32 gIOCount;
static bool gStreamActive[2] = {true, true}; // in, out
static os_log_t gLog;

// Output volume. The hardware has no digital volume, so it is applied as a
// software gain in WriteMix. Scalar -> gain uses a squared curve (perceptually
// reasonable); dB = 40*log10(scalar), floored at kMinDB.
#define kMinDB -64.0f
static _Atomic float gVolumeScalar = 0.75f;
static _Atomic bool gMute = false;
static float gAppliedGain = -1.0f; // last gain used on the IO thread, for ramping

static float ScalarToDB(float s) {
    if (s <= 0) return kMinDB;
    float db = 40.0f * log10f(s);
    return db < kMinDB ? kMinDB : (db > 0 ? 0 : db);
}
static float DBToScalar(float db) {
    if (db <= kMinDB) return 0;
    if (db >= 0) return 1;
    return powf(10.0f, db / 40.0f);
}
static float TargetGain(void) {
    if (atomic_load(&gMute)) return 0;
    float s = atomic_load(&gVolumeScalar);
    return s * s;
}

// Settings persisted through the HAL's per-plug-in storage.
#define kKeyVolume CFSTR("volume")
#define kKeyMute CFSTR("mute")
#define kKeyRate CFSTR("sampleRate")

static void StoreNumber(CFStringRef key, Float64 v) {
    if (!gHost) return;
    CFNumberRef n = CFNumberCreate(NULL, kCFNumberFloat64Type, &v);
    if (n) {
        gHost->WriteToStorage(gHost, key, n);
        CFRelease(n);
    }
}

static bool LoadNumber(CFStringRef key, Float64 *out) {
    CFPropertyListRef d = NULL;
    bool ok = false;
    if (gHost->CopyFromStorage(gHost, key, &d) == kAudioHardwareNoError && d) {
        if (CFGetTypeID(d) == CFNumberGetTypeID()) ok = CFNumberGetValue((CFNumberRef)d, kCFNumberFloat64Type, out);
        CFRelease(d);
    }
    return ok;
}

// ---------------------------------------------------------------- MIDI
//
// Core Audio's sandbox doesn't let this plug-in use CoreMIDI, so the MIDI
// ports are published by a separate CoreMIDI driver (midi/US122MIDIDriver.c)
// running in the user's MIDIServer. MIDI bytes travel between it and us through
// the us122-midirelay launchd daemon (midi/relay.c), the one service both
// sandboxes can reach. The device only passes MIDI while audio streams, so the
// MIDI driver runs a silent Core Audio stream on this device while MIDI is used.

#define RELAY_SERVICE "local.us122mkii.midirelay"

static dispatch_queue_t gRelayQ;
static xpc_connection_t gRelay; // gRelayQ

static void RelaySendPresent(bool present) {
    xpc_object_t m = xpc_dictionary_create(NULL, NULL, 0);
    xpc_dictionary_set_bool(m, "present", present);
    dispatch_async(gRelayQ, ^{ if (gRelay) xpc_connection_send_message(gRelay, m); });
}

static void RelayHello(void) {
    xpc_object_t m = xpc_dictionary_create(NULL, NULL, 0);
    xpc_dictionary_set_string(m, "role", "audio");
    xpc_connection_send_message(gRelay, m);
    m = xpc_dictionary_create(NULL, NULL, 0);
    xpc_dictionary_set_bool(m, "present", US122EngineIsPresent());
    xpc_connection_send_message(gRelay, m);
}

static void RelayConnect(void) {
    if (gRelay) return;
    xpc_connection_t c = xpc_connection_create_mach_service(RELAY_SERVICE, gRelayQ, XPC_CONNECTION_MACH_SERVICE_PRIVILEGED);
    gRelay = c;
    xpc_connection_set_event_handler(c, ^(xpc_object_t msg) {
        if (xpc_get_type(msg) == XPC_TYPE_DICTIONARY) {
            size_t n = 0;
            const void *out = xpc_dictionary_get_data(msg, "out", &n);
            if (out && n) US122EngineMidiSend(out, n);
        } else if (msg == XPC_ERROR_CONNECTION_INTERRUPTED) {
            RelayHello(); // relay restarted
        } else if (msg == XPC_ERROR_CONNECTION_INVALID) {
            // relay not installed (or rejected us): MIDI unavailable, retry occasionally
            os_log(gLog, "MIDI relay unavailable");
            gRelay = NULL;
            dispatch_after(dispatch_time(DISPATCH_TIME_NOW, 30 * NSEC_PER_SEC), gRelayQ, ^{ RelayConnect(); });
        }
    });
    xpc_connection_resume(c);
    RelayHello();
}

// engine io queue: raw bytes from the device's MIDI IN jack
static void MidiInCallback(void *ctx, const uint8_t *bytes, size_t len) {
    xpc_object_t m = xpc_dictionary_create(NULL, NULL, 0);
    xpc_dictionary_set_data(m, "in", bytes, len);
    dispatch_async(gRelayQ, ^{ if (gRelay) xpc_connection_send_message(gRelay, m); });
}

static void SetupMIDI(void) {
    gRelayQ = dispatch_queue_create("local.us122mkii.midirelay", DISPATCH_QUEUE_SERIAL);
    US122EngineSetMidiInCallback(MidiInCallback, NULL);
    dispatch_async(gRelayQ, ^{ RelayConnect(); });
}

// ---------------------------------------------------------------- engine callbacks

static void LogCallback(const char *msg) { os_log(gLog, "%{public}s", msg); }

static void PresenceCallback(void *ctx, bool present) {
    if (gRelayQ) RelaySendPresent(present);
    if (!gHost) return;
    if (!present) {
        // The HAL tears the device down; make sure the next StartIO after a
        // re-plug starts the engine again.
        pthread_mutex_lock(&gLock);
        bool wasRunning = gIOCount > 0;
        gIOCount = 0;
        pthread_mutex_unlock(&gLock);
        if (wasRunning) {
            os_log(gLog, "device removed while running; stopping IO");
            US122EngineStop();
        }
    }
    AudioObjectPropertyAddress addrs[] = {
        {kAudioPlugInPropertyDeviceList, kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain},
        {kAudioObjectPropertyOwnedObjects, kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain},
    };
    gHost->PropertiesChanged(gHost, kObjectID_PlugIn, 2, addrs);
    AudioObjectPropertyAddress alive = {kAudioDevicePropertyDeviceIsAlive, kAudioObjectPropertyScopeGlobal,
                                        kAudioObjectPropertyElementMain};
    gHost->PropertiesChanged(gHost, kObjectID_Device, 1, &alive);
}

// ---------------------------------------------------------------- helpers

static AudioStreamBasicDescription CurrentFormat(void) {
    return (AudioStreamBasicDescription){
        .mSampleRate = gSampleRate,
        .mFormatID = kAudioFormatLinearPCM,
        .mFormatFlags = kAudioFormatFlagIsFloat | kAudioFormatFlagsNativeEndian | kAudioFormatFlagIsPacked,
        .mBytesPerPacket = 8,
        .mFramesPerPacket = 1,
        .mBytesPerFrame = 8,
        .mChannelsPerFrame = 2,
        .mBitsPerChannel = 32,
    };
}

static CFURLRef CopyIconURL(void) {
    CFBundleRef b = CFBundleGetBundleWithIdentifier(CFSTR("local.us122mkii.driver"));
    return b ? CFBundleCopyResourceURL(b, CFSTR("US122MKII"), CFSTR("icns"), NULL) : NULL;
}

static bool IsStream(AudioObjectID o) { return o == kObjectID_StreamIn || o == kObjectID_StreamOut; }
static bool IsControl(AudioObjectID o) { return o == kObjectID_Volume || o == kObjectID_Mute; }
static bool ValidObject(AudioObjectID o) { return o == kObjectID_PlugIn || o == kObjectID_Device || IsStream(o) || IsControl(o); }

#define RET_SCALAR(type, val)                                                         \
    do {                                                                              \
        if (outData) {                                                                \
            if (inDataSize < sizeof(type)) return kAudioHardwareBadPropertySizeError; \
            *(type *)outData = (val);                                                 \
        }                                                                             \
        *outDataSize = sizeof(type);                                                  \
        return kAudioHardwareNoError;                                                 \
    } while (0)

#define RET_ARRAY(type, arr, count)                                     \
    do {                                                                \
        UInt32 n_ = (UInt32)(count);                                    \
        if (outData) {                                                  \
            if (n_ > inDataSize / sizeof(type)) n_ = inDataSize / sizeof(type); \
            if (n_) memcpy(outData, (arr), n_ * sizeof(type));          \
        }                                                               \
        *outDataSize = n_ * (UInt32)sizeof(type);                       \
        return kAudioHardwareNoError;                                   \
    } while (0)

// Computes the property size and, if outData != NULL, its value.
static OSStatus GetProp(AudioObjectID obj, const AudioObjectPropertyAddress *a, UInt32 qualSize, const void *qual,
                        UInt32 inDataSize, UInt32 *outDataSize, void *outData) {
    switch (obj) {
    case kObjectID_PlugIn:
        switch (a->mSelector) {
        case kAudioObjectPropertyBaseClass: RET_SCALAR(AudioClassID, kAudioObjectClassID);
        case kAudioObjectPropertyClass: RET_SCALAR(AudioClassID, kAudioPlugInClassID);
        case kAudioObjectPropertyOwner: RET_SCALAR(AudioObjectID, kAudioObjectUnknown);
        case kAudioObjectPropertyManufacturer: RET_SCALAR(CFStringRef, CFSTR("TASCAM US-122MKII community driver"));
        case kAudioObjectPropertyOwnedObjects:
        case kAudioPlugInPropertyDeviceList: {
            AudioObjectID dev = kObjectID_Device;
            RET_ARRAY(AudioObjectID, &dev, US122EngineIsPresent() ? 1 : 0);
        }
        case kAudioPlugInPropertyTranslateUIDToDevice: {
            AudioObjectID r = kAudioObjectUnknown;
            if (outData && qualSize == sizeof(CFStringRef) && qual && *(CFStringRef *)qual &&
                CFStringCompare(*(CFStringRef *)qual, CFSTR(kDeviceUID), 0) == kCFCompareEqualTo &&
                US122EngineIsPresent())
                r = kObjectID_Device;
            RET_SCALAR(AudioObjectID, r);
        }
        case kAudioPlugInPropertyResourceBundle: RET_SCALAR(CFStringRef, CFSTR(""));
        case kAudioObjectPropertyName: RET_SCALAR(CFStringRef, CFSTR("US-122MKII"));
        }
        break;

    case kObjectID_Device: {
        bool in = a->mScope == kAudioObjectPropertyScopeInput, out = a->mScope == kAudioObjectPropertyScopeOutput;
        switch (a->mSelector) {
        case kAudioObjectPropertyBaseClass: RET_SCALAR(AudioClassID, kAudioObjectClassID);
        case kAudioObjectPropertyClass: RET_SCALAR(AudioClassID, kAudioDeviceClassID);
        case kAudioObjectPropertyOwner: RET_SCALAR(AudioObjectID, kObjectID_PlugIn);
        case kAudioObjectPropertyName: RET_SCALAR(CFStringRef, CFSTR("TASCAM US-122MKII"));
        case kAudioObjectPropertyManufacturer: RET_SCALAR(CFStringRef, CFSTR("TASCAM"));
        case kAudioObjectPropertyElementName: {
            // channel names as printed on the panel
            static const CFStringRef inNames[] = {CFSTR("In L (Mic/Line)"), CFSTR("In R (Mic/Line/Guitar)")};
            static const CFStringRef outNames[] = {CFSTR("Out L"), CFSTR("Out R")};
            if ((!in && !out) || a->mElement < 1 || a->mElement > 2) return kAudioHardwareUnknownPropertyError;
            RET_SCALAR(CFStringRef, (in ? inNames : outNames)[a->mElement - 1]);
        }
        case kAudioDevicePropertyIcon: {
            if (!outData) { *outDataSize = sizeof(CFURLRef); return kAudioHardwareNoError; }
            if (inDataSize < sizeof(CFURLRef)) return kAudioHardwareBadPropertySizeError;
            CFURLRef url = CopyIconURL();
            if (!url) return kAudioHardwareUnspecifiedError;
            *(CFURLRef *)outData = url; // the caller releases it
            *outDataSize = sizeof(CFURLRef);
            return kAudioHardwareNoError;
        }
        case kAudioDevicePropertyDeviceUID: RET_SCALAR(CFStringRef, CFSTR(kDeviceUID));
        case kAudioDevicePropertyModelUID: RET_SCALAR(CFStringRef, CFSTR(kModelUID));
        case kAudioDevicePropertyConfigurationApplication: RET_SCALAR(CFStringRef, CFSTR("com.apple.audio.AudioMIDISetup"));
        case kAudioDevicePropertyTransportType: RET_SCALAR(UInt32, kAudioDeviceTransportTypeUSB);
        case kAudioDevicePropertyRelatedDevices: { AudioObjectID d = kObjectID_Device; RET_ARRAY(AudioObjectID, &d, 1); }
        case kAudioDevicePropertyClockDomain: RET_SCALAR(UInt32, 0);
        case kAudioDevicePropertyDeviceIsAlive: RET_SCALAR(UInt32, US122EngineIsPresent() ? 1 : 0);
        case kAudioDevicePropertyDeviceIsRunning: RET_SCALAR(UInt32, gIOCount > 0 ? 1 : 0);
        case kAudioDevicePropertyDeviceCanBeDefaultDevice: RET_SCALAR(UInt32, 1);
        case kAudioDevicePropertyDeviceCanBeDefaultSystemDevice: RET_SCALAR(UInt32, 1);
        case kAudioDevicePropertyLatency: RET_SCALAR(UInt32, 0);
        case kAudioDevicePropertyIsHidden: RET_SCALAR(UInt32, 0);
        case kAudioDevicePropertyZeroTimeStampPeriod: RET_SCALAR(UInt32, kZeroTimeStampPeriod);
        case kAudioDevicePropertyNominalSampleRate: RET_SCALAR(Float64, gSampleRate);
        case kAudioDevicePropertySafetyOffset:
            RET_SCALAR(UInt32, in ? US122EngineInputSafetyOffset((uint32_t)gSampleRate)
                                  : US122EngineOutputSafetyOffset((uint32_t)gSampleRate));
        case kAudioDevicePropertyAvailableNominalSampleRates: {
            AudioValueRange r[kNumRates];
            for (UInt32 i = 0; i < kNumRates; i++) r[i] = (AudioValueRange){kRates[i], kRates[i]};
            RET_ARRAY(AudioValueRange, r, kNumRates);
        }
        case kAudioDevicePropertyPreferredChannelsForStereo: {
            UInt32 ch[2] = {1, 2};
            RET_ARRAY(UInt32, ch, 2);
        }
        case kAudioObjectPropertyControlList: {
            AudioObjectID c[2] = {kObjectID_Volume, kObjectID_Mute};
            RET_ARRAY(AudioObjectID, c, 2);
        }
        case kAudioObjectPropertyOwnedObjects: {
            AudioObjectID o[4];
            UInt32 n = 0;
            if (!out) o[n++] = kObjectID_StreamIn;
            if (!in) { o[n++] = kObjectID_StreamOut; o[n++] = kObjectID_Volume; o[n++] = kObjectID_Mute; }
            RET_ARRAY(AudioObjectID, o, n);
        }
        case kAudioDevicePropertyStreams: {
            AudioObjectID s[2];
            UInt32 n = 0;
            if (!out) s[n++] = kObjectID_StreamIn;
            if (!in) s[n++] = kObjectID_StreamOut;
            RET_ARRAY(AudioObjectID, s, n);
        }
        }
        break;
    }

    case kObjectID_StreamIn:
    case kObjectID_StreamOut: {
        bool isIn = obj == kObjectID_StreamIn;
        switch (a->mSelector) {
        case kAudioObjectPropertyBaseClass: RET_SCALAR(AudioClassID, kAudioObjectClassID);
        case kAudioObjectPropertyClass: RET_SCALAR(AudioClassID, kAudioStreamClassID);
        case kAudioObjectPropertyOwner: RET_SCALAR(AudioObjectID, kObjectID_Device);
        case kAudioObjectPropertyName: RET_SCALAR(CFStringRef, isIn ? CFSTR("Inputs") : CFSTR("Outputs"));
        case kAudioObjectPropertyOwnedObjects: RET_ARRAY(AudioObjectID, NULL, 0);
        case kAudioStreamPropertyIsActive: RET_SCALAR(UInt32, gStreamActive[isIn ? 0 : 1]);
        case kAudioStreamPropertyDirection: RET_SCALAR(UInt32, isIn ? 1 : 0);
        case kAudioStreamPropertyTerminalType:
            RET_SCALAR(UInt32, isIn ? kAudioStreamTerminalTypeLine : kAudioStreamTerminalTypeHeadphones);
        case kAudioStreamPropertyStartingChannel: RET_SCALAR(UInt32, 1);
        case kAudioStreamPropertyLatency: RET_SCALAR(UInt32, 0);
        case kAudioStreamPropertyVirtualFormat:
        case kAudioStreamPropertyPhysicalFormat: RET_SCALAR(AudioStreamBasicDescription, CurrentFormat());
        case kAudioStreamPropertyAvailableVirtualFormats:
        case kAudioStreamPropertyAvailablePhysicalFormats: {
            AudioStreamRangedDescription d[kNumRates];
            for (UInt32 i = 0; i < kNumRates; i++) {
                d[i].mFormat = CurrentFormat();
                d[i].mFormat.mSampleRate = kRates[i];
                d[i].mSampleRateRange = (AudioValueRange){kRates[i], kRates[i]};
            }
            RET_ARRAY(AudioStreamRangedDescription, d, kNumRates);
        }
        }
        break;
    }
    case kObjectID_Volume:
        switch (a->mSelector) {
        case kAudioObjectPropertyBaseClass: RET_SCALAR(AudioClassID, kAudioLevelControlClassID);
        case kAudioObjectPropertyClass: RET_SCALAR(AudioClassID, kAudioVolumeControlClassID);
        case kAudioObjectPropertyOwner: RET_SCALAR(AudioObjectID, kObjectID_Device);
        case kAudioObjectPropertyOwnedObjects: RET_ARRAY(AudioObjectID, NULL, 0);
        case kAudioControlPropertyScope: RET_SCALAR(AudioObjectPropertyScope, kAudioObjectPropertyScopeOutput);
        case kAudioControlPropertyElement: RET_SCALAR(AudioObjectPropertyElement, kAudioObjectPropertyElementMain);
        case kAudioLevelControlPropertyScalarValue: RET_SCALAR(Float32, atomic_load(&gVolumeScalar));
        case kAudioLevelControlPropertyDecibelValue: RET_SCALAR(Float32, ScalarToDB(atomic_load(&gVolumeScalar)));
        case kAudioLevelControlPropertyDecibelRange: RET_SCALAR(AudioValueRange, ((AudioValueRange){kMinDB, 0.0}));
        case kAudioLevelControlPropertyConvertScalarToDecibels:
        case kAudioLevelControlPropertyConvertDecibelsToScalar: {
            // in/out: caller passes the value to convert in outData
            Float32 v = 0;
            if (outData) {
                if (inDataSize < sizeof(Float32)) return kAudioHardwareBadPropertySizeError;
                v = *(Float32 *)outData;
                v = a->mSelector == kAudioLevelControlPropertyConvertScalarToDecibels ? ScalarToDB(v) : DBToScalar(v);
            }
            RET_SCALAR(Float32, v);
        }
        }
        break;

    case kObjectID_Mute:
        switch (a->mSelector) {
        case kAudioObjectPropertyBaseClass: RET_SCALAR(AudioClassID, kAudioBooleanControlClassID);
        case kAudioObjectPropertyClass: RET_SCALAR(AudioClassID, kAudioMuteControlClassID);
        case kAudioObjectPropertyOwner: RET_SCALAR(AudioObjectID, kObjectID_Device);
        case kAudioObjectPropertyOwnedObjects: RET_ARRAY(AudioObjectID, NULL, 0);
        case kAudioControlPropertyScope: RET_SCALAR(AudioObjectPropertyScope, kAudioObjectPropertyScopeOutput);
        case kAudioControlPropertyElement: RET_SCALAR(AudioObjectPropertyElement, kAudioObjectPropertyElementMain);
        case kAudioBooleanControlPropertyValue: RET_SCALAR(UInt32, atomic_load(&gMute) ? 1 : 0);
        }
        break;

    default:
        return kAudioHardwareBadObjectError;
    }
    return kAudioHardwareUnknownPropertyError;
}

static OSStatus RequestRate(Float64 rate) {
    bool ok = false;
    for (UInt32 i = 0; i < kNumRates; i++) ok |= rate == kRates[i];
    if (!ok) return kAudioDeviceUnsupportedFormatError;
    if (rate != gSampleRate)
        gHost->RequestDeviceConfigurationChange(gHost, kObjectID_Device, (UInt64)rate, NULL);
    return kAudioHardwareNoError;
}

// ---------------------------------------------------------------- COM boilerplate

static HRESULT QueryInterface(void *drv, REFIID iid, LPVOID *out);
static ULONG AddRef(void *drv);
static ULONG Release(void *drv);
static OSStatus Initialize(AudioServerPlugInDriverRef drv, AudioServerPlugInHostRef host);
static OSStatus CreateDevice(AudioServerPlugInDriverRef, CFDictionaryRef, const AudioServerPlugInClientInfo *, AudioObjectID *);
static OSStatus DestroyDevice(AudioServerPlugInDriverRef, AudioObjectID);
static OSStatus AddDeviceClient(AudioServerPlugInDriverRef, AudioObjectID, const AudioServerPlugInClientInfo *);
static OSStatus RemoveDeviceClient(AudioServerPlugInDriverRef, AudioObjectID, const AudioServerPlugInClientInfo *);
static OSStatus PerformDeviceConfigurationChange(AudioServerPlugInDriverRef, AudioObjectID, UInt64, void *);
static OSStatus AbortDeviceConfigurationChange(AudioServerPlugInDriverRef, AudioObjectID, UInt64, void *);
static Boolean HasProperty(AudioServerPlugInDriverRef, AudioObjectID, pid_t, const AudioObjectPropertyAddress *);
static OSStatus IsPropertySettable(AudioServerPlugInDriverRef, AudioObjectID, pid_t, const AudioObjectPropertyAddress *, Boolean *);
static OSStatus GetPropertyDataSize(AudioServerPlugInDriverRef, AudioObjectID, pid_t, const AudioObjectPropertyAddress *, UInt32, const void *, UInt32 *);
static OSStatus GetPropertyData(AudioServerPlugInDriverRef, AudioObjectID, pid_t, const AudioObjectPropertyAddress *, UInt32, const void *, UInt32, UInt32 *, void *);
static OSStatus SetPropertyData(AudioServerPlugInDriverRef, AudioObjectID, pid_t, const AudioObjectPropertyAddress *, UInt32, const void *, UInt32, const void *);
static OSStatus StartIO(AudioServerPlugInDriverRef, AudioObjectID, UInt32);
static OSStatus StopIO(AudioServerPlugInDriverRef, AudioObjectID, UInt32);
static OSStatus GetZeroTimeStamp(AudioServerPlugInDriverRef, AudioObjectID, UInt32, Float64 *, UInt64 *, UInt64 *);
static OSStatus WillDoIOOperation(AudioServerPlugInDriverRef, AudioObjectID, UInt32, UInt32, Boolean *, Boolean *);
static OSStatus BeginIOOperation(AudioServerPlugInDriverRef, AudioObjectID, UInt32, UInt32, UInt32, const AudioServerPlugInIOCycleInfo *);
static OSStatus DoIOOperation(AudioServerPlugInDriverRef, AudioObjectID, AudioObjectID, UInt32, UInt32, UInt32, const AudioServerPlugInIOCycleInfo *, void *, void *);
static OSStatus EndIOOperation(AudioServerPlugInDriverRef, AudioObjectID, UInt32, UInt32, UInt32, const AudioServerPlugInIOCycleInfo *);

static AudioServerPlugInDriverInterface gInterface = {
    NULL, QueryInterface, AddRef, Release, Initialize, CreateDevice, DestroyDevice, AddDeviceClient,
    RemoveDeviceClient, PerformDeviceConfigurationChange, AbortDeviceConfigurationChange, HasProperty,
    IsPropertySettable, GetPropertyDataSize, GetPropertyData, SetPropertyData, StartIO, StopIO,
    GetZeroTimeStamp, WillDoIOOperation, BeginIOOperation, DoIOOperation, EndIOOperation,
};
static AudioServerPlugInDriverInterface *gInterfacePtr = &gInterface;
static AudioServerPlugInDriverRef gDriverRef = &gInterfacePtr;

__attribute__((visibility("default"))) void *US122_Create(CFAllocatorRef alloc, CFUUIDRef typeUUID) {
    if (CFEqual(typeUUID, kAudioServerPlugInTypeUUID)) return gDriverRef;
    return NULL;
}

static HRESULT QueryInterface(void *drv, REFIID iid, LPVOID *out) {
    if (drv != gDriverRef || !out) return kAudioHardwareIllegalOperationError;
    CFUUIDRef req = CFUUIDCreateFromUUIDBytes(NULL, iid);
    HRESULT r = E_NOINTERFACE;
    if (CFEqual(req, IUnknownUUID) || CFEqual(req, kAudioServerPlugInDriverInterfaceUUID)) {
        atomic_fetch_add(&gRefCount, 1);
        *out = gDriverRef;
        r = S_OK;
    }
    CFRelease(req);
    return r;
}

static ULONG AddRef(void *drv) { return atomic_fetch_add(&gRefCount, 1) + 1; }
static ULONG Release(void *drv) {
    UInt32 c = atomic_load(&gRefCount);
    if (c > 0) c = atomic_fetch_sub(&gRefCount, 1) - 1;
    return c;
}

static OSStatus Initialize(AudioServerPlugInDriverRef drv, AudioServerPlugInHostRef host) {
    gHost = host;
    gLog = os_log_create("local.us122mkii.driver", "driver");
    Float64 v;
    if (LoadNumber(kKeyVolume, &v) && v >= 0 && v <= 1) atomic_store(&gVolumeScalar, (float)v);
    if (LoadNumber(kKeyMute, &v)) atomic_store(&gMute, v != 0);
    if (LoadNumber(kKeyRate, &v) && US122EngineIsSupportedRate((uint32_t)v)) gSampleRate = v;
    US122EngineInit(PresenceCallback, NULL, LogCallback);
    SetupMIDI();
    os_log(gLog, "initialised; device %{public}s, volume %.2f%{public}s, %.0f Hz",
           US122EngineIsPresent() ? "present" : "absent", atomic_load(&gVolumeScalar),
           atomic_load(&gMute) ? " (muted)" : "", gSampleRate);
    return kAudioHardwareNoError;
}

static OSStatus CreateDevice(AudioServerPlugInDriverRef d, CFDictionaryRef desc, const AudioServerPlugInClientInfo *ci, AudioObjectID *out) {
    return kAudioHardwareUnsupportedOperationError;
}
static OSStatus DestroyDevice(AudioServerPlugInDriverRef d, AudioObjectID dev) { return kAudioHardwareUnsupportedOperationError; }
static OSStatus AddDeviceClient(AudioServerPlugInDriverRef d, AudioObjectID dev, const AudioServerPlugInClientInfo *ci) { return kAudioHardwareNoError; }
static OSStatus RemoveDeviceClient(AudioServerPlugInDriverRef d, AudioObjectID dev, const AudioServerPlugInClientInfo *ci) { return kAudioHardwareNoError; }

static OSStatus PerformDeviceConfigurationChange(AudioServerPlugInDriverRef d, AudioObjectID dev, UInt64 action, void *info) {
    if (dev != kObjectID_Device) return kAudioHardwareBadDeviceError;
    pthread_mutex_lock(&gLock);
    OSStatus st = kAudioHardwareNoError;
    gSampleRate = (Float64)action;
    StoreNumber(kKeyRate, gSampleRate);
    os_log(gLog, "sample rate -> %u", (unsigned)action);
    if (gIOCount > 0) {
        US122EngineStop();
        st = US122EngineStart((uint32_t)gSampleRate);
    }
    pthread_mutex_unlock(&gLock);
    return st;
}

static OSStatus AbortDeviceConfigurationChange(AudioServerPlugInDriverRef d, AudioObjectID dev, UInt64 action, void *info) {
    return kAudioHardwareNoError;
}

// ---------------------------------------------------------------- properties

static Boolean HasProperty(AudioServerPlugInDriverRef d, AudioObjectID obj, pid_t pid, const AudioObjectPropertyAddress *a) {
    UInt32 size = 0;
    return GetProp(obj, a, 0, NULL, 0, &size, NULL) == kAudioHardwareNoError;
}

static OSStatus IsPropertySettable(AudioServerPlugInDriverRef d, AudioObjectID obj, pid_t pid,
                                   const AudioObjectPropertyAddress *a, Boolean *out) {
    if (!ValidObject(obj)) return kAudioHardwareBadObjectError;
    UInt32 size = 0;
    OSStatus st = GetProp(obj, a, 0, NULL, 0, &size, NULL);
    if (st) return st;
    *out = (obj == kObjectID_Volume && (a->mSelector == kAudioLevelControlPropertyScalarValue ||
                                         a->mSelector == kAudioLevelControlPropertyDecibelValue)) ||
           (obj == kObjectID_Mute && a->mSelector == kAudioBooleanControlPropertyValue) ||
           (obj == kObjectID_Device && a->mSelector == kAudioDevicePropertyNominalSampleRate) ||
           (IsStream(obj) && (a->mSelector == kAudioStreamPropertyVirtualFormat ||
                              a->mSelector == kAudioStreamPropertyPhysicalFormat ||
                              a->mSelector == kAudioStreamPropertyIsActive));
    return kAudioHardwareNoError;
}

static OSStatus GetPropertyDataSize(AudioServerPlugInDriverRef d, AudioObjectID obj, pid_t pid,
                                    const AudioObjectPropertyAddress *a, UInt32 qs, const void *q, UInt32 *out) {
    return GetProp(obj, a, qs, q, 0, out, NULL);
}

static OSStatus GetPropertyData(AudioServerPlugInDriverRef d, AudioObjectID obj, pid_t pid,
                                const AudioObjectPropertyAddress *a, UInt32 qs, const void *q, UInt32 inSize,
                                UInt32 *outSize, void *outData) {
    if (!outData) return kAudioHardwareIllegalOperationError;
    return GetProp(obj, a, qs, q, inSize, outSize, outData);
}

static OSStatus SetPropertyData(AudioServerPlugInDriverRef d, AudioObjectID obj, pid_t pid,
                                const AudioObjectPropertyAddress *a, UInt32 qs, const void *q, UInt32 inSize,
                                const void *data) {
    if (obj == kObjectID_Volume && (a->mSelector == kAudioLevelControlPropertyScalarValue ||
                                    a->mSelector == kAudioLevelControlPropertyDecibelValue)) {
        if (inSize != sizeof(Float32)) return kAudioHardwareBadPropertySizeError;
        Float32 v = *(const Float32 *)data;
        if (a->mSelector == kAudioLevelControlPropertyDecibelValue) v = DBToScalar(v);
        v = v < 0 ? 0 : (v > 1 ? 1 : v);
        if (v != atomic_load(&gVolumeScalar)) {
            atomic_store(&gVolumeScalar, v);
            StoreNumber(kKeyVolume, v);
            AudioObjectPropertyAddress ch[2] = {
                {kAudioLevelControlPropertyScalarValue, kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain},
                {kAudioLevelControlPropertyDecibelValue, kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain},
            };
            gHost->PropertiesChanged(gHost, kObjectID_Volume, 2, ch);
        }
        return kAudioHardwareNoError;
    }
    if (obj == kObjectID_Mute && a->mSelector == kAudioBooleanControlPropertyValue) {
        if (inSize != sizeof(UInt32)) return kAudioHardwareBadPropertySizeError;
        bool m = *(const UInt32 *)data != 0;
        if (m != atomic_load(&gMute)) {
            atomic_store(&gMute, m);
            StoreNumber(kKeyMute, m ? 1 : 0);
            AudioObjectPropertyAddress ch = {kAudioBooleanControlPropertyValue, kAudioObjectPropertyScopeGlobal,
                                             kAudioObjectPropertyElementMain};
            gHost->PropertiesChanged(gHost, kObjectID_Mute, 1, &ch);
        }
        return kAudioHardwareNoError;
    }
    if (obj == kObjectID_Device && a->mSelector == kAudioDevicePropertyNominalSampleRate) {
        if (inSize != sizeof(Float64)) return kAudioHardwareBadPropertySizeError;
        return RequestRate(*(const Float64 *)data);
    }
    if (IsStream(obj)) {
        if (a->mSelector == kAudioStreamPropertyIsActive) {
            if (inSize != sizeof(UInt32)) return kAudioHardwareBadPropertySizeError;
            bool v = *(const UInt32 *)data != 0;
            gStreamActive[obj == kObjectID_StreamIn ? 0 : 1] = v;
            AudioObjectPropertyAddress ad = {kAudioStreamPropertyIsActive, kAudioObjectPropertyScopeGlobal,
                                             kAudioObjectPropertyElementMain};
            gHost->PropertiesChanged(gHost, obj, 1, &ad);
            return kAudioHardwareNoError;
        }
        if (a->mSelector == kAudioStreamPropertyVirtualFormat || a->mSelector == kAudioStreamPropertyPhysicalFormat) {
            if (inSize != sizeof(AudioStreamBasicDescription)) return kAudioHardwareBadPropertySizeError;
            const AudioStreamBasicDescription *f = data;
            AudioStreamBasicDescription cur = CurrentFormat();
            if (f->mFormatID != cur.mFormatID || f->mChannelsPerFrame != 2 || f->mBitsPerChannel != 32 ||
                !(f->mFormatFlags & kAudioFormatFlagIsFloat))
                return kAudioDeviceUnsupportedFormatError;
            return RequestRate(f->mSampleRate);
        }
    }
    if (!ValidObject(obj)) return kAudioHardwareBadObjectError;
    return kAudioHardwareUnsupportedOperationError;
}

// ---------------------------------------------------------------- IO

static OSStatus StartIO(AudioServerPlugInDriverRef d, AudioObjectID dev, UInt32 client) {
    if (dev != kObjectID_Device) return kAudioHardwareBadObjectError;
    pthread_mutex_lock(&gLock);
    OSStatus st = kAudioHardwareNoError;
    if (gIOCount == 0) {
        st = US122EngineStart((uint32_t)gSampleRate);
        if (st) os_log_error(gLog, "StartIO failed: %d", (int)st);
    }
    if (!st) gIOCount++;
    pthread_mutex_unlock(&gLock);
    return st;
}

static OSStatus StopIO(AudioServerPlugInDriverRef d, AudioObjectID dev, UInt32 client) {
    if (dev != kObjectID_Device) return kAudioHardwareBadObjectError;
    pthread_mutex_lock(&gLock);
    if (gIOCount > 0 && --gIOCount == 0) US122EngineStop();
    pthread_mutex_unlock(&gLock);
    return kAudioHardwareNoError;
}

static OSStatus GetZeroTimeStamp(AudioServerPlugInDriverRef d, AudioObjectID dev, UInt32 client, Float64 *sampleTime,
                                 UInt64 *hostTime, UInt64 *seed) {
    double s;
    uint64_t h;
    if (!US122EngineGetZeroTimeStamp(kZeroTimeStampPeriod, &s, &h)) return kAudioHardwareNotRunningError;
    *sampleTime = s;
    *hostTime = h;
    *seed = 1;
    return kAudioHardwareNoError;
}

static OSStatus WillDoIOOperation(AudioServerPlugInDriverRef d, AudioObjectID dev, UInt32 client, UInt32 op,
                                  Boolean *willDo, Boolean *inPlace) {
    *willDo = op == kAudioServerPlugInIOOperationReadInput || op == kAudioServerPlugInIOOperationWriteMix;
    *inPlace = true;
    return kAudioHardwareNoError;
}

static OSStatus BeginIOOperation(AudioServerPlugInDriverRef d, AudioObjectID dev, UInt32 client, UInt32 op, UInt32 frames,
                                 const AudioServerPlugInIOCycleInfo *ci) {
    return kAudioHardwareNoError;
}

static OSStatus DoIOOperation(AudioServerPlugInDriverRef d, AudioObjectID dev, AudioObjectID stream, UInt32 client,
                              UInt32 op, UInt32 frames, const AudioServerPlugInIOCycleInfo *ci, void *main, void *secondary) {
    if (op == kAudioServerPlugInIOOperationReadInput && stream == kObjectID_StreamIn)
        US122EngineReadInput((int64_t)floor(ci->mInputTime.mSampleTime), frames, main);
    else if (op == kAudioServerPlugInIOOperationWriteMix && stream == kObjectID_StreamOut) {
        // software volume, ramped linearly across the buffer to avoid zipper noise
        float *buf = main;
        float target = TargetGain();
        float start = gAppliedGain < 0 ? target : gAppliedGain;
        if (start != 1.0f || target != 1.0f) {
            float step = frames ? (target - start) / frames : 0;
            float g = start;
            for (UInt32 f = 0; f < frames; f++, g += step) {
                buf[2 * f] *= g;
                buf[2 * f + 1] *= g;
            }
        }
        gAppliedGain = target;
        US122EngineWriteOutput((int64_t)floor(ci->mOutputTime.mSampleTime), frames, buf);
    }
    return kAudioHardwareNoError;
}

static OSStatus EndIOOperation(AudioServerPlugInDriverRef d, AudioObjectID dev, UInt32 client, UInt32 op, UInt32 frames,
                               const AudioServerPlugInIOCycleInfo *ci) {
    return kAudioHardwareNoError;
}
