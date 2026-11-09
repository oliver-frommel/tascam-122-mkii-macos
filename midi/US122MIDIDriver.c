// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Oliver Frommel

// US122MIDIDriver — CoreMIDI driver plug-in for the TASCAM US-122MKII.
//
// Loaded by the user's MIDIServer. Publishes a "US-122MKII" MIDI device with
// one source (the device's MIDI IN jack) and one destination (MIDI OUT). The
// USB device itself belongs to the audio plug-in, which runs in Core Audio's
// sandbox; MIDI bytes travel between the two through us122-midirelay (see
// relay.c for the message format).
//
// The device only passes MIDI while its audio stream runs, and the audio
// plug-in can only stream properly while Core Audio does IO on it (its helper
// process is throttled to background priority otherwise). So while MIDI is in
// use (an app listens to MIDI IN, or MIDI went out recently) we run a silent
// Core Audio stream on the device, like any audio app would.

#include <CoreFoundation/CoreFoundation.h>
#include <CoreMIDI/CoreMIDI.h>
#include <CoreMIDI/MIDIDriver.h>
#include <CoreAudio/CoreAudio.h>
#include <mach/mach_time.h>
#include <os/log.h>
#include <xpc/xpc.h>
#include <stdatomic.h>

#define RELAY_SERVICE "local.us122mkii.midirelay"
#define AUDIO_DEVICE_UID "US122MKII-0644-8021" // see src/US122Driver.c
#define OUT_HOLD_SEC 30                         // keep streaming this long after MIDI output

static os_log_t gLog;

typedef struct {
    MIDIDriverInterface *iface;
} Driver;
static Driver gDriver;
static _Atomic UInt32 gRefCount;

static dispatch_queue_t gQ;         // relay connection + parser
static xpc_connection_t gConn;
static MIDIDeviceRef gDevice;
static MIDIEndpointRef gSource, gDest;
static bool gListen;                // a client is connected to our source (gQ)
static bool gStarted;               // gQ
static bool gPresent;               // audio side reports the device connected (gQ)
static uint64_t gLastOut;           // host time of the last MIDI output (gQ)
static AudioObjectID gAudioDev;     // device we hold a silent stream on (gQ)
static AudioDeviceIOProcID gProc;   // gQ
static dispatch_source_t gTimer;

// ---------------------------------------------------------------- input parser
//
// The device delivers a raw MIDI byte stream in small chunks that can split
// messages; CoreMIDI wants whole messages (or sysex fragments) per packet.
// Runs on gQ.

static struct {
    uint8_t status, buf[3];
    int n, need;
    bool sysex;
    uint8_t sx[256];
    int sxn;
} gParse;

static void Emit(const uint8_t *b, int len) {
    if (!gSource || len <= 0) return;
    uint8_t mem[512];
    MIDIPacketList *pl = (MIDIPacketList *)mem;
    MIDIPacket *pk = MIDIPacketListInit(pl);
    pk = MIDIPacketListAdd(pl, sizeof mem, pk, mach_absolute_time(), (ByteCount)len, b);
    if (pk) MIDIReceived(gSource, pl);
}

static void FlushSysex(void) {
    Emit(gParse.sx, gParse.sxn);
    gParse.sxn = 0;
}

static int DataBytes(uint8_t st) {
    switch (st & 0xF0) {
        case 0x80: case 0x90: case 0xA0: case 0xB0: case 0xE0: return 2;
        case 0xC0: case 0xD0: return 1;
    }
    switch (st) {
        case 0xF1: case 0xF3: return 1;
        case 0xF2: return 2;
        default: return 0;
    }
}

static void Parse(const uint8_t *bytes, size_t len) {
    for (size_t i = 0; i < len; i++) {
        uint8_t b = bytes[i];
        if (b >= 0xF8) { Emit(&b, 1); continue; } // real-time: may appear anywhere
        if (gParse.sysex) {
            if (b < 0x80 || b == 0xF7) {
                gParse.sx[gParse.sxn++] = b;
                if (b == 0xF7) { FlushSysex(); gParse.sysex = false; }
                else if (gParse.sxn == (int)sizeof gParse.sx) FlushSysex();
                continue;
            }
            FlushSysex(); // unterminated sysex cut off by a new status byte
            gParse.sysex = false;
        }
        if (b == 0xF0) { gParse.sysex = true; gParse.sxn = 0; gParse.sx[gParse.sxn++] = b; gParse.status = 0; continue; }
        if (b >= 0x80) {
            gParse.status = b;
            gParse.n = 0;
            gParse.need = DataBytes(b);
            if (!gParse.need) { Emit(&b, 1); gParse.status = 0; }
            continue;
        }
        if (!gParse.status) continue; // stray data byte
        gParse.buf[1 + gParse.n++] = b;
        if (gParse.n == gParse.need) {
            gParse.buf[0] = gParse.status;
            Emit(gParse.buf, 1 + gParse.need);
            gParse.n = 0;
            if (gParse.status >= 0xF0) gParse.status = 0; // system common: no running status
        }
    }
}

// ---------------------------------------------------------------- relay link (gQ)

// ---- silent audio stream that keeps the device's MIDI alive (gQ)

static void ReleaseHold(void) {
    if (!gProc) return;
    AudioDeviceStop(gAudioDev, gProc);
    AudioDeviceDestroyIOProcID(gAudioDev, gProc);
    gProc = NULL;
    gAudioDev = 0;
    os_log(gLog, "audio hold released");
}

static void TakeHold(void) {
    if (gProc) return;
    CFStringRef uid = CFSTR(AUDIO_DEVICE_UID);
    AudioObjectID dev = 0;
    UInt32 sz = sizeof dev;
    AudioObjectPropertyAddress a = {kAudioHardwarePropertyTranslateUIDToDevice, kAudioObjectPropertyScopeGlobal,
                                    kAudioObjectPropertyElementMain};
    OSStatus st = AudioObjectGetPropertyData(kAudioObjectSystemObject, &a, sizeof uid, &uid, &sz, &dev);
    if (st || !dev) { os_log(gLog, "audio hold: device not found (%d)", (int)st); return; }
    AudioDeviceIOProcID proc = NULL;
    // output buffers arrive zeroed: leaving them alone plays silence
    st = AudioDeviceCreateIOProcIDWithBlock(&proc, dev, NULL,
        ^(const AudioTimeStamp *now, const AudioBufferList *in, const AudioTimeStamp *inTime,
          AudioBufferList *out, const AudioTimeStamp *outTime) {});
    if (st) { os_log_error(gLog, "audio hold: IOProc create failed (%d)", (int)st); return; }
    st = AudioDeviceStart(dev, proc);
    if (st) {
        os_log_error(gLog, "audio hold: start failed (%d)", (int)st);
        AudioDeviceDestroyIOProcID(dev, proc);
        return;
    }
    gAudioDev = dev;
    gProc = proc;
    os_log(gLog, "audio hold taken");
}

static void UpdateHold(void) {
    mach_timebase_info_data_t tb;
    mach_timebase_info(&tb);
    uint64_t hold = (uint64_t)OUT_HOLD_SEC * 1000000000ull * tb.denom / tb.numer;
    bool recentOut = gLastOut && mach_absolute_time() - gLastOut < hold;
    if (gStarted && gPresent && (gListen || recentOut)) TakeHold();
    else ReleaseHold();
}

static void SetOnline(bool on) {
    MIDIDeviceRef dev = gDevice;
    dispatch_async(dispatch_get_main_queue(), ^{
        if (dev) MIDIObjectSetIntegerProperty(dev, kMIDIPropertyOffline, on ? 0 : 1);
    });
    os_log(gLog, "device %{public}s", on ? "online" : "offline");
    gPresent = on;
    if (!on) ReleaseHold(); // the Core Audio device is going away
    UpdateHold();
}

static void Connect(void);

static void Hello(void) {
    xpc_object_t m = xpc_dictionary_create(NULL, NULL, 0);
    xpc_dictionary_set_string(m, "role", "midi");
    xpc_connection_send_message(gConn, m);
}

static void Connect(void) {
    if (!gStarted || gConn) return;
    xpc_connection_t c = xpc_connection_create_mach_service(RELAY_SERVICE, gQ, XPC_CONNECTION_MACH_SERVICE_PRIVILEGED);
    gConn = c;
    xpc_connection_set_event_handler(c, ^(xpc_object_t msg) {
        if (xpc_get_type(msg) == XPC_TYPE_DICTIONARY) {
            size_t n = 0;
            const void *in = xpc_dictionary_get_data(msg, "in", &n);
            if (in && n) Parse(in, n);
            if (xpc_dictionary_get_value(msg, "present")) SetOnline(xpc_dictionary_get_bool(msg, "present"));
        } else if (msg == XPC_ERROR_CONNECTION_INTERRUPTED) {
            // relay restarted: launchd relaunches it on our next message
            SetOnline(false);
            if (c == gConn) Hello();
        } else if (msg == XPC_ERROR_CONNECTION_INVALID) {
            if (c != gConn) return; // cancelled by Stop
            os_log_error(gLog, "relay unavailable; retrying");
            gConn = NULL;
            SetOnline(false);
            dispatch_after(dispatch_time(DISPATCH_TIME_NOW, 5 * NSEC_PER_SEC), gQ, ^{ Connect(); });
        }
    });
    xpc_connection_resume(c);
    Hello();
}

// ---------------------------------------------------------------- driver interface

static HRESULT QueryInterface(void *self, REFIID iid, LPVOID *out) {
    CFUUIDRef u = CFUUIDCreateFromUUIDBytes(NULL, iid);
    bool ok = CFEqual(u, IUnknownUUID) || CFEqual(u, kMIDIDriverInterface2ID);
    CFRelease(u);
    if (!ok) { *out = NULL; return E_NOINTERFACE; }
    atomic_fetch_add(&gRefCount, 1);
    *out = &gDriver;
    return S_OK;
}
static ULONG AddRef(void *self) { return atomic_fetch_add(&gRefCount, 1) + 1; }
static ULONG Release(void *self) { return atomic_fetch_sub(&gRefCount, 1) - 1; }

static OSStatus FindDevices(MIDIDriverRef self, MIDIDeviceListRef devList) { return noErr; } // version 1 only

static OSStatus Start(MIDIDriverRef self, MIDIDeviceListRef devList) {
    // version 2: devList holds our device from the saved MIDI setup, if any
    MIDIDeviceRef dev = 0;
    if (MIDIDeviceListGetNumberOfDevices(devList) > 0) dev = MIDIDeviceListGetDevice(devList, 0);
    if (!dev) {
        OSStatus st = MIDIDeviceCreate(self, CFSTR("US-122MKII"), CFSTR("TASCAM"), CFSTR("US-122MKII"), &dev);
        if (st) { os_log_error(gLog, "MIDIDeviceCreate: %d", (int)st); return st; }
        MIDIEntityRef e;
        st = MIDIDeviceNewEntity(dev, CFSTR("MIDI"), kMIDIProtocol_1_0, false, 1, 1, &e);
        if (st) { os_log_error(gLog, "MIDIDeviceNewEntity: %d", (int)st); return st; }
        st = MIDISetupAddDevice(dev);
        if (st) { os_log_error(gLog, "MIDISetupAddDevice: %d", (int)st); return st; }
        os_log(gLog, "created device");
    }
    MIDIEntityRef ent = MIDIDeviceGetNumberOfEntities(dev) ? MIDIDeviceGetEntity(dev, 0) : 0;
    MIDIEndpointRef src = ent && MIDIEntityGetNumberOfSources(ent) ? MIDIEntityGetSource(ent, 0) : 0;
    MIDIEndpointRef dst = ent && MIDIEntityGetNumberOfDestinations(ent) ? MIDIEntityGetDestination(ent, 0) : 0;
    if (!src || !dst) { os_log_error(gLog, "device has no MIDI endpoints"); return kMIDIUnknownEndpoint; }
    MIDIObjectSetIntegerProperty(dev, kMIDIPropertyOffline, 1); // until the relay says otherwise
    dispatch_sync(gQ, ^{
        gDevice = dev;
        gSource = src;
        gDest = dst;
        gStarted = true;
        Connect();
        if (!gTimer) { // expire the output hold
            gTimer = dispatch_source_create(DISPATCH_SOURCE_TYPE_TIMER, 0, 0, gQ);
            dispatch_source_set_timer(gTimer, dispatch_time(DISPATCH_TIME_NOW, 5 * NSEC_PER_SEC), 5 * NSEC_PER_SEC,
                                      NSEC_PER_SEC);
            dispatch_source_set_event_handler(gTimer, ^{ UpdateHold(); });
            dispatch_resume(gTimer);
        }
    });
    os_log(gLog, "started");
    return noErr;
}

static OSStatus Stop(MIDIDriverRef self) {
    dispatch_sync(gQ, ^{
        gStarted = false;
        ReleaseHold();
        if (gConn) { xpc_connection_t c = gConn; gConn = NULL; xpc_connection_cancel(c); }
        gSource = gDest = 0;
        gDevice = 0;
    });
    os_log(gLog, "stopped");
    return noErr;
}

static OSStatus Configure(MIDIDriverRef self, MIDIDeviceRef device) { return noErr; }

static OSStatus Send(MIDIDriverRef self, const MIDIPacketList *pl, void *r1, void *r2) {
    uint8_t buf[4096];
    size_t n = 0;
    const MIDIPacket *p = &pl->packet[0];
    for (UInt32 i = 0; i < pl->numPackets; i++) {
        if (n + p->length > sizeof buf) break;
        memcpy(buf + n, p->data, p->length);
        n += p->length;
        p = MIDIPacketNext(p);
    }
    if (!n) return noErr;
    xpc_object_t m = xpc_dictionary_create(NULL, NULL, 0);
    xpc_dictionary_set_data(m, "out", buf, n);
    dispatch_async(gQ, ^{
        gLastOut = mach_absolute_time();
        UpdateHold(); // bytes queue in the audio plug-in until its stream runs
        if (gConn) xpc_connection_send_message(gConn, m);
    });
    return noErr;
}

static OSStatus EnableSource(MIDIDriverRef self, MIDIEndpointRef src, Boolean on) {
    dispatch_async(gQ, ^{
        if (gListen == (bool)on) return;
        gListen = on;
        os_log(gLog, "input %{public}s", on ? "has listeners" : "idle");
        UpdateHold();
    });
    return noErr;
}

static OSStatus Flush(MIDIDriverRef self, MIDIEndpointRef dest, void *r1, void *r2) { return noErr; }
static OSStatus Monitor(MIDIDriverRef self, MIDIEndpointRef dest, const MIDIPacketList *pl) { return noErr; }

static MIDIDriverInterface gInterface = {
    NULL, QueryInterface, AddRef, Release,
    FindDevices, Start, Stop, Configure, Send, EnableSource, Flush, Monitor,
    NULL, NULL, // SendPackets / MonitorEvents: version 3 only
};

__attribute__((visibility("default")))
void *US122MIDI_Create(CFAllocatorRef alloc, CFUUIDRef typeID) {
    if (!CFEqual(typeID, kMIDIDriverTypeID)) return NULL;
    gLog = os_log_create("local.us122mkii.driver", "midi");
    gQ = dispatch_queue_create("local.us122mkii.midi", DISPATCH_QUEUE_SERIAL);
    gDriver.iface = &gInterface;
    atomic_store(&gRefCount, 1);
    return &gDriver;
}
