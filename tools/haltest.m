// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Oliver Frommel

// haltest — exercises the installed US-122MKII Core Audio device through the
// HAL (AudioDeviceIOProc): plays a sine, measures input level and checks the
// IO timeline for discontinuities (skipped/overloaded cycles).
//
//   haltest [--seconds 10] [--freq 440] [--level -24] [--rate 48000] [--buffer 512]

#import <Foundation/Foundation.h>
#import <CoreAudio/CoreAudio.h>
#include <math.h>
#include <stdatomic.h>

static double gPhase, gInc, gAmp;
static _Atomic uint64_t gCycles, gDiscontinuities, gOverloads;
static double gLastOutSample = -1, gLastInSample = -1;
static float gPeakL, gPeakR;

static OSStatus IOProc(AudioObjectID dev, const AudioTimeStamp *now, const AudioBufferList *in,
                       const AudioTimeStamp *inTime, AudioBufferList *out, const AudioTimeStamp *outTime,
                       void *ctx) {
    gCycles++;
    if (out && out->mNumberBuffers) {
        float *o = out->mBuffers[0].mData;
        UInt32 ch = out->mBuffers[0].mNumberChannels;
        UInt32 frames = out->mBuffers[0].mDataByteSize / (ch * sizeof(float));
        if (gLastOutSample >= 0 && outTime->mSampleTime != gLastOutSample + frames) gDiscontinuities++;
        gLastOutSample = outTime->mSampleTime;
        for (UInt32 f = 0; f < frames; f++) {
            float v = (float)(sin(gPhase) * gAmp);
            gPhase += gInc;
            if (gPhase > 2 * M_PI) gPhase -= 2 * M_PI;
            for (UInt32 c = 0; c < ch; c++) o[f * ch + c] = v;
        }
    }
    if (in && in->mNumberBuffers) {
        const float *i = in->mBuffers[0].mData;
        UInt32 ch = in->mBuffers[0].mNumberChannels;
        UInt32 frames = in->mBuffers[0].mDataByteSize / (ch * sizeof(float));
        if (gLastInSample >= 0 && inTime->mSampleTime != gLastInSample + frames) gDiscontinuities++;
        gLastInSample = inTime->mSampleTime;
        for (UInt32 f = 0; f < frames; f++) {
            gPeakL = fmaxf(gPeakL, fabsf(i[f * ch]));
            if (ch > 1) gPeakR = fmaxf(gPeakR, fabsf(i[f * ch + 1]));
        }
    }
    return noErr;
}

static OSStatus OverloadListener(AudioObjectID o, UInt32 n, const AudioObjectPropertyAddress *a, void *ctx) {
    gOverloads++;
    return noErr;
}

int main(int argc, const char *argv[]) {
    @autoreleasepool {
        double seconds = 10, freq = 440, level = -24, rate = 48000;
        UInt32 buffer = 512;
        for (int i = 1; i + 1 < argc; i += 2) {
            NSString *a = @(argv[i]);
            if ([a isEqualToString:@"--seconds"]) seconds = atof(argv[i + 1]);
            else if ([a isEqualToString:@"--freq"]) freq = atof(argv[i + 1]);
            else if ([a isEqualToString:@"--level"]) level = atof(argv[i + 1]);
            else if ([a isEqualToString:@"--rate"]) rate = atof(argv[i + 1]);
            else if ([a isEqualToString:@"--buffer"]) buffer = (UInt32)atoi(argv[i + 1]);
        }

        CFStringRef uid = CFSTR("US122MKII-0644-8021");
        AudioObjectID dev = kAudioObjectUnknown;
        UInt32 size = sizeof dev;
        AudioObjectPropertyAddress a = {kAudioHardwarePropertyTranslateUIDToDevice, kAudioObjectPropertyScopeGlobal,
                                        kAudioObjectPropertyElementMain};
        OSStatus st = AudioObjectGetPropertyData(kAudioObjectSystemObject, &a, sizeof uid, &uid, &size, &dev);
        if (st || dev == kAudioObjectUnknown) { fprintf(stderr, "US-122MKII device not found (%d)\n", (int)st); return 1; }

        a = (AudioObjectPropertyAddress){kAudioDevicePropertyNominalSampleRate, kAudioObjectPropertyScopeGlobal,
                                         kAudioObjectPropertyElementMain};
        st = AudioObjectSetPropertyData(dev, &a, 0, NULL, sizeof rate, &rate);
        if (st) fprintf(stderr, "set rate: %d\n", (int)st);
        usleep(300 * 1000);
        Float64 actual = 0;
        size = sizeof actual;
        AudioObjectGetPropertyData(dev, &a, 0, NULL, &size, &actual);

        a.mSelector = kAudioDevicePropertyBufferFrameSize;
        st = AudioObjectSetPropertyData(dev, &a, 0, NULL, sizeof buffer, &buffer);
        if (st) fprintf(stderr, "set buffer size: %d\n", (int)st);

        UInt32 inLat = 0, outLat = 0, inSO = 0, outSO = 0;
        size = sizeof(UInt32);
        a = (AudioObjectPropertyAddress){kAudioDevicePropertySafetyOffset, kAudioObjectPropertyScopeInput, kAudioObjectPropertyElementMain};
        AudioObjectGetPropertyData(dev, &a, 0, NULL, &size, &inSO);
        a.mScope = kAudioObjectPropertyScopeOutput;
        AudioObjectGetPropertyData(dev, &a, 0, NULL, &size, &outSO);
        a.mSelector = kAudioDevicePropertyLatency;
        AudioObjectGetPropertyData(dev, &a, 0, NULL, &size, &outLat);
        a.mScope = kAudioObjectPropertyScopeInput;
        AudioObjectGetPropertyData(dev, &a, 0, NULL, &size, &inLat);
        printf("device %u, rate %.0f, buffer %u, safety offset in %u / out %u\n", dev, actual, buffer, inSO, outSO);

        a = (AudioObjectPropertyAddress){kAudioDeviceProcessorOverload, kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain};
        AudioObjectAddPropertyListener(dev, &a, OverloadListener, NULL);

        gInc = 2 * M_PI * freq / actual;
        gAmp = pow(10, level / 20);
        AudioDeviceIOProcID proc = NULL;
        st = AudioDeviceCreateIOProcID(dev, IOProc, NULL, &proc);
        if (st) { fprintf(stderr, "create ioproc: %d\n", (int)st); return 1; }
        st = AudioDeviceStart(dev, proc);
        if (st) { fprintf(stderr, "AudioDeviceStart: %d\n", (int)st); return 1; }

        for (int s = 0; s < (int)seconds; s++) {
            sleep(1);
            printf("t=%2ds cycles %6llu  discontinuities %llu  overloads %llu  in peak %6.1f / %6.1f dBFS\n", s + 1,
                   gCycles, gDiscontinuities, gOverloads, 20 * log10(fmax(gPeakL, 1e-9)), 20 * log10(fmax(gPeakR, 1e-9)));
            fflush(stdout);
            gPeakL = gPeakR = 0;
        }
        AudioDeviceStop(dev, proc);
        AudioDeviceDestroyIOProcID(dev, proc);
    }
    return 0;
}
