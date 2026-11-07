// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Oliver Frommel

// enginetest — drives US122Engine the way the Core Audio HAL would:
// an IO "cycle" every buffer period that writes output ahead of the device
// position (output safety offset) and reads input behind it (input safety
// offset), using only the engine's clock model.
//
//   enginetest [--rate 48000] [--seconds 10] [--freq 440] [--level -24]
//              [--buffer 512] [--record in.wav]

#import <Foundation/Foundation.h>
#include <mach/mach_time.h>
#include <math.h>
#include "../src/US122Engine.h"
#include <pthread.h>
#include <mach/thread_policy.h>
#include <mach/thread_act.h>
#include <stdatomic.h>

// --probe: a dedicated real-time thread that wakes every 1 ms and records how
// late it was, to tell system-wide scheduling stalls from GCD/USB delivery ones.
static _Atomic uint64_t gProbeMaxLate;

static void midiIn(void *ctx, const uint8_t *b, size_t n) {
    printf("MIDI in:");
    for (size_t i = 0; i < n; i++) printf(" %02X", b[i]);
    printf("\n");
    fflush(stdout);
}
static void *probeThread(void *arg) {
    mach_timebase_info_data_t tb; mach_timebase_info(&tb);
    uint64_t ms = 1000000ull * tb.denom / tb.numer;
    thread_time_constraint_policy_data_t pol = {.period = (uint32_t)ms, .computation = (uint32_t)(ms / 10),
                                                .constraint = (uint32_t)(ms / 2), .preemptible = 1};
    thread_policy_set(pthread_mach_thread_np(pthread_self()), THREAD_TIME_CONSTRAINT_POLICY,
                      (thread_policy_t)&pol, THREAD_TIME_CONSTRAINT_POLICY_COUNT);
    uint64_t next = mach_absolute_time() + ms;
    for (;;) {
        mach_wait_until(next);
        uint64_t now = mach_absolute_time(), late = now - next;
        uint64_t cur = atomic_load(&gProbeMaxLate);
        while (late > cur && !atomic_compare_exchange_weak(&gProbeMaxLate, &cur, late)) {}
        next += ms;
        if (now > next) next = now + ms;
    }
    return NULL;
}

static void wavHeader(FILE *f, uint32_t rate, uint32_t dataBytes) {
    uint8_t h[44] = {0};
    memcpy(h, "RIFF", 4);
    *(uint32_t *)(h + 4) = 36 + dataBytes;
    memcpy(h + 8, "WAVEfmt ", 8);
    *(uint32_t *)(h + 16) = 16;
    *(uint16_t *)(h + 20) = 3; // IEEE float
    *(uint16_t *)(h + 22) = 2;
    *(uint32_t *)(h + 24) = rate;
    *(uint32_t *)(h + 28) = rate * 8;
    *(uint16_t *)(h + 32) = 8;
    *(uint16_t *)(h + 34) = 32;
    memcpy(h + 36, "data", 4);
    *(uint32_t *)(h + 40) = dataBytes;
    fseek(f, 0, SEEK_SET);
    fwrite(h, 1, 44, f);
}

int main(int argc, const char *argv[]) {
    @autoreleasepool {
        uint32_t rate = 48000, buf = 512;
        double seconds = 10, freq = 440, level = -24;
        const char *wavPath = NULL;
        bool midi = false;
        double killAt = -1;
        for (int i = 1; i < argc; i++) {
            NSString *a = @(argv[i]);
            BOOL v = i + 1 < argc;
            if ([a isEqualToString:@"--rate"] && v) rate = atoi(argv[++i]);
            else if ([a isEqualToString:@"--seconds"] && v) seconds = atof(argv[++i]);
            else if ([a isEqualToString:@"--freq"] && v) freq = atof(argv[++i]);
            else if ([a isEqualToString:@"--level"] && v) level = atof(argv[++i]);
            else if ([a isEqualToString:@"--buffer"] && v) buf = atoi(argv[++i]);
            else if ([a isEqualToString:@"--record"] && v) wavPath = argv[++i];
            else if ([a isEqualToString:@"--kill-at"] && v) killAt = atof(argv[++i]);
            else if ([a isEqualToString:@"--depth"] && v) setenv("US122_DEPTH", argv[++i], 1);
            else if ([a isEqualToString:@"--midi"]) midi = true;
            else if ([a isEqualToString:@"--probe"]) { pthread_t th; pthread_create(&th, NULL, probeThread, NULL); }
            else { fprintf(stderr, "bad argument %s\n", argv[i]); return 64; }
        }

        US122EngineInit(NULL, NULL, NULL);
        if (midi) US122EngineSetMidiInCallback(midiIn, NULL);
        for (int i = 0; i < 200 && !US122EngineIsPresent(); i++) usleep(10000);
        if (!US122EngineIsPresent()) { fprintf(stderr, "device not present\n"); return 1; }
        OSStatus st = US122EngineStart(rate);
        if (st) { fprintf(stderr, "start failed: %d\n", (int)st); return 1; }

        mach_timebase_info_data_t tb;
        mach_timebase_info(&tb);
        double ticksPerSec = 1e9 * tb.denom / tb.numer;
        uint32_t inSafety = US122EngineInputSafetyOffset(rate);
        if (getenv("ET_IN_SAFETY")) inSafety = atoi(getenv("ET_IN_SAFETY"));
        uint32_t outSafety = US122EngineOutputSafetyOffset(rate);
        printf("rate %u, buffer %u, safety in %u / out %u frames\n", rate, buf, inSafety, outSafety);

        FILE *wav = wavPath ? fopen(wavPath, "wb") : NULL;
        if (wav) wavHeader(wav, rate, 0);
        uint64_t wavBytes = 0;

        float *tmp = calloc(buf * 2, sizeof(float));
        double amp = pow(10, level / 20), phase = 0, inc = 2 * M_PI * freq / rate;
        int64_t outPos = INT64_MIN, inPos = INT64_MIN;
        float peakL = 0, peakR = 0;
        uint64_t late = 0;

        const uint32_t period = 16384;
        double lastZS = -1;
        uint64_t lastZH = 0;
        double worstZErrUs = 0;

        uint64_t t0 = mach_absolute_time(), nextReport = t0 + (uint64_t)ticksPerSec;
        uint64_t end = t0 + (uint64_t)(seconds * ticksPerSec);
        double lastS = -1;
        uint64_t backwards = 0;
        while (mach_absolute_time() < end) {
            uint64_t now = mach_absolute_time();
            double s = US122EngineSampleTimeAtHostTime(now);
            if (killAt >= 0 && now - t0 >= (uint64_t)(killAt * ticksPerSec)) {
                killAt = -1;
                US122EngineDebugKillStream();
            }
            if (lastS >= 0 && s < lastS) backwards++;
            lastS = s;

            // output: keep [now + safety, now + safety + buffer) written
            if (outPos == INT64_MIN) outPos = (int64_t)s + outSafety;
            if (outPos < (int64_t)s + (int64_t)outSafety / 2) late++; // fell behind the device
            while (outPos < (int64_t)s + outSafety + buf) {
                for (uint32_t f = 0; f < buf; f++) {
                    float v = (float)(sin(phase) * amp);
                    phase += inc;
                    if (phase > 2 * M_PI) phase -= 2 * M_PI;
                    tmp[2 * f] = tmp[2 * f + 1] = v;
                }
                US122EngineWriteOutput(outPos, buf, tmp);
                outPos += buf;
            }

            // input: read everything older than now - safety
            if (inPos == INT64_MIN) inPos = (int64_t)s - inSafety - buf;
            while (inPos + buf <= (int64_t)s - inSafety) {
                US122EngineReadInput(inPos, buf, tmp);
                for (uint32_t f = 0; f < buf; f++) {
                    peakL = fmaxf(peakL, fabsf(tmp[2 * f]));
                    peakR = fmaxf(peakR, fabsf(tmp[2 * f + 1]));
                }
                if (wav) { fwrite(tmp, sizeof(float), buf * 2, wav); wavBytes += buf * 8; }
                inPos += buf;
            }

            // zero timestamp consistency: successive periods must be period/rate apart
            double zs; uint64_t zh;
            if (US122EngineGetZeroTimeStamp(period, &zs, &zh)) {
                if (lastZS >= 0 && zs == lastZS) {
                    double errUs = ((double)zh - (double)lastZH) / ticksPerSec * 1e6;
                    worstZErrUs = fmax(worstZErrUs, fabs(errUs));
                } else if (lastZS >= 0 && zs == lastZS + period) {
                    // host-time spacing vs nominal, informational
                }
                if (zs != lastZS) { lastZS = zs; lastZH = zh; }
            }

            if (now >= nextReport) {
                nextReport += (uint64_t)ticksPerSec;
                if (midi) { // note on + off, middle C
                    static const uint8_t note[] = {0x90, 60, 100, 0x80, 60, 0};
                    US122EngineMidiSend(note, sizeof note);
                }
                US122Stats stt;
                US122EngineGetStats(&stt, true);
                printf("pos %10.0f  rate %10.3f Hz  in %5.1f/%5.1f dBFS  gaps %llu resnap %llu resync %llu txerr %llu  late %llu  backwards %llu  zts-jitter %.1f us  maxgap %.1f ms  outlead %.1f ms  inlag %.1f ms  sublead %.2f ms  cmplat %.2f ms  probe %.2f ms  inlate %llu\n",
                       s, stt.measuredRate, 20 * log10(fmax(peakL, 1e-9)), 20 * log10(fmax(peakR, 1e-9)),
                       stt.captureGaps, stt.playbackResnaps, stt.resyncs, stt.transactionErrors, late, backwards,
                       worstZErrUs, stt.maxCompletionGapMs, stt.maxOutLeadMs, stt.maxInLagMs, stt.minSubmitLeadMs, stt.maxCompletionLatencyMs,
                       atomic_exchange(&gProbeMaxLate, 0) / ticksPerSec * 1e3, stt.inputLateFrames);
                fflush(stdout);
                peakL = peakR = 0;
                worstZErrUs = 0;
            }
            usleep((useconds_t)(buf * 1e6 / rate / 2));
        }

        US122EngineStop();
        if (wav) {
            wavHeader(wav, rate, (uint32_t)wavBytes);
            fclose(wav);
            printf("wrote %s\n", wavPath);
        }
    }
    return 0;
}
