// US122Engine — userspace USB audio engine for the TASCAM US-122MKII.
//
// Owns the device (via IOUSBHost), streams isochronous audio in both
// directions and exposes:
//   * float ring buffers indexed by *device sample time* (interleaved stereo)
//   * a clock model mapping device sample time <-> mach host time, derived
//     from capture packet sizes vs. USB microframe numbers
//
// There is a single global engine (the driver supports one device).
// All functions are thread-safe unless noted.

#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <MacTypes.h>

#ifdef __cplusplus
extern "C" {
#endif

#define US122_CHANNELS 2

typedef void (*US122PresenceCallback)(void *ctx, bool present);
typedef void (*US122LogCallback)(const char *msg);

// Start hot-plug monitoring. `cb` is called on an internal queue whenever the
// device appears or disappears (also once at startup if it is already there).
void US122EngineInit(US122PresenceCallback cb, void *ctx, US122LogCallback log);

bool US122EngineIsPresent(void);
bool US122EngineIsRunning(void);
bool US122EngineIsSupportedRate(uint32_t rate);

// Open the device, set `rate` and start streaming. Blocks until the device
// clock is locked (capture data flowing) or fails. Returns 0 on success.
OSStatus US122EngineStart(uint32_t rate);
void US122EngineStop(void);

// Clock. Valid only while running.
// Zero timestamp per AudioServerPlugIn semantics: the most recent sample time
// that is a multiple of `period`, and the host time it occurred at.
bool US122EngineGetZeroTimeStamp(uint32_t period, double *sampleTime, uint64_t *hostTime);
double US122EngineSampleTimeAtHostTime(uint64_t hostTime);
uint64_t US122EngineHostTimeAtSampleTime(double sampleTime);

// Audio. `sampleTime` is in the device clock domain (same as the zero timestamps).
void US122EngineReadInput(int64_t sampleTime, uint32_t frames, float *dst);
void US122EngineWriteOutput(int64_t sampleTime, uint32_t frames, const float *src);

// Recommended safety offsets in frames for the current rate.
uint32_t US122EngineInputSafetyOffset(uint32_t rate);
uint32_t US122EngineOutputSafetyOffset(uint32_t rate);

typedef struct {
    uint64_t captureFrames, playbackFrames;
    uint64_t captureGaps, playbackResnaps, resyncs, transactionErrors, emptyPackets;
    double measuredRate;     // device sample rate in Hz relative to host clock
    double maxCompletionGapMs;
    // Timing margins (worst case since last reset), in ms of device time:
    // outLead = how far ahead of the device position output samples are
    //           packed into USB requests (output safety offset must cover it);
    // inLag   = how far behind the device position the newest captured
    //           sample is when a capture request completes (input safety
    //           offset must cover it).
    double maxOutLeadMs, maxInLagMs;
    // USB queue health: smallest margin between enqueueing a request and its
    // first microframe, and largest delay from a request's last microframe to
    // its completion handler running.
    double minSubmitLeadMs, maxCompletionLatencyMs;
    uint64_t inputLateFrames; // frames ReadInput returned as silence because they hadn't arrived
} US122Stats;
void US122EngineGetStats(US122Stats *out, bool resetMaxGap);

// Test hook: kill the USB stream underneath the engine (watchdog should recover).
void US122EngineDebugKillStream(void);

#ifdef __cplusplus
}
#endif
