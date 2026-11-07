// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Oliver Frommel

// US122Engine — see US122Engine.h.
//
// Clocking
// --------
// The device's isochronous endpoints are asynchronous with no feedback
// endpoint, so the device's sample clock is only observable through the
// number of frames it puts into each capture packet. Every capture transaction
// belongs to a known USB microframe m (we schedule them explicitly), so we fit
//
//     device sample S(m) = C0 + R * (m - m0)          (R = frames/microframe)
//     host time      h(m) = hRef + (m - mRef) * hPerMf
//
// R comes from the cumulative capture frame count, which is exact up to ±1
// frame, so it is far less noisy than completion timestamps. The microframe ->
// host time mapping is calibrated from IOUSBHost's reference microframe.
//
// Playback packet sizes follow the same model (n = S(m+1) - S(m)), so the host
// sends samples at exactly the rate the device consumes them.
//
// Scheduling
// ----------
// USB completions reach userspace through GCD, which can deliver them 10-20 ms
// late even when the system is otherwise idle. So completions do not drive
// submission: a dedicated real-time "pump" thread wakes every 1 ms and keeps
// each pipe queued a fixed number of frames ahead of the bus, drawing from a
// pool of request buffers that completions merely hand back. Output latency is
// therefore set by the pump lead (`depth`), not by completion delivery.

#import "US122Engine.h"
#import <Foundation/Foundation.h>
#import <IOKit/IOKitLib.h>
#import <IOUSBHost/IOUSBHost.h>
#import <CoreAudio/AudioHardwareBase.h>
#include <mach/mach_time.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <mach/mach.h>
#include <mach/thread_policy.h>
#include <os/lock.h>

#define US122_VID 0x0644
#define US122_PID 0x8021
#define US122_INTF 1
#define US122_ALT 1
#define EP_IN 0x81
#define EP_OUT 0x02
#define EP_MIDI_IN 0x83   // bulk, 9-byte packets: up to 8 MIDI bytes padded with 0xFD
#define EP_MIDI_OUT 0x04  // bulk, same framing plus a trailing 0xE0
#define MIDI_PACKET 9
#define MIDI_PAYLOAD 8
#define MIDI_PAD 0xFD
#define MIDI_IN_REQS 2
#define MIDI_FIFO 4096
#define BYTES_PER_FRAME 6
#define UF_PER_REQ 8 // one 1 ms frame per request
#define POOL 64           // request buffers per direction (covers long completion stalls)
#define RING_FRAMES 65536 // power of two
#define RING_MASK (RING_FRAMES - 1)
#define DEFAULT_DEPTH 16  // ms: output is packed this many USB frames ahead of the bus.
                          // The kernel enqueue path can block ~10-13 ms; 12 still drops out
                          // every minute or two, 16 measured clean (tools/enginetest --depth).
#define MAX_DEPTH 32
#define MIN_LEAD 3        // frames: requests closer to "now" than this are likely rejected as late
#define IN_AHEAD 32       // frames: capture requests are posted this far ahead (no latency cost)
#define DEFAULT_IN_SAFETY_MS 8 // p99 input lag (completion delivery) measured ~10 ms

typedef struct {
    NSMutableData *data;
    uint64_t frame; // USB frame number this request is scheduled for
    IOUSBHostIsochronousTransaction tx[UF_PER_REQ];
    _Atomic int busy; // owned by the USB stack until its completion has run
} IsoReq;

typedef struct {
    double C0, m0, R;
    double mRef, hRef, hPerMf;
} ClockModel;

static struct {
    US122PresenceCallback presenceCb;
    void *presenceCtx;
    US122LogCallback logCb;

    dispatch_queue_t notifyQ, ioQ;
    IONotificationPortRef notifyPort;
    io_iterator_t addIter, removeIter;
    pthread_mutex_t ctl; // serialises Start/Stop

    _Atomic int present;
    _Atomic int running;
    _Atomic int live;       // capture data flowing in the current USB stream
    _Atomic int clockValid; // clock model usable (stays valid across automatic restarts)
    _Atomic int wantRunning;// host wants IO (set by Start, cleared by Stop)
    _Atomic int keepAlive;  // keep streaming without host IO (MIDI only works while streaming)
    int depth, inSafetyMs;

    // watchdog / recovery
    dispatch_queue_t ctlQ;
    dispatch_source_t watchdog;
    _Atomic uint64_t lastActivity; // mach time of last capture completion
    uint64_t lastRecoverAttempt;
    int continueTimeline;   // next goLive continues the previous sample timeline
    ClockModel prevClock;
    int64_t prevC;
    int64_t Cstart;         // E.C at the anchor (C0) of the current model

    // stream objects (owned by Start/Stop)
    IOUSBHostInterface *intf;
    IOUSBHostPipe *outPipe, *inPipe;
    uint32_t outMax, inMax, rate;
    IsoReq outReq[POOL], inReq[POOL];

    // pump thread state
    pthread_t pump[2]; // [0] playback, [1] capture: separate so one blocked enqueue can't starve the other
    int pumpStarted;
    _Atomic int pumpRun;
    uint64_t nextOutFrame, nextInFrame;
    int outIdx, inIdx;
    uint32_t accum;
    int64_t O;          // next device sample to send on the playback side
    int outInit;
    uint64_t lastPoolWarn;

    _Atomic int stopping;
    _Atomic int inflight;

    // io-queue state
    int64_t C;          // device samples accounted for on the capture side
    _Atomic int64_t capHead; // published E.C: input ring is valid below this
    uint64_t expectedMf;
    uint64_t lastCalMf;
    double calMfA, calHA;
    ClockModel cw;      // writer's copy of the clock
    uint64_t lastCompletion, maxGapTicks;
    double maxOutLead, maxInLag;        // frames, since last GetStats reset
    double logOutLead, logInLag;        // frames, since last stats log
    double minLead, maxLat, logMinLead, logMaxLat; // microframes
    uint64_t lastStatsMf;
    US122Stats lastLogged;
    double maxGapSinceLog;
    US122Stats st;

    // MIDI (bulk pipes on the audio interface; completions on the io queue)
    IOUSBHostPipe *midiInPipe, *midiOutPipe;
    NSMutableData *midiInBuf[MIDI_IN_REQS], *midiOutBuf;
    US122MidiInCallback midiCb;
    void *midiCtx;
    os_unfair_lock midiLock; // guards the out FIFO
    uint8_t midiFifo[MIDI_FIFO];
    uint32_t midiHead, midiTail; // FIFO indices (head = write)
    int midiOutBusy;             // io queue only
    uint64_t midiDropped;

    // published clock (seqlock)
    _Atomic uint32_t seq;
    ClockModel clk;

    float *inRing, *outRing;
    double ticksPerNs;
} E;

// ---------------------------------------------------------------- logging

static void elog(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void elog(const char *fmt, ...) {
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    if (E.logCb) E.logCb(buf);
    else fprintf(stderr, "[us122] %s\n", buf);
}

// ---------------------------------------------------------------- clock

static inline double sampleAtMf(const ClockModel *c, double mf) { return c->C0 + c->R * (mf - c->m0); }
static inline double mfAtSample(const ClockModel *c, double s) { return c->m0 + (s - c->C0) / c->R; }
static inline double hostAtMf(const ClockModel *c, double mf) { return c->hRef + (mf - c->mRef) * c->hPerMf; }
static inline double mfAtHost(const ClockModel *c, double h) { return c->mRef + (h - c->hRef) / c->hPerMf; }

static void clockPublish(void) {
    uint32_t s = atomic_load_explicit(&E.seq, memory_order_relaxed);
    atomic_store_explicit(&E.seq, s + 1, memory_order_relaxed);
    atomic_thread_fence(memory_order_release);
    E.clk = E.cw;
    atomic_store_explicit(&E.seq, s + 2, memory_order_release);
}

static ClockModel clockRead(void) {
    ClockModel m;
    for (;;) {
        uint32_t s1 = atomic_load_explicit(&E.seq, memory_order_acquire);
        if (s1 & 1) continue;
        m = E.clk;
        atomic_thread_fence(memory_order_acquire);
        if (atomic_load_explicit(&E.seq, memory_order_relaxed) == s1) return m;
    }
}

static void watchdogTick(void);
static void teardownLocked(void);
static void primeDevice(void);
static void midiStart(void);
static void midiKick(void);
static bool streamWanted(void) { return atomic_load(&E.wantRunning) || atomic_load(&E.keepAlive); }

// ---------------------------------------------------------------- matching / hot-plug

static CFMutableDictionaryRef matchDict(void) {
    return [IOUSBHostInterface createMatchingDictionaryWithVendorID:@US122_VID productID:@US122_PID
                                                          bcdDevice:nil interfaceNumber:@US122_INTF
                                                 configurationValue:@1 interfaceClass:nil
                                                  interfaceSubclass:nil interfaceProtocol:nil
                                                              speed:nil productIDArray:nil];
}

static int drain(io_iterator_t it) {
    int n = 0;
    io_service_t s;
    while ((s = IOIteratorNext(it))) { IOObjectRelease(s); n++; }
    return n;
}

static void onAdded(void *ref, io_iterator_t it) {
    if (drain(it) && !atomic_exchange(&E.present, 1)) {
        elog("device connected");
        if (E.presenceCb) E.presenceCb(E.presenceCtx, true);
        dispatch_async(E.ctlQ, ^{ primeDevice(); });
    }
}

static void onRemoved(void *ref, io_iterator_t it) {
    if (drain(it) && atomic_exchange(&E.present, 0)) {
        elog("device disconnected");
        if (E.presenceCb) E.presenceCb(E.presenceCtx, false);
        dispatch_async(E.ctlQ, ^{
            pthread_mutex_lock(&E.ctl);
            if (E.intf && !atomic_load(&E.present)) {
                teardownLocked();
                atomic_store(&E.clockValid, 0);
                elog("stopped (device gone)");
            }
            pthread_mutex_unlock(&E.ctl);
        });
    }
}

void US122EngineInit(US122PresenceCallback cb, void *ctx, US122LogCallback log) {
    static dispatch_once_t once;
    dispatch_once(&once, ^{
        E.presenceCb = cb;
        E.presenceCtx = ctx;
        E.logCb = log;
        pthread_mutex_init(&E.ctl, NULL);
        E.midiLock = OS_UNFAIR_LOCK_INIT;
        mach_timebase_info_data_t tb;
        mach_timebase_info(&tb);
        E.ticksPerNs = (double)tb.denom / tb.numer;
        const char *d = getenv("US122_DEPTH");
        E.depth = d ? atoi(d) : DEFAULT_DEPTH;
        if (E.depth < MIN_LEAD || E.depth > MAX_DEPTH) E.depth = DEFAULT_DEPTH;
        const char *is = getenv("US122_IN_SAFETY_MS");
        E.inSafetyMs = is ? atoi(is) : DEFAULT_IN_SAFETY_MS;
        if (E.inSafetyMs < 1 || E.inSafetyMs > 50) E.inSafetyMs = DEFAULT_IN_SAFETY_MS;
        E.inRing = calloc(RING_FRAMES * US122_CHANNELS, sizeof(float));
        E.outRing = calloc(RING_FRAMES * US122_CHANNELS, sizeof(float));

        E.ioQ = dispatch_queue_create("us122.io",
            dispatch_queue_attr_make_with_qos_class(DISPATCH_QUEUE_SERIAL, QOS_CLASS_USER_INTERACTIVE, 0));
        E.notifyQ = dispatch_queue_create("us122.notify", DISPATCH_QUEUE_SERIAL);
        E.ctlQ = dispatch_queue_create("us122.ctl", DISPATCH_QUEUE_SERIAL);
        E.watchdog = dispatch_source_create(DISPATCH_SOURCE_TYPE_TIMER, 0, 0, E.ctlQ);
        dispatch_source_set_timer(E.watchdog, dispatch_time(DISPATCH_TIME_NOW, 200 * NSEC_PER_MSEC),
                                  200 * NSEC_PER_MSEC, 20 * NSEC_PER_MSEC);
        dispatch_source_set_event_handler(E.watchdog, ^{ watchdogTick(); });
        dispatch_resume(E.watchdog);
        E.notifyPort = IONotificationPortCreate(kIOMainPortDefault);
        IONotificationPortSetDispatchQueue(E.notifyPort, E.notifyQ);
        dispatch_sync(E.notifyQ, ^{
            kern_return_t kr = IOServiceAddMatchingNotification(E.notifyPort, kIOFirstMatchNotification,
                                                                matchDict(), onAdded, NULL, &E.addIter);
            if (kr == KERN_SUCCESS) onAdded(NULL, E.addIter);
            else elog("add notification failed: 0x%x", kr);
            kr = IOServiceAddMatchingNotification(E.notifyPort, kIOTerminatedNotification,
                                                  matchDict(), onRemoved, NULL, &E.removeIter);
            if (kr == KERN_SUCCESS) drain(E.removeIter);
            else elog("remove notification failed: 0x%x", kr);
        });
        elog("engine initialised (output lead %d ms, input safety %d ms)", E.depth, E.inSafetyMs);
    });
}

bool US122EngineIsPresent(void) { return atomic_load(&E.present); }
bool US122EngineIsRunning(void) { return atomic_load(&E.running); }
bool US122EngineIsSupportedRate(uint32_t r) { return r == 44100 || r == 48000 || r == 88200 || r == 96000; }

// Output samples are packed up to depth+1 ms before they play (the pump fills
// frame f once the bus reaches f-depth, possibly at the start of that frame);
// one more ms of margin for the HAL's own wake-up jitter.
uint32_t US122EngineInputSafetyOffset(uint32_t rate) { return rate * (uint32_t)E.inSafetyMs / 1000; }
uint32_t US122EngineOutputSafetyOffset(uint32_t rate) { return rate * (uint32_t)(E.depth + 2) / 1000; }

// ---------------------------------------------------------------- streaming (io queue)

static kern_return_t setRealtime(thread_t th, double periodMs, double computationMs, double constraintMs) {
    double perMs = E.ticksPerNs * 1e6;
    thread_time_constraint_policy_data_t pol = {
        .period = (uint32_t)(periodMs * perMs),
        .computation = (uint32_t)(computationMs * perMs),
        .constraint = (uint32_t)(constraintMs * perMs),
        .preemptible = 1,
    };
    return thread_policy_set(th, THREAD_TIME_CONSTRAINT_POLICY, (thread_policy_t)&pol,
                             THREAD_TIME_CONSTRAINT_POLICY_COUNT);
}

// Completions run on GCD worker threads, whose scheduling policy GCD resets
// whenever a thread returns to the pool, so re-promote on every completion
// (cheap when it is still set). This keeps capture latency low.
static void ensureRealtime(void) {
    thread_t th = pthread_mach_thread_np(pthread_self());
    thread_time_constraint_policy_data_t cur;
    mach_msg_type_number_t cnt = THREAD_TIME_CONSTRAINT_POLICY_COUNT;
    boolean_t isDefault = FALSE;
    if (thread_policy_get(th, THREAD_TIME_CONSTRAINT_POLICY, (thread_policy_t)&cur, &cnt, &isDefault) == KERN_SUCCESS &&
        !isDefault)
        return; // already real-time
    kern_return_t kr = setRealtime(th, 1.0, 0.25, 1.0);
    if (kr != KERN_SUCCESS) elog("real-time thread policy failed: %d", kr);
}

static inline void put24(uint8_t *d, int32_t s) { d[0] = s; d[1] = s >> 8; d[2] = s >> 16; }
static inline int32_t get24(const uint8_t *s) {
    return (int32_t)((uint32_t)s[0] << 8 | (uint32_t)s[1] << 16 | (uint32_t)s[2] << 24) >> 8;
}
static inline int32_t f2i(float x) {
    if (x >= 1.0f) return 8388607;
    if (x <= -1.0f) return -8388608;
    return (int32_t)lrintf(x * 8388607.0f);
}

static inline double deviceNowWith(const ClockModel *c) {
    return sampleAtMf(c, mfAtHost(c, (double)mach_absolute_time()));
}
static inline double deviceNow(void) { return deviceNowWith(&E.cw); }

// Queue-health probes. noteSubmit runs on the pump thread, noteCompletion on
// the io queue; the values are diagnostics only, so unsynchronised is fine.
static void noteSubmit(const ClockModel *c, uint64_t frame) {
    double lead = (double)(frame * UF_PER_REQ) - mfAtHost(c, (double)mach_absolute_time());
    if (lead < E.minLead) E.minLead = lead;
    if (lead < E.logMinLead) E.logMinLead = lead;
}
static void noteCompletion(uint64_t frame) {
    if (!atomic_load_explicit(&E.live, memory_order_relaxed)) return;
    double lat = mfAtHost(&E.cw, (double)mach_absolute_time()) - (double)((frame + 1) * UF_PER_REQ);
    if (lat > E.maxLat) E.maxLat = lat;
    if (lat > E.logMaxLat) E.logMaxLat = lat;
}

// Pump thread. `c` is a snapshot of the published clock; `live` whether it is valid.
static void fillOut(IsoReq *r, const ClockModel *c, bool live) {
    uint8_t *base = r->data.mutableBytes;
    uint32_t maxFrames = E.outMax / BYTES_PER_FRAME;
    if (live && E.outInit) {
        double lead = (double)E.O - deviceNowWith(c);
        if (lead > E.maxOutLead) E.maxOutLead = lead;
        if (lead > E.logOutLead) E.logOutLead = lead;
    }
    for (int i = 0; i < UF_PER_REQ; i++) {
        uint64_t m = r->frame * UF_PER_REQ + i;
        uint8_t *p = base + i * E.outMax;
        uint32_t n;
        if (live) {
            if (!E.outInit) { E.O = llround(sampleAtMf(c, m)); E.outInit = 1; }
            int64_t target = llround(sampleAtMf(c, m + 1));
            int64_t want = target - E.O;
            if (want < 0 || want > (int64_t)maxFrames) {
                // model moved (skipped frames / restart / large correction): snap
                E.st.playbackResnaps++;
                elog("playback resnap at mf %llu: wanted %lld frames (O %lld, model %.1f)", m, want, E.O, sampleAtMf(c, m));
                E.O = llround(sampleAtMf(c, m));
                want = target - E.O;
                if (want < 0) want = 0;
                if (want > (int64_t)maxFrames) want = maxFrames;
            }
            n = (uint32_t)want;
            for (uint32_t f = 0; f < n; f++, p += BYTES_PER_FRAME) {
                float *s = &E.outRing[((E.O + f) & RING_MASK) * US122_CHANNELS];
                put24(p, f2i(s[0]));
                put24(p + 3, f2i(s[1]));
                s[0] = s[1] = 0; // consumed: don't replay stale audio if the host stops writing
            }
            E.O += n;
            E.st.playbackFrames += n;
        } else {
            // until capture flows, send silence at the nominal rate (the device
            // will not start capturing without playback data)
            E.accum += E.rate;
            n = E.accum / 8000;
            E.accum %= 8000;
            memset(p, 0, n * BYTES_PER_FRAME);
        }
        r->tx[i] = (IOUSBHostIsochronousTransaction){
            .status = kIOReturnInvalid, .requestCount = n * BYTES_PER_FRAME, .offset = (uint32_t)(i * E.outMax)};
    }
}

static void goLive(uint64_t m) {
    IOUSBHostTime t = 0;
    uint64_t refMf = [E.intf referenceMicroframeWithTime:&t error:nil];
    double nominalHPerMf = 125000.0 * E.ticksPerNs;
    if (!refMf || !t) { // fall back to "now"
        IOUSBHostTime t2 = 0;
        uint64_t f = [E.intf frameNumberWithTime:&t2];
        refMf = f * UF_PER_REQ;
        t = t2;
    }
    E.cw = (ClockModel){.C0 = 0, .m0 = (double)m, .R = E.rate / 8000.0,
                        .mRef = (double)refMf, .hRef = (double)t, .hPerMf = nominalHPerMf};
    int64_t c0 = 0;
    if (E.continueTimeline) {
        // Restarted stream: place it on the previous timeline so host sample
        // times stay continuous. Keep the previous rate estimates.
        E.continueTimeline = 0;
        double hostLive = hostAtMf(&E.cw, (double)m);
        c0 = llround(sampleAtMf(&E.prevClock, mfAtHost(&E.prevClock, hostLive)));
        if (c0 < E.prevC) c0 = E.prevC;
        E.cw.R = E.prevClock.R;
        E.cw.hPerMf = E.prevClock.hPerMf;
        E.cw.C0 = (double)c0;
        elog("resuming timeline at sample %lld (%lld frames after last capture)", c0, c0 - E.prevC);
    }
    E.calMfA = refMf;
    E.calHA = t;
    E.lastCalMf = m;
    E.C = c0;
    atomic_store_explicit(&E.capHead, c0, memory_order_release);
    E.Cstart = c0;
    E.expectedMf = m;
    clockPublish();
    atomic_store(&E.clockValid, 1);
    uint64_t now = mach_absolute_time();
    elog("capture live at microframe %llu (ref microframe %llu, ref age %.2f ms)", m, refMf,
         ((double)now - (double)t) / E.ticksPerNs / 1e6);
    atomic_store(&E.live, 1);
}

static void calibrate(uint64_t mLast) {
    // device rate: cumulative frames over microframes since the anchor
    double span = (double)(mLast + 1) - E.cw.m0;
    if (span >= 8000) {
        double R = (double)(E.C - E.Cstart) / span, nominal = E.rate / 8000.0;
        if (fabs(R / nominal - 1) < 1e-3) E.cw.R = R; // ignore physically impossible estimates
        else elog("ignoring rate estimate %.3f Hz", R * 8000);
    }

    // microframe -> host time slope, from two reference points >= 1 s apart
    if (mLast - E.lastCalMf >= 8000) {
        E.lastCalMf = mLast;
        IOUSBHostTime t = 0;
        uint64_t mf = [E.intf referenceMicroframeWithTime:&t error:nil];
        if (mf && t && mf - E.calMfA >= 8000) {
            double slope = ((double)t - E.calHA) / ((double)mf - E.calMfA);
            double nominal = 125000.0 * E.ticksPerNs;
            if (fabs(slope / nominal - 1) < 1e-3) { // sanity: within 1000 ppm
                E.cw.hPerMf = slope;
                E.cw.mRef = E.calMfA;
                E.cw.hRef = E.calHA;
            }
        }
    }
    clockPublish();
}

// Status codes seen when the device is unplugged or the bus goes away: stop
// resubmitting quietly instead of retrying (the presence/watchdog paths take over).
static bool deviceGone(IOReturn r) {
    return r == kIOReturnNoDevice || r == kIOReturnNotResponding || r == kIOReturnOffline ||
           r == kIOReturnNotAttached || r == kIOReturnNotOpen;
}
static bool interfaceGone(NSError *err) {
    return deviceGone((IOReturn)err.code) || [E.intf frameNumberWithTime:NULL] == 0;
}

static void handleIn(IsoReq *r, IOUSBHostIsochronousTransaction *tl) {
    const uint8_t *base = r->data.bytes;
    uint64_t lastM = 0;
    int nerr = 0; IOReturn firstErr = 0; uint64_t firstErrM = 0;
    int64_t cBefore = E.C; // worst-case input lag is just before this data lands
    for (int i = 0; i < UF_PER_REQ; i++) {
        uint64_t m = r->frame * UF_PER_REQ + i;
        if (tl[i].status != kIOReturnSuccess && tl[i].status != kIOReturnUnderrun) {
            E.st.transactionErrors++;
            if (!nerr++) { firstErr = tl[i].status; firstErrM = m; }
            continue;
        }
        uint32_t frames = tl[i].completeCount / BYTES_PER_FRAME;
        if (!atomic_load_explicit(&E.live, memory_order_relaxed)) {
            if (!frames) continue;
            goLive(m);
        }
        if (!frames) {
            // The device never sends empty packets while streaming; an empty
            // completion means the request missed its microframe. Treat it as
            // lost so the gap logic fills it from the clock model.
            E.st.emptyPackets++;
            continue;
        }
        if (m < E.expectedMf) continue; // duplicate / out of order: ignore
        if (m > E.expectedMf) {
            // missed microframes: advance by the model and fill silence
            int64_t newC = llround(sampleAtMf(&E.cw, m));
            for (int64_t s = E.C; s < newC; s++) {
                float *d = &E.inRing[(s & RING_MASK) * US122_CHANNELS];
                d[0] = d[1] = 0;
            }
            if (newC > E.C) E.C = newC;
            E.st.captureGaps++;
            elog("capture gap: mf %llu expected %llu (%llu microframes lost)", m, E.expectedMf, m - E.expectedMf);
        }
        E.expectedMf = m + 1;
        const uint8_t *p = base + tl[i].offset;
        for (uint32_t f = 0; f < frames; f++, p += BYTES_PER_FRAME) {
            float *d = &E.inRing[((E.C + f) & RING_MASK) * US122_CHANNELS];
            d[0] = get24(p) * (1.0f / 8388608.0f);
            d[1] = get24(p + 3) * (1.0f / 8388608.0f);
        }
        E.C += frames;
        E.st.captureFrames += frames;
        lastM = m;
    }
    atomic_store_explicit(&E.capHead, E.C, memory_order_release);
    if (nerr && !deviceGone(firstErr))
        elog("capture: %d transaction error(s), first 0x%08x at mf %llu", nerr, firstErr, firstErrM);
    if (lastM && atomic_load_explicit(&E.live, memory_order_relaxed)) {
        calibrate(lastM);
        double lag = deviceNow() - (double)cBefore;
        if (lag > E.maxInLag) E.maxInLag = lag;
        if (lag > E.logInLag) E.logInLag = lag;
        double gapMs = E.maxGapTicks / E.ticksPerNs / 1e6;
        if (gapMs > E.maxGapSinceLog) E.maxGapSinceLog = gapMs;
        if (lastM - E.lastStatsMf >= 8000 * 5) { // every 5 s
            E.lastStatsMf = lastM;
            US122Stats *a = &E.st, *b = &E.lastLogged;
            bool changed = a->captureGaps != b->captureGaps || a->playbackResnaps != b->playbackResnaps ||
                           a->resyncs != b->resyncs || a->transactionErrors != b->transactionErrors ||
                           a->emptyPackets != b->emptyPackets;
            if (changed || E.maxGapSinceLog > 8.0 || lastM - E.cw.m0 < 8000 * 6)
                elog("stats: rate %.3f Hz, gaps %llu, empty %llu, resnaps %llu, resyncs %llu, txerr %llu, max completion gap %.1f ms, out lead %.1f ms, in lag %.1f ms, min submit lead %.2f ms, max completion latency %.2f ms",
                     E.cw.R / E.cw.hPerMf * E.ticksPerNs * 1e9, a->captureGaps, a->emptyPackets, a->playbackResnaps, a->resyncs,
                     a->transactionErrors, E.maxGapSinceLog, E.logOutLead > -1e8 ? E.logOutLead * 1000.0 / E.rate : 0, E.logInLag > -1e8 ? E.logInLag * 1000.0 / E.rate : 0,
                     E.logMinLead < 1e8 ? E.logMinLead / 8.0 : 0, E.logMaxLat > -1e8 ? E.logMaxLat / 8.0 : 0);
            E.logOutLead = E.logInLag = E.logMaxLat = -1e9;
            E.logMinLead = 1e9;
            E.lastLogged = *a;
            E.maxGapSinceLog = 0;
            E.maxGapTicks = 0;
        }
    }
}

// Completion handlers (io queue). They only account and hand the buffer back.
static void outDone(IsoReq *r, IOReturn status, IOUSBHostIsochronousTransaction *tl) {
    ensureRealtime();
    if (status != kIOReturnAborted && !deviceGone(status) && !atomic_load(&E.stopping)) {
        noteCompletion(r->frame);
        int nerr = 0; IOReturn firstErr = 0;
        for (int i = 0; i < UF_PER_REQ; i++)
            if (tl[i].status != kIOReturnSuccess) {
                E.st.transactionErrors++;
                if (!nerr++) firstErr = tl[i].status;
            }
        if (nerr && !deviceGone(firstErr))
            elog("playback: %d transaction error(s), first 0x%08x at frame %llu", nerr, firstErr, r->frame);
    }
    atomic_store_explicit(&r->busy, 0, memory_order_release);
    atomic_fetch_sub(&E.inflight, 1);
}

static void inDone(IsoReq *r, IOReturn status, IOUSBHostIsochronousTransaction *tl) {
    ensureRealtime();
    if (status != kIOReturnAborted && !deviceGone(status) && !atomic_load(&E.stopping)) {
        noteCompletion(r->frame);
        uint64_t now = mach_absolute_time();
        if (E.lastCompletion && now - E.lastCompletion > E.maxGapTicks) E.maxGapTicks = now - E.lastCompletion;
        E.lastCompletion = now;
        atomic_store_explicit(&E.lastActivity, now, memory_order_relaxed);
        handleIn(r, tl);
    }
    atomic_store_explicit(&r->busy, 0, memory_order_release);
    atomic_fetch_sub(&E.inflight, 1);
}

// Returns false if the interface is gone (pump should stop).
static bool pumpOut(uint64_t nowF, const ClockModel *c, bool live) {
    while (E.nextOutFrame <= nowF + (uint64_t)E.depth) {
        if (E.nextOutFrame < nowF + MIN_LEAD) {
            // fell behind (pump stalled or pool ran dry): skip ahead
            E.st.resyncs++;
            elog("playback skipped frames %llu..%llu", E.nextOutFrame, nowF + E.depth - 1);
            E.nextOutFrame = nowF + E.depth;
        }
        IsoReq *r = &E.outReq[E.outIdx];
        if (atomic_load_explicit(&r->busy, memory_order_acquire)) return true; // pool exhausted: wait
        r->frame = E.nextOutFrame;
        fillOut(r, c, live);
        atomic_store(&r->busy, 1);
        atomic_fetch_add(&E.inflight, 1);
        NSError *err = nil;
        uint64_t tq0 = mach_absolute_time();
        BOOL ok = [E.outPipe enqueueIORequestWithData:r->data transactionList:r->tx
                                 transactionListCount:UF_PER_REQ firstFrameNumber:r->frame
                                              options:0 error:&err
                                    completionHandler:^(IOReturn st, IOUSBHostIsochronousTransaction *tl) { outDone(r, st, tl); }];
        double tqMs = (double)(mach_absolute_time() - tq0) / E.ticksPerNs / 1e6;
        if (tqMs > 8.0) elog("outPipe enqueue took %.2f ms (frame %llu, now was %llu, now is %llu)", tqMs, r->frame, nowF, [E.intf frameNumberWithTime:NULL]);
        if (!ok) {
            atomic_store(&r->busy, 0);
            atomic_fetch_sub(&E.inflight, 1);
            if (interfaceGone(err)) return false;
            E.st.resyncs++;
            elog("playback enqueue rejected for frame %llu (now %llu): 0x%08lx", r->frame, nowF, (long)err.code);
            E.nextOutFrame = nowF + E.depth; // back to full lead, or it keeps missing
            return true;
        }
        if (live) noteSubmit(c, r->frame);
        E.nextOutFrame++;
        E.outIdx = (E.outIdx + 1) % POOL;
    }
    return true;
}

static bool pumpIn(uint64_t nowF) {
    while (E.nextInFrame <= nowF + IN_AHEAD) {
        if (E.nextInFrame < nowF + MIN_LEAD) {
            E.st.resyncs++;
            elog("capture skipped frames %llu..%llu", E.nextInFrame, nowF + IN_AHEAD - 1);
            E.nextInFrame = nowF + IN_AHEAD;
        }
        IsoReq *r = &E.inReq[E.inIdx];
        if (atomic_load_explicit(&r->busy, memory_order_acquire)) {
            uint64_t t = mach_absolute_time();
            if ((double)(t - E.lastPoolWarn) / E.ticksPerNs > 1e9) {
                E.lastPoolWarn = t;
                elog("capture buffer pool exhausted (completions delayed)");
            }
            return true;
        }
        r->frame = E.nextInFrame;
        for (int i = 0; i < UF_PER_REQ; i++)
            r->tx[i] = (IOUSBHostIsochronousTransaction){
                .status = kIOReturnInvalid, .requestCount = E.inMax, .offset = (uint32_t)(i * E.inMax)};
        atomic_store(&r->busy, 1);
        atomic_fetch_add(&E.inflight, 1);
        NSError *err = nil;
        uint64_t tq0 = mach_absolute_time();
        BOOL ok = [E.inPipe enqueueIORequestWithData:r->data transactionList:r->tx
                                transactionListCount:UF_PER_REQ firstFrameNumber:r->frame
                                             options:0 error:&err
                                   completionHandler:^(IOReturn st, IOUSBHostIsochronousTransaction *tl) { inDone(r, st, tl); }];
        double tqMs = (double)(mach_absolute_time() - tq0) / E.ticksPerNs / 1e6;
        if (tqMs > 8.0) elog("inPipe enqueue took %.2f ms (frame %llu, now was %llu, now is %llu)", tqMs, r->frame, nowF, [E.intf frameNumberWithTime:NULL]);
        if (!ok) {
            atomic_store(&r->busy, 0);
            atomic_fetch_sub(&E.inflight, 1);
            if (interfaceGone(err)) return false;
            E.st.resyncs++;
            elog("capture enqueue rejected for frame %llu (now %llu): 0x%08lx", r->frame, nowF, (long)err.code);
            E.nextInFrame = nowF + IN_AHEAD;
            return true;
        }
        E.nextInFrame++;
        E.inIdx = (E.inIdx + 1) % POOL;
    }
    return true;
}

static void *pumpMain(void *arg) {
    bool capture = arg != NULL;
    pthread_setname_np(capture ? "us122.pump.in" : "us122.pump.out");
    kern_return_t kr = setRealtime(pthread_mach_thread_np(pthread_self()), 1.0, 0.2, 0.5);
    if (kr != KERN_SUCCESS) elog("pump: real-time policy failed: %d", kr);
    uint64_t tick = (uint64_t)(1e6 * E.ticksPerNs), next = mach_absolute_time();
    while (atomic_load_explicit(&E.pumpRun, memory_order_acquire)) {
        uint64_t nowF = [E.intf frameNumberWithTime:NULL];
        if (!nowF) break; // interface gone
        if (capture) {
            if (!pumpIn(nowF)) break;
        } else {
            bool live = atomic_load_explicit(&E.live, memory_order_acquire);
            ClockModel c = clockRead();
            if (!pumpOut(nowF, &c, live)) break;
        }
        next += tick;
        uint64_t now = mach_absolute_time();
        if (next < now) next = now + tick / 2; // overslept: don't try to catch up in a burst
        mach_wait_until(next);
    }
    return NULL;
}

// ---------------------------------------------------------------- MIDI (io queue)

static void midiRead(int i) {
    NSMutableData *buf = E.midiInBuf[i];
    IOUSBHostPipe *pipe = E.midiInPipe;
    if (!pipe || !buf || atomic_load(&E.stopping)) return;
    atomic_fetch_add(&E.inflight, 1);
    NSError *err = nil;
    BOOL ok = [pipe enqueueIORequestWithData:buf completionTimeout:0 error:&err
                           completionHandler:^(IOReturn st, NSUInteger n) {
        atomic_fetch_sub(&E.inflight, 1);
        if (st == kIOReturnAborted || deviceGone(st) || atomic_load(&E.stopping) || pipe != E.midiInPipe) return;
        if (st == kIOReturnSuccess) {
            const uint8_t *b = buf.bytes;
            size_t len = 0;
            while (len < n && len < MIDI_PAYLOAD && b[len] != MIDI_PAD) len++;
            if (len && E.midiCb) E.midiCb(E.midiCtx, b, len);
        } else {
            elog("MIDI in: 0x%08x", st);
            [pipe clearStallWithError:nil];
        }
        midiRead(i);
    }];
    if (!ok) {
        atomic_fetch_sub(&E.inflight, 1);
        elog("MIDI in enqueue: %s", err.localizedDescription.UTF8String);
    }
}

static void midiStart(void) {
    for (int i = 0; i < MIDI_IN_REQS; i++) midiRead(i);
    midiKick(); // anything queued while the stream was down
}

static void midiKick(void) {
    IOUSBHostPipe *pipe = E.midiOutPipe;
    if (E.midiOutBusy || !pipe || !E.midiOutBuf || atomic_load(&E.stopping)) return;
    uint8_t pkt[MIDI_PACKET];
    uint32_t n = 0;
    os_unfair_lock_lock(&E.midiLock);
    while (n < MIDI_PAYLOAD && E.midiTail != E.midiHead) {
        pkt[n++] = E.midiFifo[E.midiTail % MIDI_FIFO];
        E.midiTail++;
    }
    os_unfair_lock_unlock(&E.midiLock);
    if (!n) return;
    memset(pkt + n, MIDI_PAD, MIDI_PAYLOAD - n);
    pkt[MIDI_PAYLOAD] = 0xE0;
    NSMutableData *buf = E.midiOutBuf;
    memcpy(buf.mutableBytes, pkt, MIDI_PACKET);
    E.midiOutBusy = 1;
    atomic_fetch_add(&E.inflight, 1);
    NSError *err = nil;
    BOOL ok = [pipe enqueueIORequestWithData:buf completionTimeout:1.0 error:&err
                           completionHandler:^(IOReturn st, NSUInteger sent) {
        atomic_fetch_sub(&E.inflight, 1);
        if (pipe != E.midiOutPipe) return;
        E.midiOutBusy = 0;
        if (st == kIOReturnAborted || deviceGone(st) || atomic_load(&E.stopping)) return;
        if (st != kIOReturnSuccess) {
            elog("MIDI out: 0x%08x", st);
            [pipe clearStallWithError:nil];
        }
        midiKick();
    }];
    if (!ok) {
        atomic_fetch_sub(&E.inflight, 1);
        E.midiOutBusy = 0;
        elog("MIDI out enqueue: %s", err.localizedDescription.UTF8String);
    }
}

void US122EngineSetMidiInCallback(US122MidiInCallback cb, void *ctx) {
    dispatch_sync(E.ioQ, ^{ E.midiCb = cb; E.midiCtx = ctx; });
}

void US122EngineMidiSend(const uint8_t *bytes, size_t len) {
    os_unfair_lock_lock(&E.midiLock);
    for (size_t i = 0; i < len; i++) {
        if (E.midiHead - E.midiTail >= MIDI_FIFO) { E.midiDropped += len - i; break; }
        E.midiFifo[E.midiHead % MIDI_FIFO] = bytes[i];
        E.midiHead++;
    }
    os_unfair_lock_unlock(&E.midiLock);
    dispatch_async(E.ioQ, ^{ midiKick(); });
}

// ---------------------------------------------------------------- start / stop

static void stopPump(void) {
    if (!E.pumpStarted) return;
    atomic_store(&E.pumpRun, 0);
    for (int i = 0; i < E.pumpStarted; i++) pthread_join(E.pump[i], NULL);
    E.pumpStarted = 0;
}

static void teardownLocked(void) {
    if (!E.intf) return;
    atomic_store(&E.running, 0);
    atomic_store(&E.stopping, 1);
    stopPump();
    [E.outPipe abortWithError:nil];
    [E.inPipe abortWithError:nil];
    [E.midiInPipe abortWithError:nil];
    [E.midiOutPipe abortWithError:nil];
    for (int i = 0; i < 200 && atomic_load(&E.inflight) > 0; i++) usleep(5000);
    if (atomic_load(&E.inflight) > 0) elog("teardown: %d requests still in flight", atomic_load(&E.inflight));
    [E.intf selectAlternateSetting:0 error:nil];
    [E.intf destroy];
    dispatch_sync(E.ioQ, ^{
        for (int i = 0; i < POOL; i++) {
            E.outReq[i].data = nil; E.inReq[i].data = nil;
            atomic_store(&E.outReq[i].busy, 0); atomic_store(&E.inReq[i].busy, 0);
        }
        E.outPipe = nil;
        E.inPipe = nil;
        E.midiInPipe = E.midiOutPipe = nil;
        for (int i = 0; i < MIDI_IN_REQS; i++) E.midiInBuf[i] = nil;
        E.midiOutBuf = nil;
        E.midiOutBusy = 0;
        E.intf = nil;
    });
    atomic_store(&E.inflight, 0);
    atomic_store(&E.live, 0);
}

static OSStatus startLocked(uint32_t rate, bool continueTimeline) {
    OSStatus result = kAudioHardwareNoError;
    NSError *err = nil;

    if (E.intf) {
        if (E.rate == rate && atomic_load(&E.running)) goto out;
        teardownLocked();
    }

    // The interface can be briefly busy (e.g. a previous owner still closing),
    // so retry for up to ~1 s.
    for (int attempt = 0; attempt < 10 && !E.intf; attempt++) {
        if (attempt) usleep(100 * 1000);
        io_service_t svc = IOServiceGetMatchingService(kIOMainPortDefault, matchDict());
        if (!svc) { elog("start: device not found"); result = kAudioHardwareBadDeviceError; goto out; }
        E.intf = [[IOUSBHostInterface alloc] initWithIOService:svc options:0 queue:E.ioQ error:&err
                                               interestHandler:^(IOUSBHostObject *o, uint32_t type, void *arg) {
            if (type == kIOMessageServiceIsTerminated) {
                elog("interface terminated while open");
                atomic_store(&E.running, 0);
                atomic_store(&E.stopping, 1);
            }
        }];
        IOObjectRelease(svc);
        if (!E.intf) elog("open interface (attempt %d): %s", attempt + 1, err.localizedDescription.UTF8String);
    }
    if (!E.intf) { elog("open interface: %s", err.localizedDescription.UTF8String); result = kAudioHardwareUnspecifiedError; goto out; }

    if (![E.intf selectAlternateSetting:US122_ALT error:&err]) { elog("alt setting: %s", err.localizedDescription.UTF8String); goto fail; }
    E.outPipe = [E.intf copyPipeWithAddress:EP_OUT error:&err];
    E.inPipe = [E.intf copyPipeWithAddress:EP_IN error:&err];
    if (!E.outPipe || !E.inPipe) { elog("open pipes: %s", err.localizedDescription.UTF8String); goto fail; }
    E.outMax = E.outPipe.descriptors->descriptor.wMaxPacketSize & 0x7ff;
    E.inMax = E.inPipe.descriptors->descriptor.wMaxPacketSize & 0x7ff;
    if (E.outMax < (rate / 8000 + 1) * BYTES_PER_FRAME) { elog("max packet %u too small", E.outMax); goto fail; }
    {
        NSError *merr = nil;
        E.midiInPipe = [E.intf copyPipeWithAddress:EP_MIDI_IN error:&merr];
        E.midiOutPipe = [E.intf copyPipeWithAddress:EP_MIDI_OUT error:&merr];
        for (int i = 0; i < MIDI_IN_REQS && E.midiInPipe; i++)
            E.midiInBuf[i] = [E.intf ioDataWithCapacity:MIDI_PACKET error:&merr];
        E.midiOutBuf = E.midiOutPipe ? [E.intf ioDataWithCapacity:MIDI_PACKET error:&merr] : nil;
        if (!E.midiInPipe || !E.midiOutPipe || !E.midiOutBuf)
            elog("MIDI pipes unavailable: %s", merr.localizedDescription.UTF8String);
    }

    {
        NSMutableData *d = [NSMutableData dataWithLength:3];
        uint8_t *p = d.mutableBytes;
        p[0] = rate & 0xff; p[1] = (rate >> 8) & 0xff; p[2] = (rate >> 16) & 0xff;
        IOUSBDeviceRequest req = {.bmRequestType = 0x22, .bRequest = 0x01, .wValue = 0x0100,
                                  .wIndex = EP_IN, .wLength = 3};
        if (![E.intf sendDeviceRequest:req data:d bytesTransferred:NULL completionTimeout:1.0 error:&err]) {
            elog("set rate %u: %s", rate, err.localizedDescription.UTF8String);
            goto fail;
        }
    }

    for (int i = 0; i < POOL; i++) {
        E.outReq[i].data = [E.intf ioDataWithCapacity:UF_PER_REQ * E.outMax error:&err];
        E.inReq[i].data = [E.intf ioDataWithCapacity:UF_PER_REQ * E.inMax error:&err];
        if (!E.outReq[i].data || !E.inReq[i].data) { elog("alloc: %s", err.localizedDescription.UTF8String); goto fail; }
    }

    if (!continueTimeline) {
        memset(E.inRing, 0, RING_FRAMES * US122_CHANNELS * sizeof(float));
        memset(E.outRing, 0, RING_FRAMES * US122_CHANNELS * sizeof(float));
    }
    E.rate = rate;
    atomic_store(&E.lastActivity, mach_absolute_time());
    dispatch_sync(E.ioQ, ^{
        E.continueTimeline = continueTimeline;
        if (!continueTimeline) { E.C = 0; atomic_store(&E.capHead, 0); }
        E.st = (US122Stats){0};
        E.lastCompletion = E.maxGapTicks = 0;
        E.maxOutLead = E.maxInLag = E.logOutLead = E.logInLag = E.maxLat = E.logMaxLat = -1e9;
        E.minLead = E.logMinLead = 1e9;
        E.lastStatsMf = 0;
        E.lastLogged = (US122Stats){0};
        E.maxGapSinceLog = 0;
        atomic_store(&E.live, 0);
    });
    atomic_store(&E.stopping, 0);
    atomic_store(&E.inflight, 0);
    E.accum = 0;
    E.outInit = 0;
    E.outIdx = E.inIdx = 0;
    E.nextOutFrame = E.nextInFrame = [E.intf frameNumberWithTime:NULL] + 4;
    atomic_store(&E.pumpRun, 1);
    {
        pthread_attr_t attr;
        pthread_attr_init(&attr);
        int pe = 0;
        for (int i = 0; i < 2 && !pe; i++)
            if (!(pe = pthread_create(&E.pump[i], &attr, pumpMain, i ? (void *)1 : NULL))) E.pumpStarted++;
        pthread_attr_destroy(&attr);
        if (pe) { elog("pump thread: %d", pe); goto fail; }
    }

    for (int i = 0; i < 2000 && !atomic_load(&E.live); i++) usleep(1000);
    if (!atomic_load(&E.live)) { elog("no capture data within 2 s"); goto fail; }
    atomic_store(&E.running, 1);
    dispatch_sync(E.ioQ, ^{ midiStart(); });
    elog("streaming at %u Hz", rate);
    goto out;

fail:
    teardownLocked();
    result = kAudioHardwareUnspecifiedError;
out:
    return result;
}

OSStatus US122EngineStart(uint32_t rate) {
    if (!US122EngineIsSupportedRate(rate)) return kAudioHardwareIllegalOperationError;
    pthread_mutex_lock(&E.ctl);
    if (E.rate != rate) atomic_store(&E.clockValid, 0); // new rate: new timeline
    OSStatus st = startLocked(rate, false);
    atomic_store(&E.wantRunning, st == kAudioHardwareNoError);
    pthread_mutex_unlock(&E.ctl);
    return st;
}

void US122EngineStop(void) {
    pthread_mutex_lock(&E.ctl);
    atomic_store(&E.wantRunning, 0);
    if (atomic_load(&E.keepAlive) && atomic_load(&E.present) && E.intf) {
        // host IO stopped, but MIDI needs the stream: keep it (sending silence)
    } else {
        if (E.intf) {
            teardownLocked();
            elog("stopped");
        }
        atomic_store(&E.clockValid, 0);
    }
    pthread_mutex_unlock(&E.ctl);
}

void US122EngineSetKeepAlive(bool on) {
    atomic_store(&E.keepAlive, on);
    dispatch_async(E.ctlQ, ^{
        pthread_mutex_lock(&E.ctl);
        if (on && atomic_load(&E.present) && !E.intf) {
            if (startLocked(E.rate ? E.rate : 48000, false) == kAudioHardwareNoError) elog("streaming for MIDI");
        } else if (!on && !atomic_load(&E.wantRunning) && E.intf) {
            teardownLocked();
            atomic_store(&E.clockValid, 0);
            elog("stopped");
        }
        pthread_mutex_unlock(&E.ctl);
    });
}

// Watchdog (ctlQ, every 200 ms): if the host wants IO but no capture data has
// arrived for 300 ms (device reset, sleep/wake, re-plug, stuck pipes), tear the
// stream down and restart it on the same timeline. The clock model keeps
// extrapolating meanwhile, so the HAL sees a dropout rather than an error.
static void watchdogTick(void) {
    if (!streamWanted()) return;
    uint64_t now = mach_absolute_time();
    double sinceMs = ((double)now - (double)atomic_load(&E.lastActivity)) / E.ticksPerNs / 1e6;
    bool healthy = E.intf && atomic_load(&E.running) && sinceMs < 300;
    if (healthy || !atomic_load(&E.present)) return;
    if (((double)now - (double)E.lastRecoverAttempt) / E.ticksPerNs < 1e9) return; // at most once a second
    E.lastRecoverAttempt = now;

    pthread_mutex_lock(&E.ctl);
    if (streamWanted() && atomic_load(&E.present)) {
        elog("stream stalled (%.0f ms without capture data); restarting", sinceMs);
        __block int64_t lastC = 0;
        bool haveTimeline = atomic_load(&E.clockValid);
        if (E.intf) dispatch_sync(E.ioQ, ^{ lastC = E.C; });
        teardownLocked();
        // the host keeps reading input during the outage: make it silence, not stale audio
        memset(E.inRing, 0, RING_FRAMES * US122_CHANNELS * sizeof(float));
        if (haveTimeline) {
            E.prevClock = clockRead();
            E.prevC = lastC;
        }
        OSStatus st = startLocked(E.rate, haveTimeline);
        if (st) elog("restart failed (%d); will retry", (int)st);
    }
    pthread_mutex_unlock(&E.ctl);
}

// After power-up the US-122MKII puts out a loud sweeping noise until it has
// received its first audio stream; afterwards it stays quiet even when idle.
// So on every connect, stream a moment of silence unless IO is already wanted.
// After power-up the US-122MKII puts out a loud sweeping noise until it is
// configured. Selecting the streaming alt setting and setting the sample rate
// is enough (no audio stream needed), and takes a few ms, so do it right on
// connect. Streaming instead took ~2 s because the bus is busy after plug-in.
static void primeDevice(void) {
    pthread_mutex_lock(&E.ctl);
    if (!streamWanted() && !E.intf && atomic_load(&E.present)) {
        uint32_t rate = E.rate ? E.rate : 48000;
        uint64_t t0 = mach_absolute_time();
#define SINCE() ((double)(mach_absolute_time() - t0) / E.ticksPerNs / 1e6)
        NSError *err = nil;
        IOUSBHostInterface *intf = nil;
        for (int attempt = 0; attempt < 20 && !intf; attempt++) {
            if (attempt) usleep(50 * 1000);
            io_service_t svc = IOServiceGetMatchingService(kIOMainPortDefault, matchDict());
            if (!svc) break;
            intf = [[IOUSBHostInterface alloc] initWithIOService:svc options:0 queue:E.ioQ error:&err interestHandler:nil];
            IOObjectRelease(svc);
        }
        if (!intf) {
            elog("prime: open failed: %s", err.localizedDescription.UTF8String);
        } else {
            double tOpen = SINCE();
            BOOL alt = [intf selectAlternateSetting:US122_ALT error:&err];
            double tAlt = SINCE();
            NSMutableData *d = [NSMutableData dataWithLength:3];
            uint8_t *p = d.mutableBytes;
            p[0] = rate & 0xff; p[1] = (rate >> 8) & 0xff; p[2] = (rate >> 16) & 0xff;
            IOUSBDeviceRequest req = {.bmRequestType = 0x22, .bRequest = 0x01, .wValue = 0x0100,
                                      .wIndex = EP_IN, .wLength = 3};
            BOOL rateOk = [intf sendDeviceRequest:req data:d bytesTransferred:NULL completionTimeout:1.0 error:&err];
            double tRate = SINCE();
            [intf selectAlternateSetting:0 error:nil];
            [intf destroy];
            elog("primed device (config only): open %.0f ms, alt setting %s at %.0f ms, rate %u %s at %.0f ms",
                 tOpen, alt ? "ok" : "FAILED", tAlt, rate, rateOk ? "ok" : "FAILED", tRate);
        }
#undef SINCE
    }
    pthread_mutex_unlock(&E.ctl);
}

// Test hook: kill the stream without telling the engine, simulating one that
// dies underneath us (sleep/wake, controller reset).
void US122EngineDebugKillStream(void) {
    elog("debug: stopping pump and aborting pipes");
    atomic_store(&E.pumpRun, 0);
    [E.outPipe abortWithError:nil];
    [E.inPipe abortWithError:nil];
}

// ---------------------------------------------------------------- host-facing API

bool US122EngineGetZeroTimeStamp(uint32_t period, double *sampleTime, uint64_t *hostTime) {
    if (!atomic_load_explicit(&E.clockValid, memory_order_acquire)) return false;
    ClockModel c = clockRead();
    double now = (double)mach_absolute_time();
    double s = sampleAtMf(&c, mfAtHost(&c, now));
    double zs = floor(s / period) * period;
    *sampleTime = zs;
    *hostTime = (uint64_t)llround(hostAtMf(&c, mfAtSample(&c, zs)));
    return true;
}

double US122EngineSampleTimeAtHostTime(uint64_t h) {
    ClockModel c = clockRead();
    return sampleAtMf(&c, mfAtHost(&c, (double)h));
}

uint64_t US122EngineHostTimeAtSampleTime(double s) {
    ClockModel c = clockRead();
    return (uint64_t)llround(hostAtMf(&c, mfAtSample(&c, s)));
}

static _Atomic uint64_t gInputLateFrames;

void US122EngineReadInput(int64_t sampleTime, uint32_t frames, float *dst) {
    if (!atomic_load_explicit(&E.clockValid, memory_order_acquire)) {
        memset(dst, 0, frames * US122_CHANNELS * sizeof(float));
        return;
    }
    // Samples the device hasn't delivered yet (late completion) read as
    // silence rather than whatever the ring held a lap ago.
    int64_t head = atomic_load_explicit(&E.capHead, memory_order_acquire);
    uint32_t valid = sampleTime >= head ? 0 : (sampleTime + frames <= head ? frames : (uint32_t)(head - sampleTime));
    for (uint32_t f = 0; f < valid; f++) {
        const float *s = &E.inRing[((sampleTime + f) & RING_MASK) * US122_CHANNELS];
        dst[f * 2] = s[0];
        dst[f * 2 + 1] = s[1];
    }
    if (valid < frames) {
        memset(dst + valid * 2, 0, (frames - valid) * US122_CHANNELS * sizeof(float));
        atomic_fetch_add_explicit(&gInputLateFrames, frames - valid, memory_order_relaxed);
    }
}

void US122EngineWriteOutput(int64_t sampleTime, uint32_t frames, const float *src) {
    for (uint32_t f = 0; f < frames; f++) {
        float *d = &E.outRing[((sampleTime + f) & RING_MASK) * US122_CHANNELS];
        d[0] = src[f * 2];
        d[1] = src[f * 2 + 1];
    }
}

void US122EngineGetStats(US122Stats *out, bool resetMaxGap) {
    if (!E.ioQ) { *out = (US122Stats){0}; return; }
    dispatch_sync(E.ioQ, ^{
        *out = E.st;
        out->measuredRate = E.cw.R / E.cw.hPerMf * E.ticksPerNs * 1e9;
        out->maxCompletionGapMs = E.maxGapTicks / E.ticksPerNs / 1e6;
        out->maxOutLeadMs = E.rate ? E.maxOutLead * 1000.0 / E.rate : 0;
        out->maxInLagMs = E.rate ? E.maxInLag * 1000.0 / E.rate : 0;
        out->minSubmitLeadMs = E.minLead / 8.0;
        out->maxCompletionLatencyMs = E.maxLat / 8.0;
        out->inputLateFrames = atomic_load(&gInputLateFrames);
        if (resetMaxGap) { E.maxGapTicks = 0; E.maxOutLead = E.maxInLag = E.maxLat = -1e9; E.minLead = 1e9; }
    });
}
