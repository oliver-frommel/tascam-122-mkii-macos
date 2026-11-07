// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Oliver Frommel

// us122ctl — userspace test tool for the TASCAM US-122MKII on macOS.
//
// Talks to the device directly through IOUSBHost (no kernel driver).
// Protocol notes (from serifpersia's Linux driver, us122mkii branch):
//   * interface 1, alternate setting 1
//   * sample rate: UAC1-style SET_CUR (class, endpoint recipient) on EP 0x81,
//     3-byte little-endian rate
//   * EP 0x02 iso OUT playback, EP 0x81 iso IN capture
//   * stereo, 24-bit little-endian packed (6 bytes per frame)
//   * high-speed, one packet per 125 µs microframe, no feedback endpoint
//
// Usage:
//   us122ctl info
//   us122ctl run [--rate N] [--seconds S] [--freq HZ] [--level DBFS]
//                [--silence] [--record out.wav] [--depth MS]

#import <Foundation/Foundation.h>
#import <IOKit/IOKitLib.h>
#import <IOUSBHost/IOUSBHost.h>
#include <mach/mach_time.h>
#include <math.h>
#include <signal.h>

#define US122_VID 0x0644
#define US122_PID 0x8021
#define US122_INTF 1
#define US122_ALT 1
#define EP_IN 0x81
#define EP_OUT 0x02
#define BYTES_PER_FRAME 6
#define UFRAMES_PER_REQ 8 // one 1 ms USB frame per request
#define MAX_REQ 64        // upper bound on requests in flight per direction
static int gDepth = 16;   // requests (ms) in flight per direction, --depth
static int gFeedback = 1; // derive playback packet sizes from capture, --no-feedback

// ---------------------------------------------------------------- helpers

static void die(NSString *what, NSError *err) {
    fprintf(stderr, "error: %s%s%s\n", what.UTF8String, err ? ": " : "",
            err ? err.description.UTF8String : "");
    exit(1);
}

static io_service_t matchInterface(void) {
    CFMutableDictionaryRef m = [IOUSBHostInterface
        createMatchingDictionaryWithVendorID:@US122_VID productID:@US122_PID bcdDevice:nil
                             interfaceNumber:@US122_INTF configurationValue:@1
                              interfaceClass:nil interfaceSubclass:nil interfaceProtocol:nil
                                       speed:nil productIDArray:nil];
    return IOServiceGetMatchingService(kIOMainPortDefault, m);
}

static io_service_t matchDevice(void) {
    CFMutableDictionaryRef m = [IOUSBHostDevice
        createMatchingDictionaryWithVendorID:@US122_VID productID:@US122_PID bcdDevice:nil
                                 deviceClass:nil deviceSubclass:nil deviceProtocol:nil
                                       speed:nil productIDArray:nil];
    return IOServiceGetMatchingService(kIOMainPortDefault, m);
}

// Returns the interface service, configuring the device first if macOS left it
// unconfigured (no driver matched the vendor-specific device).
static io_service_t findInterface(void) {
    io_service_t svc = matchInterface();
    if (svc) return svc;

    io_service_t dsvc = matchDevice();
    if (!dsvc) die(@"US-122MKII (0644:8021) not found — is it plugged in?", nil);

    printf("interface %d not published; configuring device...\n", US122_INTF);
    NSError *err = nil;
    IOUSBHostDevice *dev = [[IOUSBHostDevice alloc] initWithIOService:dsvc options:0
                                                                queue:nil error:&err
                                                      interestHandler:nil];
    IOObjectRelease(dsvc);
    if (!dev) die(@"open device", err);
    const IOUSBConfigurationDescriptor *cd = [dev configurationDescriptorWithIndex:0 error:&err];
    if (!cd) die(@"read configuration descriptor", err);
    if (![dev configureWithValue:cd->bConfigurationValue matchInterfaces:YES error:&err])
        die(@"set configuration", err);
    [dev destroy];

    for (int i = 0; i < 50 && !svc; i++) {
        usleep(100 * 1000);
        svc = matchInterface();
    }
    if (!svc) die(@"interface did not appear after configuring", nil);
    return svc;
}

static IOUSBHostInterface *openInterface(dispatch_queue_t q) {
    io_service_t svc = findInterface();
    NSError *err = nil;
    IOUSBHostInterface *intf = [[IOUSBHostInterface alloc]
        initWithIOService:svc options:0 queue:q error:&err
          interestHandler:^(IOUSBHostObject *obj, uint32_t type, void *arg) {
              if (type == kIOMessageServiceIsTerminated) {
                  fprintf(stderr, "\ndevice disconnected\n");
                  exit(2);
              }
          }];
    IOObjectRelease(svc);
    if (!intf) die(@"open interface (is another process using it?)", err);
    return intf;
}

// ---------------------------------------------------------------- info

static const char *epType(uint8_t attr) {
    static const char *t[] = {"control", "isochronous", "bulk", "interrupt"};
    return t[attr & 3];
}
static const char *syncType(uint8_t attr) {
    static const char *t[] = {"none", "async", "adaptive", "sync"};
    return t[(attr >> 2) & 3];
}

static void cmdInfo(void) {
    dispatch_queue_t q = dispatch_queue_create("us122.info", DISPATCH_QUEUE_SERIAL);
    IOUSBHostInterface *intf = openInterface(q);
    NSError *err = nil;

    const IOUSBDeviceDescriptor *dd = intf.deviceDescriptor;
    if (dd) {
        printf("Device: USB %x.%02x  class %02x/%02x/%02x  ep0 max %u  VID %04x PID %04x bcdDevice %04x\n",
               dd->bcdUSB >> 8, dd->bcdUSB & 0xff, dd->bDeviceClass, dd->bDeviceSubClass,
               dd->bDeviceProtocol, dd->bMaxPacketSize0, dd->idVendor, dd->idProduct, dd->bcdDevice);
        NSString *mfr = dd->iManufacturer ? (NSString *)[intf stringWithIndex:dd->iManufacturer error:nil] : nil;
        NSString *prod = dd->iProduct ? (NSString *)[intf stringWithIndex:dd->iProduct error:nil] : nil;
        printf("        %s / %s\n", mfr.UTF8String ?: "?", prod.UTF8String ?: "?");
    }

    const IOUSBConfigurationDescriptor *cd = intf.configurationDescriptor;
    if (!cd) die(@"no configuration descriptor", err);
    printf("Config %u: %u interfaces, %u bytes, maxPower %u mA\n", cd->bConfigurationValue,
           cd->bNumInterfaces, cd->wTotalLength, cd->MaxPower * 2);

    const IOUSBDescriptorHeader *h = NULL;
    while ((h = IOUSBGetNextDescriptor(cd, h))) {
        const uint8_t *b = (const uint8_t *)h;
        switch (h->bDescriptorType) {
        case kIOUSBDescriptorTypeInterface: {
            const IOUSBInterfaceDescriptor *id = (const void *)h;
            printf("  Interface %u alt %u: %u endpoints, class %02x/%02x/%02x\n",
                   id->bInterfaceNumber, id->bAlternateSetting, id->bNumEndpoints,
                   id->bInterfaceClass, id->bInterfaceSubClass, id->bInterfaceProtocol);
            break;
        }
        case kIOUSBDescriptorTypeEndpoint: {
            const IOUSBEndpointDescriptor *ed = (const void *)h;
            uint16_t mps = ed->wMaxPacketSize;
            printf("    EP 0x%02x %s %s, maxPacket %u (x%u), bInterval %u\n",
                   ed->bEndpointAddress, epType(ed->bmAttributes),
                   (ed->bmAttributes & 3) == 1 ? syncType(ed->bmAttributes) : "",
                   mps & 0x7ff, ((mps >> 11) & 3) + 1, ed->bInterval);
            break;
        }
        default: {
            printf("    desc type 0x%02x len %u:", h->bDescriptorType, h->bLength);
            for (int i = 2; i < h->bLength; i++) printf(" %02x", b[i]);
            printf("\n");
        }
        }
    }

    // Read back the current sample rate (UAC1 GET_CUR, endpoint recipient).
    NSMutableData *rate = [NSMutableData dataWithLength:3];
    IOUSBDeviceRequest req = {.bmRequestType = 0xA2, .bRequest = 0x81, .wValue = 0x0100,
                              .wIndex = EP_IN, .wLength = 3};
    NSUInteger got = 0;
    if ([intf sendDeviceRequest:req data:rate bytesTransferred:&got completionTimeout:1.0 error:&err] && got == 3) {
        const uint8_t *r = rate.bytes;
        printf("Current sample rate (GET_CUR on EP 0x81): %u Hz\n", r[0] | r[1] << 8 | r[2] << 16);
    } else {
        printf("GET_CUR sample rate not supported: %s\n", err.localizedDescription.UTF8String ?: "?");
    }
    [intf destroy];
}

// ---------------------------------------------------------------- streaming

typedef struct {
    NSMutableData *data;
    IOUSBHostIsochronousTransaction tx[UFRAMES_PER_REQ];
} IsoReq;

static struct {
    IOUSBHostInterface *intf;
    IOUSBHostPipe *out, *in;
    uint32_t outMax, inMax;
    uint32_t rate;
    IsoReq outReq[MAX_REQ], inReq[MAX_REQ];
    uint64_t nextOutFrame, nextInFrame;
    uint32_t accum;
    volatile int stopping;
    int outstanding;

    // tone
    double phase, phaseInc, amp;
    double phaseR, phaseIncR; // right channel tone (--right-freq), 0 = same as left

    // stats (only touched on the serial queue)
    uint64_t outFrames, inFrames, outErrors, inErrors, lateResync;
    uint64_t lastCompletion, maxGap; // mach ticks between capture completions
    int32_t peakL, peakR;
    uint64_t firstInTime, lastInTime, inFramesAtFirst;

    // implicit feedback: capture packet sizes (frames per microframe) drive playback packet sizes
    uint8_t fb[4096];
    uint32_t fbHead, fbTail;
    uint64_t fbUsed, fbNominal;
    int captureLive;

    FILE *wav;
    uint64_t wavBytes;
} S;

static void setRate(uint32_t rate) {
    NSError *err = nil;
    NSMutableData *d = [NSMutableData dataWithLength:3];
    uint8_t *p = d.mutableBytes;
    p[0] = rate & 0xff; p[1] = (rate >> 8) & 0xff; p[2] = (rate >> 16) & 0xff;
    IOUSBDeviceRequest req = {.bmRequestType = 0x22, .bRequest = 0x01, .wValue = 0x0100,
                              .wIndex = EP_IN, .wLength = 3};
    if (![S.intf sendDeviceRequest:req data:d bytesTransferred:NULL completionTimeout:1.0 error:&err])
        die([NSString stringWithFormat:@"set sample rate %u", rate], err);
}

static inline void put24(uint8_t *d, int32_t s) { d[0] = s; d[1] = s >> 8; d[2] = s >> 16; }
static inline int32_t get24(const uint8_t *s) { return (int32_t)((uint32_t)s[0] << 8 | (uint32_t)s[1] << 16 | (uint32_t)s[2] << 24) >> 8; }

static void fillOut(IsoReq *r) {
    uint8_t *base = r->data.mutableBytes;
    for (int i = 0; i < UFRAMES_PER_REQ; i++) {
        uint32_t frames;
        if (gFeedback && S.fbHead != S.fbTail) {
            frames = S.fb[S.fbTail++ & 4095];
            uint32_t maxFrames = S.outMax / BYTES_PER_FRAME;
            if (frames > maxFrames) frames = maxFrames;
            S.fbUsed++;
        } else {
            S.accum += S.rate;
            frames = S.accum / 8000;
            S.accum %= 8000;
            S.fbNominal++;
        }
        uint8_t *p = base + i * S.outMax;
        for (uint32_t f = 0; f < frames; f++) {
            int32_t v = (int32_t)lrint(sin(S.phase) * S.amp * 8388607.0);
            S.phase += S.phaseInc;
            if (S.phase > 2 * M_PI) S.phase -= 2 * M_PI;
            int32_t vr = v;
            if (S.phaseIncR > 0) {
                vr = (int32_t)lrint(sin(S.phaseR) * S.amp * 8388607.0);
                S.phaseR += S.phaseIncR;
                if (S.phaseR > 2 * M_PI) S.phaseR -= 2 * M_PI;
            }
            put24(p, v); put24(p + 3, vr);
            p += BYTES_PER_FRAME;
        }
        r->tx[i] = (IOUSBHostIsochronousTransaction){
            .status = kIOReturnInvalid, .requestCount = frames * BYTES_PER_FRAME,
            .offset = (uint32_t)(i * S.outMax)};
        S.outFrames += frames;
    }
}

static void prepIn(IsoReq *r) {
    for (int i = 0; i < UFRAMES_PER_REQ; i++)
        r->tx[i] = (IOUSBHostIsochronousTransaction){
            .status = kIOReturnInvalid, .requestCount = S.inMax, .offset = (uint32_t)(i * S.inMax)};
}

static void submitOut(int idx);
static void submitIn(int idx);

static uint64_t resyncFrame(void) {
    return [S.intf frameNumberWithTime:NULL] + 4;
}

static void submitOut(int idx) {
    if (S.stopping) { S.outstanding--; return; }
    IsoReq *r = &S.outReq[idx];
    fillOut(r);
    NSError *err = nil;
    for (int attempt = 0; attempt < 2; attempt++) {
        BOOL ok = [S.out enqueueIORequestWithData:r->data transactionList:r->tx
                             transactionListCount:UFRAMES_PER_REQ firstFrameNumber:S.nextOutFrame
                                          options:0 error:&err
                                completionHandler:^(IOReturn status, IOUSBHostIsochronousTransaction *tl) {
            if (status == kIOReturnAborted || S.stopping) { S.outstanding--; return; }
            for (int i = 0; i < UFRAMES_PER_REQ; i++)
                if (tl[i].status != kIOReturnSuccess) S.outErrors++;
            submitOut(idx);
        }];
        if (ok) { S.nextOutFrame++; return; }
        S.lateResync++;
        { IOUSBHostTime t; uint64_t cur = [S.intf frameNumberWithTime:&t];
          fprintf(stderr, "\n[out] enqueue frame %llu rejected (current %llu, delta %lld): 0x%08lx\n",
                  S.nextOutFrame, cur, (long long)(S.nextOutFrame - cur), (long)err.code); }
        S.nextOutFrame = resyncFrame();
    }
    fprintf(stderr, "\nplayback submit failed: %s\n", err.description.UTF8String);
    S.outstanding--;
}

static void handleIn(IOUSBHostIsochronousTransaction *tl, IsoReq *r) {
    const uint8_t *base = r->data.bytes;
    for (int i = 0; i < UFRAMES_PER_REQ; i++) {
        if (tl[i].status != kIOReturnSuccess && tl[i].status != kIOReturnUnderrun) { S.inErrors++; continue; }
        uint32_t frames = tl[i].completeCount / BYTES_PER_FRAME;
        if (frames) S.captureLive = 1;
        // only follow the device clock once capture is actually flowing; the device
        // sends empty packets until it has received playback data.
        if (S.captureLive && S.fbHead - S.fbTail < 4096) S.fb[S.fbHead++ & 4095] = (uint8_t)frames;
        const uint8_t *p = base + tl[i].offset;
        for (uint32_t f = 0; f < frames; f++, p += BYTES_PER_FRAME) {
            int32_t l = abs(get24(p)), rr = abs(get24(p + 3));
            if (l > S.peakL) S.peakL = l;
            if (rr > S.peakR) S.peakR = rr;
        }
        if (S.wav && frames) {
            fwrite(base + tl[i].offset, 1, frames * BYTES_PER_FRAME, S.wav);
            S.wavBytes += frames * BYTES_PER_FRAME;
        }
        if (frames) {
            if (!S.firstInTime) { S.firstInTime = tl[i].timeStamp; S.inFramesAtFirst = S.inFrames + frames; }
            S.lastInTime = tl[i].timeStamp;
        }
        S.inFrames += frames;
    }
}

static void submitIn(int idx) {
    if (S.stopping) { S.outstanding--; return; }
    IsoReq *r = &S.inReq[idx];
    prepIn(r);
    NSError *err = nil;
    for (int attempt = 0; attempt < 2; attempt++) {
        BOOL ok = [S.in enqueueIORequestWithData:r->data transactionList:r->tx
                            transactionListCount:UFRAMES_PER_REQ firstFrameNumber:S.nextInFrame
                                         options:0 error:&err
                               completionHandler:^(IOReturn status, IOUSBHostIsochronousTransaction *tl) {
            if (status == kIOReturnAborted || S.stopping) { S.outstanding--; return; }
            uint64_t now = mach_absolute_time();
            if (S.lastCompletion && now - S.lastCompletion > S.maxGap) S.maxGap = now - S.lastCompletion;
            S.lastCompletion = now;
            handleIn(tl, r);
            submitIn(idx);
        }];
        if (ok) { S.nextInFrame++; return; }
        S.lateResync++;
        { IOUSBHostTime t; uint64_t cur = [S.intf frameNumberWithTime:&t];
          fprintf(stderr, "\n[in] enqueue frame %llu rejected (current %llu, delta %lld): 0x%08lx\n",
                  S.nextInFrame, cur, (long long)(S.nextInFrame - cur), (long)err.code); }
        S.nextInFrame = resyncFrame();
    }
    fprintf(stderr, "\ncapture submit failed: %s\n", err.description.UTF8String);
    S.outstanding--;
}

static void wavHeader(FILE *f, uint32_t rate, uint32_t dataBytes) {
    uint8_t h[44];
    memcpy(h, "RIFF", 4);
    *(uint32_t *)(h + 4) = 36 + dataBytes;
    memcpy(h + 8, "WAVEfmt ", 8);
    *(uint32_t *)(h + 16) = 16;
    *(uint16_t *)(h + 20) = 1;     // PCM
    *(uint16_t *)(h + 22) = 2;     // channels
    *(uint32_t *)(h + 24) = rate;
    *(uint32_t *)(h + 28) = rate * BYTES_PER_FRAME;
    *(uint16_t *)(h + 32) = BYTES_PER_FRAME;
    *(uint16_t *)(h + 34) = 24;
    memcpy(h + 36, "data", 4);
    *(uint32_t *)(h + 40) = dataBytes;
    fseek(f, 0, SEEK_SET);
    fwrite(h, 1, 44, f);
}

static double dbfs(int32_t peak) { return peak > 0 ? 20 * log10(peak / 8388607.0) : -INFINITY; }

static double gRightFreq = 0;

static void cmdRun(uint32_t rate, double seconds, double freq, double levelDb, BOOL silence, const char *wavPath) {
    if (rate != 44100 && rate != 48000 && rate != 88200 && rate != 96000)
        die(@"rate must be 44100, 48000, 88200 or 96000", nil);

    dispatch_queue_t q = dispatch_queue_create("us122.io",
        dispatch_queue_attr_make_with_qos_class(DISPATCH_QUEUE_SERIAL, QOS_CLASS_USER_INTERACTIVE, 0));
    NSError *err = nil;
    S.intf = openInterface(q);
    S.rate = rate;
    S.phaseInc = 2 * M_PI * freq / rate;
    S.phaseIncR = 2 * M_PI * gRightFreq / rate;
    S.amp = silence ? 0 : pow(10, levelDb / 20);

    if (![S.intf selectAlternateSetting:US122_ALT error:&err]) die(@"select alt setting 1", err);
    S.out = [S.intf copyPipeWithAddress:EP_OUT error:&err];
    if (!S.out) die(@"open playback pipe 0x02", err);
    S.in = [S.intf copyPipeWithAddress:EP_IN error:&err];
    if (!S.in) die(@"open capture pipe 0x81", err);
    S.outMax = S.out.descriptors->descriptor.wMaxPacketSize & 0x7ff;
    S.inMax = S.in.descriptors->descriptor.wMaxPacketSize & 0x7ff;
    printf("playback EP 0x02 maxPacket %u, capture EP 0x81 maxPacket %u\n", S.outMax, S.inMax);
    uint32_t needed = (rate / 8000 + 1) * BYTES_PER_FRAME;
    if (S.outMax < needed) die([NSString stringWithFormat:@"playback maxPacket %u too small for %u Hz", S.outMax, rate], nil);

    setRate(rate);
    printf("sample rate set to %u Hz\n", rate);

    if (wavPath) {
        S.wav = fopen(wavPath, "wb");
        if (!S.wav) die([NSString stringWithFormat:@"open %s", wavPath], nil);
        wavHeader(S.wav, rate, 0);
    }

    for (int i = 0; i < gDepth; i++) {
        S.outReq[i].data = [S.intf ioDataWithCapacity:UFRAMES_PER_REQ * S.outMax error:&err];
        S.inReq[i].data = [S.intf ioDataWithCapacity:UFRAMES_PER_REQ * S.inMax error:&err];
        if (!S.outReq[i].data || !S.inReq[i].data) die(@"allocate I/O buffers", err);
        memset(S.outReq[i].data.mutableBytes, 0, S.outReq[i].data.length);
    }

    dispatch_sync(q, ^{
        uint64_t start = [S.intf frameNumberWithTime:NULL] + 4;
        S.nextOutFrame = S.nextInFrame = start;
        S.outstanding = 2 * gDepth;
        for (int i = 0; i < gDepth; i++) { submitOut(i); submitIn(i); }
    });
    printf("streaming%s for %.1f s (Ctrl-C to stop)\n",
           silence ? " silence" : [NSString stringWithFormat:@" %.0f Hz @ %.1f dBFS", freq, levelDb].UTF8String,
           seconds);

    signal(SIGINT, SIG_IGN);
    dispatch_source_t sig = dispatch_source_create(DISPATCH_SOURCE_TYPE_SIGNAL, SIGINT, 0, dispatch_get_main_queue());
    __block BOOL interrupted = NO;
    dispatch_source_set_event_handler(sig, ^{ interrupted = YES; });
    dispatch_resume(sig);

    mach_timebase_info_data_t tb;
    mach_timebase_info(&tb);
    NSDate *end = [NSDate dateWithTimeIntervalSinceNow:seconds];
    while (!interrupted && [end timeIntervalSinceNow] > 0) {
        [[NSRunLoop currentRunLoop] runUntilDate:[NSDate dateWithTimeIntervalSinceNow:1.0]];
        dispatch_sync(q, ^{
            double measured = 0;
            if (S.lastInTime > S.firstInTime) {
                double secs = (double)(S.lastInTime - S.firstInTime) * tb.numer / tb.denom / 1e9;
                measured = (S.inFrames - S.inFramesAtFirst) / secs;
            }
            printf("\rout %8llu fr  in %8llu fr  in-rate %9.2f Hz  peak L %6.1f R %6.1f dBFS  err out %llu in %llu  resync %llu  maxgap %.1f ms   ",
                   S.outFrames, S.inFrames, measured, dbfs(S.peakL), dbfs(S.peakR),
                   S.outErrors, S.inErrors, S.lateResync, (double)S.maxGap * tb.numer / tb.denom / 1e6);
            S.maxGap = 0;
            fflush(stdout);
            S.peakL = S.peakR = 0;
        });
    }
    printf("\nstopping (playback packets: %llu from feedback, %llu nominal)\n", S.fbUsed, S.fbNominal);

    dispatch_sync(q, ^{ S.stopping = 1; });
    [S.out abortWithError:nil];
    [S.in abortWithError:nil];
    for (int i = 0; i < 100; i++) {
        __block int left;
        dispatch_sync(q, ^{ left = S.outstanding; });
        if (left <= 0) break;
        usleep(10 * 1000);
    }
    [S.intf selectAlternateSetting:0 error:nil];
    [S.intf destroy];

    if (S.wav) {
        wavHeader(S.wav, rate, (uint32_t)S.wavBytes);
        fclose(S.wav);
        printf("wrote %s (%.2f s)\n", wavPath, (double)S.wavBytes / BYTES_PER_FRAME / rate);
    }
}

// ---------------------------------------------------------------- main

static void usage(void) {
    fprintf(stderr,
            "usage: us122ctl info\n"
            "       us122ctl run [--rate 48000] [--seconds 5] [--freq 440] [--level -18]\n"
            "                    [--silence] [--record out.wav] [--depth 16]\n"
            "                    [--no-feedback]\n");
    exit(64);
}

int main(int argc, const char *argv[]) {
    @autoreleasepool {
        if (argc < 2) usage();
        NSString *cmd = @(argv[1]);
        if ([cmd isEqualToString:@"info"]) {
            cmdInfo();
        } else if ([cmd isEqualToString:@"run"]) {
            uint32_t rate = 48000;
            double seconds = 5, freq = 440, level = -18;
            BOOL silence = NO;
            const char *wav = NULL;
            for (int i = 2; i < argc; i++) {
                NSString *a = @(argv[i]);
                BOOL hasVal = i + 1 < argc;
                if ([a isEqualToString:@"--rate"] && hasVal) rate = (uint32_t)atoi(argv[++i]);
                else if ([a isEqualToString:@"--seconds"] && hasVal) seconds = atof(argv[++i]);
                else if ([a isEqualToString:@"--freq"] && hasVal) freq = atof(argv[++i]);
                else if ([a isEqualToString:@"--level"] && hasVal) level = atof(argv[++i]);
                else if ([a isEqualToString:@"--record"] && hasVal) wav = argv[++i];
                else if ([a isEqualToString:@"--depth"] && hasVal) gDepth = atoi(argv[++i]);
                else if ([a isEqualToString:@"--no-feedback"]) gFeedback = 0;
                else if ([a isEqualToString:@"--right-freq"] && hasVal) gRightFreq = atof(argv[++i]);
                else if ([a isEqualToString:@"--silence"]) silence = YES;
                else usage();
            }
            if (level > 0) level = 0;
            if (gDepth < 2 || gDepth > MAX_REQ) usage();
            cmdRun(rate, seconds, freq, level, silence, wav);
        } else {
            usage();
        }
    }
    return 0;
}
