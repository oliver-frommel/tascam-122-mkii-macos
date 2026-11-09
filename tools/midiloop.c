// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Oliver Frommel

// midiloop — end-to-end MIDI test through CoreMIDI, like an app would use it.
// With the US-122MKII's MIDI OUT cabled to its MIDI IN, sends notes and a
// sysex message to the "US-122MKII" destination and checks they come back on
// the source.
//
//   midiloop [--seconds 5] [--sysex 1000]

#include <CoreMIDI/CoreMIDI.h>
#include <mach/mach_time.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static uint8_t gRx[65536];
static _Atomic size_t gRxN;
static _Atomic uint64_t gFirstRx;

static MIDIEndpointRef find(bool source) {
    ItemCount n = source ? MIDIGetNumberOfSources() : MIDIGetNumberOfDestinations();
    for (ItemCount i = 0; i < n; i++) {
        MIDIEndpointRef e = source ? MIDIGetSource(i) : MIDIGetDestination(i);
        MIDIEntityRef ent; MIDIDeviceRef dev;
        if (MIDIEndpointGetEntity(e, &ent) || MIDIEntityGetDevice(ent, &dev)) continue;
        CFStringRef name = NULL;
        MIDIObjectGetStringProperty(dev, kMIDIPropertyName, &name);
        bool match = name && CFStringCompare(name, CFSTR("US-122MKII"), 0) == kCFCompareEqualTo;
        if (name) CFRelease(name);
        if (match) return e;
    }
    return 0;
}

static void sendBytes(MIDIPortRef out, MIDIEndpointRef dst, const uint8_t *b, size_t n) {
    static uint8_t mem[70000];
    MIDIPacketList *pl = (MIDIPacketList *)mem;
    MIDIPacket *p = MIDIPacketListInit(pl);
    p = MIDIPacketListAdd(pl, sizeof mem, p, 0, n, b);
    if (p) MIDISend(out, dst, pl);
}

int main(int argc, char **argv) {
    double seconds = 5;
    size_t sysexLen = 1000;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--seconds") && i + 1 < argc) seconds = atof(argv[++i]);
        else if (!strcmp(argv[i], "--sysex") && i + 1 < argc) sysexLen = (size_t)atol(argv[++i]);
        else { fprintf(stderr, "usage: midiloop [--seconds 5] [--sysex 1000]\n"); return 64; }
    }
    MIDIClientRef client;
    if (MIDIClientCreate(CFSTR("midiloop"), NULL, NULL, &client)) { fprintf(stderr, "no CoreMIDI client\n"); return 1; }
    MIDIEndpointRef src = find(true), dst = find(false);
    if (!src || !dst) { fprintf(stderr, "US-122MKII MIDI ports not found\n"); return 1; }
    SInt32 offline = 0;
    MIDIEndpointRef ent; MIDIDeviceRef dev;
    MIDIEndpointGetEntity(src, &ent); MIDIEntityGetDevice(ent, &dev);
    MIDIObjectGetIntegerProperty(dev, kMIDIPropertyOffline, &offline);
    printf("found US-122MKII ports (%s)\n", offline ? "offline" : "online");

    MIDIPortRef in, out;
    MIDIInputPortCreateWithBlock(client, CFSTR("in"), &in, ^(const MIDIPacketList *pl, void *ref) {
        const MIDIPacket *p = &pl->packet[0];
        for (UInt32 i = 0; i < pl->numPackets; i++) {
            size_t at = atomic_fetch_add(&gRxN, p->length);
            if (at + p->length <= sizeof gRx) memcpy(gRx + at, p->data, p->length);
            uint64_t zero = 0;
            atomic_compare_exchange_strong(&gFirstRx, &zero, mach_absolute_time());
            p = MIDIPacketNext(p);
        }
    });
    MIDIOutputPortCreate(client, CFSTR("out"), &out);
    MIDIPortConnectSource(in, src, NULL);

    mach_timebase_info_data_t tb; mach_timebase_info(&tb);
    usleep(1500 * 1000); // let the device stream start (input now has a listener)

    // expected stream: one note on/off per 100 ms, then one sysex
    uint8_t exp[65536]; size_t en = 0;
    int notes = (int)(seconds * 10) - 10;
    if (notes < 1) notes = 1;
    // discard anything that arrived before we started (e.g. bytes left in the device)
    size_t stale = atomic_exchange(&gRxN, 0);
    if (stale) {
        printf("discarded %zu stale bytes received before sending:", stale);
        for (size_t i = 0; i < stale && i < 48; i++) printf(" %02X", gRx[i]);
        printf("\n");
    }
    atomic_store(&gFirstRx, 0);
    uint64_t t0 = mach_absolute_time();
    for (int k = 0; k < notes; k++) {
        uint8_t m[6] = {0x90, (uint8_t)(48 + k % 24), 100, 0x80, (uint8_t)(48 + k % 24), 0};
        sendBytes(out, dst, m, 6);
        memcpy(exp + en, m, 6); en += 6;
        usleep(100 * 1000);
    }
    if (sysexLen >= 3) {
        static uint8_t sx[65536];
        sx[0] = 0xF0;
        for (size_t i = 1; i < sysexLen - 1; i++) sx[i] = (uint8_t)(i % 128);
        sx[sysexLen - 1] = 0xF7;
        sendBytes(out, dst, sx, sysexLen);
        memcpy(exp + en, sx, sysexLen); en += sysexLen;
    }
    // wait for the tail (31.25 kbaud: ~3 bytes/ms)
    int waits = (int)(en / 300) + 30; // 100 ms steps: wire time plus 3 s
    for (int w = 0; w < waits && atomic_load(&gRxN) < en; w++) usleep(100 * 1000);
    usleep(200 * 1000);

    size_t rn = atomic_load(&gRxN);
    uint64_t first = atomic_load(&gFirstRx);
    printf("sent %zu bytes (%d note pairs + %zu-byte sysex), received %zu\n", en, notes, sysexLen, rn);
    if (first > t0) printf("first byte back after %.1f ms\n", (double)(first - t0) * tb.numer / tb.denom / 1e6);
    if (rn == en && !memcmp(gRx, exp, en)) { printf("PASS: identical\n"); return 0; }
    for (size_t i = 0; i < rn && i < en; i++)
        if (gRx[i] != exp[i]) { printf("FAIL: first difference at byte %zu: got %02X, expected %02X\n", i, gRx[i], exp[i]); return 1; }
    printf("FAIL: %s\n", rn < en ? "bytes missing" : "extra bytes");
    return 1;
}
