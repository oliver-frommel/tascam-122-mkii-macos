# TASCAM US-122MKII on modern macOS

Userspace driver for the TASCAM US-122MKII (USB `0644:8021`). No kernel
extension and no SIP changes: it talks to the device through Apple's
IOUSBHost framework.

## Motivation

The Tascam US-122MKII is a capable USB Audio/MIDI interface but it's not "class compliant" therefore won't work without a special driver. The latest drivers for macOS are for version 10.14. This driver makes the device work on modern macOS.

## AI policy

This was purely vibe coded with an LLM.

## Protocol

Taken from serifpersia's Linux driver
(<https://github.com/serifpersia/us144mkii/tree/us122mkii>, GPL-2.0), used as a
protocol reference only:

| What | Value |
|---|---|
| Interface / alt setting | 1 / 1 |
| Sample rate | UAC1 `SET_CUR` (bmRequestType `0x22`, bRequest `0x01`, wValue `0x0100`, wIndex `0x81`), 3-byte LE rate. 44.1/48/88.2/96 kHz |
| Playback | EP `0x02` isochronous OUT |
| Capture | EP `0x81` isochronous IN |
| Format | 2 ch, 24-bit LE packed, 6 bytes/frame |
| Timing | high-speed, one packet per 125 µs microframe (6 frames at 48 kHz), no feedback endpoint |

## Layout

| Path | What |
|---|---|
| `src/US122Engine.{h,m}` | USB engine: hot-plug, iso streaming, clock model, ring buffers |
| `src/US122Driver.c` | AudioServerPlugIn (Core Audio device "US-122MKII") |
| `plugin/Info.plist` | bundle plist (declares the IOUSBHost user-client class for the sandbox) |
| `tools/us122ctl.m` | low-level protocol test (descriptors, raw streaming) |
| `tools/enginetest.m` | drives the engine like the HAL would, without coreaudiod |
| `tools/haltest.m` | plays/records through the installed Core Audio device |

## Clocking

The iso endpoints are asynchronous with no feedback endpoint. The engine counts
capture frames per USB microframe to estimate the device clock
(`S(m) = C0 + R·(m − m0)`), maps microframes to host time via IOUSBHost's
reference microframe, and uses that model for Core Audio zero timestamps and
for sizing playback packets, so playback follows the device clock exactly.
Measured device clock: about 47,999.6 Hz at a nominal 48 kHz (around 7–25 ppm slow, depending on the measurement).

## Scheduling and latency

Measured on macOS 26 (Apple Silicon) with `enginetest --probe`:

- USB completions reach userspace through GCD and are sometimes 10–20 ms late,
  while a dedicated real-time thread never wakes more than 0.1 ms late. GCD
  also resets a worker thread's real-time policy whenever it parks.
- The `enqueueIORequest` call itself sometimes blocks for 8–13 ms inside the
  kernel. This happens on either pipe and on separate threads, so it is a
  global lock in the IOUSBHost path.

So completions don't drive submission. Two real-time **pump threads**, one per
pipe, wake every 1 ms and keep the queue a fixed number of frames ahead of the
bus. Completions only hand buffers back (pool of 64) and deliver capture data.
Capture requests go 32 frames ahead, which adds no latency. Output goes
`US122_DEPTH` frames ahead, and that lead has to outlast the kernel blocks:

| output lead | dropouts / 3 min |
|---|---|
| 8 ms  | ~14 |
| 12 ms | 2 |
| 16 ms | 0 (default) |

Input samples that haven't arrived when the HAL reads them come back as
silence. Before this change they returned whatever the ring held a lap
earlier.

## Build & install

```sh
make                    # builds tools + build/US122MKII.driver
sudo make install       # copies to /Library/Audio/Plug-Ins/HAL, restarts coreaudiod
sudo make uninstall
```

Logs: `log stream --predicate 'subsystem == "local.us122mkii.driver"'`

Tuning (environment, engine tests only): `US122_DEPTH` sets the output lead in
ms (default 16; output safety offset is lead + 2 ms). `US122_IN_SAFETY_MS`
sets the input safety offset (default 8).
`build/enginetest --depth N --probe` reports dropouts, kernel enqueue stalls
and real-time wake-up jitter.

## Status

- [x] Playback and capture at 44.1/48/88.2/96 kHz
- [x] Capture verified with a real source (guitar on input R, 48 kHz)
- [x] Sample-rate switching while playing (~130 ms gap per switch)
- [x] Core Audio device with hot-plug
- [x] Output volume/mute (software gain), persisted with the sample rate
- [x] Empty/late capture packets treated as lost
- [x] Watchdog: restarts a stalled stream on the same sample timeline
      (test: `build/enginetest --kill-at 3`)
- [x] Unplug/replug while playing, sleep/wake (quiet shutdown on removal)
- [x] Real-time pump threads decoupled from completion delivery
- [ ] Long soak (10 min+) through Core Audio with the pump build
- [ ] Lower latency: limited by kernel enqueue stalls (see above); 16 ms output lead
- [x] Silences the device's power-up noise: configures it (alt setting +
      rate) within a few ms of connect
- [ ] MIDI: device transport verified by loopback (MIDI OUT cabled to MIDI
      IN, `enginetest --midi`): note messages and 1500-byte bursts arrive
      intact. The device transmits only the first 4 bytes of each OUT packet.
      Not yet visible to apps: the Core Audio helper's sandbox refuses CoreMIDI
      clients (-10833), so the ports need a relay outside the sandbox.

## License

Copyright (C) 2026 Oliver Frommel

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version. See [LICENSE](LICENSE) for the full text.

The Linux driver [us144mkii](https://github.com/serifpersia/us144mkii) was used
as a reference for the device protocol only; no code was taken from it.
