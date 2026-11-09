CFLAGS = -fobjc-arc -O2 -Wall -Wextra -Wno-unused-parameter -mmacosx-version-min=12.0
FRAMEWORKS = -framework Foundation -framework IOKit -framework IOUSBHost -framework CoreAudio -framework CoreFoundation
DRIVER_FRAMEWORKS = $(FRAMEWORKS)

DRIVER = build/US122MKII.driver
HAL_DIR = /Library/Audio/Plug-Ins/HAL
MIDI_PLUGIN = build/US122MIDI.plugin
MIDI_DIR = /Library/Audio/MIDI Drivers
RELAY = build/us122-midirelay
RELAY_BIN = /usr/local/libexec/us122-midirelay
RELAY_PLIST = /Library/LaunchDaemons/local.us122mkii.midirelay.plist

all: build/us122ctl build/enginetest build/haltest build/midiloop $(DRIVER) $(MIDI_PLUGIN) $(RELAY)

build/us122ctl: tools/us122ctl.m
	@mkdir -p build
	clang $(CFLAGS) -o $@ $< $(FRAMEWORKS)

build/enginetest: tools/enginetest.m src/US122Engine.m src/US122Engine.h
	@mkdir -p build
	clang $(CFLAGS) -o $@ tools/enginetest.m src/US122Engine.m $(FRAMEWORKS)

build/midiloop: tools/midiloop.c
	@mkdir -p build
	clang $(CFLAGS) -o $@ $< -framework CoreMIDI -framework CoreFoundation

build/haltest: tools/haltest.m
	@mkdir -p build
	clang $(CFLAGS) -o $@ $< -framework Foundation -framework CoreAudio

$(DRIVER): src/US122Driver.c src/US122Engine.m src/US122Engine.h plugin/Info.plist plugin/US122MKII.icns
	@mkdir -p $(DRIVER)/Contents/MacOS $(DRIVER)/Contents/Resources
	cp plugin/Info.plist $(DRIVER)/Contents/Info.plist
	cp plugin/US122MKII.icns $(DRIVER)/Contents/Resources/
	clang $(CFLAGS) -bundle -fvisibility=hidden -o $(DRIVER)/Contents/MacOS/US122MKII \
		src/US122Driver.c src/US122Engine.m $(DRIVER_FRAMEWORKS)
	codesign --force --sign - $(DRIVER)

$(MIDI_PLUGIN): midi/US122MIDIDriver.c midi/Info.plist
	@mkdir -p $(MIDI_PLUGIN)/Contents/MacOS
	cp midi/Info.plist $(MIDI_PLUGIN)/Contents/Info.plist
	clang $(CFLAGS) -bundle -fvisibility=hidden -o $(MIDI_PLUGIN)/Contents/MacOS/US122MIDI \
		midi/US122MIDIDriver.c -framework CoreMIDI -framework CoreAudio -framework CoreFoundation
	codesign --force --sign - $(MIDI_PLUGIN)

$(RELAY): midi/relay.c
	@mkdir -p build
	clang $(CFLAGS) -o $@ $<
	codesign --force --sign - $@

# needs root: sudo make install (run plain `make` first; install never builds, so build/ stays user-owned)
install:
	@test -d $(DRIVER) -a -d $(MIDI_PLUGIN) -a -f $(RELAY) || { echo "run 'make' first"; exit 1; }
	@# leftovers of the sandbox test (Nov 2026); harmless if absent
	-launchctl bootout system/local.us122mkii.probe.daemon 2>/dev/null
	-launchctl bootout gui/$(or $(SUDO_UID),501)/local.us122mkii.probe.agent 2>/dev/null
	rm -f /Library/LaunchDaemons/local.us122mkii.probe.daemon.plist /Library/LaunchAgents/local.us122mkii.probe.agent.plist /usr/local/libexec/us122-machprobe
	@# MIDI relay daemon
	mkdir -p /usr/local/libexec
	-launchctl bootout system/local.us122mkii.midirelay 2>/dev/null
	cp $(RELAY) $(RELAY_BIN)
	cp midi/local.us122mkii.midirelay.plist $(RELAY_PLIST)
	launchctl bootstrap system $(RELAY_PLIST)
	@# CoreMIDI driver
	mkdir -p "$(MIDI_DIR)"
	rm -rf "$(MIDI_DIR)/US122MIDI.plugin"
	cp -R $(MIDI_PLUGIN) "$(MIDI_DIR)/"
	@# audio driver
	rm -rf $(HAL_DIR)/US122MKII.driver
	cp -R $(DRIVER) $(HAL_DIR)/
	killall coreaudiod || true
	killall MIDIServer || true

uninstall:
	rm -rf $(HAL_DIR)/US122MKII.driver
	rm -rf "$(MIDI_DIR)/US122MIDI.plugin"
	-launchctl bootout system/local.us122mkii.midirelay 2>/dev/null
	rm -f $(RELAY_PLIST) $(RELAY_BIN)
	killall coreaudiod || true
	killall MIDIServer || true

clean:
	rm -rf build

# regenerate the device icon (needs swift; the .icns is checked in)
icon:
	swift tools/makeicon.swift build/US122MKII.iconset
	iconutil -c icns build/US122MKII.iconset -o plugin/US122MKII.icns

.PHONY: all clean install uninstall icon
