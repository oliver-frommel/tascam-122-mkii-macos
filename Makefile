CFLAGS = -fobjc-arc -O2 -Wall -Wextra -Wno-unused-parameter -mmacosx-version-min=12.0
FRAMEWORKS = -framework Foundation -framework IOKit -framework IOUSBHost -framework CoreAudio -framework CoreFoundation
DRIVER_FRAMEWORKS = $(FRAMEWORKS) -framework CoreMIDI

DRIVER = build/US122MKII.driver
HAL_DIR = /Library/Audio/Plug-Ins/HAL

all: build/us122ctl build/enginetest build/haltest $(DRIVER)

build/us122ctl: tools/us122ctl.m
	@mkdir -p build
	clang $(CFLAGS) -o $@ $< $(FRAMEWORKS)

build/enginetest: tools/enginetest.m src/US122Engine.m src/US122Engine.h
	@mkdir -p build
	clang $(CFLAGS) -o $@ tools/enginetest.m src/US122Engine.m $(FRAMEWORKS)

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

# needs root: sudo make install (run plain `make` first; install never builds, so build/ stays user-owned)
install:
	@test -d $(DRIVER) || { echo "run 'make' first"; exit 1; }
	rm -rf $(HAL_DIR)/US122MKII.driver
	cp -R $(DRIVER) $(HAL_DIR)/
	killall coreaudiod || true

uninstall:
	rm -rf $(HAL_DIR)/US122MKII.driver
	killall coreaudiod || true

clean:
	rm -rf build

# regenerate the device icon (needs swift; the .icns is checked in)
icon:
	swift tools/makeicon.swift build/US122MKII.iconset
	iconutil -c icns build/US122MKII.iconset -o plugin/US122MKII.icns

.PHONY: all clean install uninstall icon
