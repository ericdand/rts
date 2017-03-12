# --- mega2560 ide 1.6
BOARD_TAG    = mega
BOARD_SUB    = atmega2560
MONITOR_PORT = /dev/tty.usbmodem1421
ARDUINO_DIR   = /Users/eric/Applications/Arduino.app/Contents/Java
USER_LIB_PATH := $(realpath ../libraries)
# The default serial monitor used by targets like "monitor" and "test" is
# screen. Uncomment the following lines to switch it to picocom.
# MONITOR_CMD		= picocom
# MONITOR_PARAMS	= -c		# -c enables "local echo".
include /usr/local/opt/arduino-mk/Arduino.mk

# The test target builds and uploads the code, then connects the serial
# monitor, expecting test-related output from the device.
test:	upload monitor

.PHONY: test

