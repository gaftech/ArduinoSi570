# Makefile, using Arduino-Makefile (https://github.com/gaftech/Arduino-Makefile)

# Hardware
BOARD_TAG			= uno
ARDUINO_PORT		= /dev/ttyUSB0

# Needed libs
ARDUINO_LIBS		= Wire Wire/utility ArduinoSi570

# libs locations
ARDUINO_DIR			= /home/gabriel/usr/arduino
ARDMK_DIR     		= /home/gabriel/dev/arduino/Arduino-Makefile
AVR_TOOLS_DIR	 	= /usr
ARDUINO_SKETCHBOOK	= /home/gabriel/dev/arduino

CPPFLAGS 			+= -DSI570_DEBUG

include $(ARDMK_DIR)/arduino-mk/Arduino.mk

