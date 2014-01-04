# (c) 2013 h.zeller@acm.org
# This is free software. License: GNU 3.0

# Stuff based on the environment. This assumes stuff to be compiled on the
# beaglebone and the am335x_pru_package checked out.
# https://github.com/beagleboard/am335x_pru_package

#CROSS_COMPILE?=arm-arago-linux-gnueabi-
CROSS_COMPILE=
AM335_BASE=../../am335x_pru_package/pru_sw
PASM=$(AM335_BASE)/utils/pasm
LIBDIR_APP_LOADER?=$(AM335_BASE)/app_loader/lib
INCDIR_APP_LOADER?=$(AM335_BASE)/app_loader/include

CFLAGS+= -Wall -I$(INCDIR_APP_LOADER) -std=c99 -D__DEBUG -O2 -mtune=cortex-a8 -march=armv7-a
LDFLAGS+=-L$(LIBDIR_APP_LOADER) -lprussdrv -lpthread -lm

OBJECTS=motor-interface.o gcode-parser.o send-gcode.o
PRU_BIN=motor-control_bin.h
TARGET=send-gcode

all : $(TARGET)

%.o: %.c
	$(CROSS_COMPILE)gcc $(CFLAGS) -c -o $@ $< 

$(TARGET): $(OBJECTS)
	$(CROSS_COMPILE)gcc $(CFLAGS) -o $@ $^ $(LDFLAGS)

%_bin.h : %.p
	$(PASM) -V3 -c $<

motor-interface.o : shared-constants.h $(PRU_BIN)
motor-control.bin : shared-constants.h

clean:
	rm -rf $(TARGET) $(OBJECTS) $(PRU_BIN)
