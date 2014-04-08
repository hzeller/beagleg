#!/bin/bash
# Run as root.
# What is happening here is well explained in Derek Molloy's excellent
# video http://www.youtube.com/watch?v=wui_wU1AeQc
##

VERBOSE=0
BIN_DTB=BeagleG-00A0.dtbo

if [ ! -e $BIN_DTB ] ; then
    echo "Need $BIN_DTB"
    exit 1
fi

PINS=/sys/kernel/debug/pinctrl/44e10800.pinmux/pins
SLOTS=/sys/devices/bone_capemgr.*/slots

# Some dance around minimal tools available on the system. We get the offsets
# and add 44e10800 to it, so that we can grep these in the $PINS
OFFSETS_800=$(for f in $(cat BeagleG.dts | grep 0x | awk '{print $1}') ; do \
                 printf "%0d" $f | awk '{printf("44e10%03x\n", $1 + 2048)}'; \
              done)

if [ $VERBOSE -ne 0 ] ; then
    echo "This is how these pins look before."
    for f in $OFFSETS_800 ; do
	grep $f $PINS
    done
fi

cp $BIN_DTB /lib/firmware

echo
echo "Adding BeagleG overlay"
echo "BeagleG" > $SLOTS
cat $SLOTS

if [ $VERBOSE -ne 0 ] ; then
    echo
    echo "This is how these pins look afterwards."
    for f in $OFFSETS_800 ; do
	grep $f $PINS
    done
fi


