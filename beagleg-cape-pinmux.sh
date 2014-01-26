#!/bin/bash
# Run as root.
# What is happening here is well explained in Derek Molloy's excellent
# video http://www.youtube.com/watch?v=wui_wU1AeQc
##

BIN_DTB=BeagleG-00A0.dtbo

if [ ! -e $BIN_DTB ] ; then
    echo "Need $BIN_DTB"
    exit 1
fi

PINS=/sys/kernel/debug/pinctrl/44e10800.pinmux/pins
SLOTS=/sys/devices/bone_capemgr.9/slots

OFFSETS_800=$(cat BeagleG.dts | grep 0x | awk '{printf("%03x\n", $1 + 2048);}')

echo "This is how these pins look before."
for f in $OFFSETS_800 ; do
    grep $f $PINS
done

cp $BIN_DTB /lib/firmware

echo
echo "Adding BeagleG overlay"
echo "BeagleG" > $SLOTS
cat $SLOTS

echo
echo "This is how these pins look afterwards"
for f in $OFFSETS_800 ; do
    grep $f $PINS
done

