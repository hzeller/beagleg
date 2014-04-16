#!/bin/bash
##

set -e

CAPE_NAME=BeagleG
BOOT_PARAM="capemgr.enable_partno=$CAPE_NAME"
BOOT_DIR=/boot/uboot
UENV_FILE=$BOOT_DIR/uEnv.txt
INITRD_FILE=$BOOT_DIR/uInitrd

if [ $# -ne 1 ] ; then
    echo "usage: $0 <path-to-dtbo-file>"
    exit 1
fi

DTBO_FILE=$(readlink -f $1)
if [ ! -r $DTBO_FILE ] ; then
    echo "Can't read $DTBO_FILE"
    exit 1
fi

# First: set up UENV_FILE.
echo "* Setting up $UENV_FILE"
# Note, this only works properly if optargs is not set yet.
if grep $BOOT_PARAM $UENV_FILE ; then
    echo "  * Already configured in $UENV_FILE"
else
    echo "  * Adding $BOOT_PARAM to $UENV_FILE"
    echo "optargs=$BOOT_PARAM" >> $UENV_FILE
fi

echo "* Storing the $DTBO_FILE in $INITRD_FILE"
TEMP_BASE=/tmp/new-boot.$$

# Unpack uInitrd, copy dtbo file into it.
echo "* Unpacking initrd and adding $DTBO_FILE"
mkdir -p ${TEMP_BASE}/tree
cd ${TEMP_BASE}/tree
dd if=${INITRD_FILE} skip=1 bs=64 | gzip -d | cpio -id
mkdir -p lib/firmware
cp $DTBO_FILE lib/firmware

# Re-pack
echo "* Repacking initrd"
find . | cpio -o -H newc | gzip -9 > ${TEMP_BASE}/initrd.mod.gz
mkimage -A arm -O linux -T ramdisk -C none -a 0 -e 0 -n initramfs -d ${TEMP_BASE}/initrd.mod.gz ${TEMP_BASE}/uInitrd.new

echo "* Replace $INITRD_FILE with new file"
cp ${INITRD_FILE} ${INITRD_FILE}.orig
cp ${TEMP_BASE}/uInitrd.new ${INITRD_FILE}.new
mv ${INITRD_FILE}.new ${INITRD_FILE}

sync

echo "Now reboot for the cape to be initialized at boot time"