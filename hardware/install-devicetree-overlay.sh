#!/bin/bash
# TODO: make more robust - this might change depending on the distribution.
# Maybe there are already standard scripts available on the distribution that
# do these things ?
#
# Essentially what we are doing is to embed the DTBO file in the initrd file, so that it
# is available right away on boot-up. That is the quickest way to enable the cape at boot-up
# short of compiling the device-tree file into the kernel.
##

set -e

DT_VERSION=00A0

usage() {
    echo "Usage: $0 <dts-file>"
    echo "Available:"
    for f in $(ls `dirname $0`/*/*.dts) ; do
	echo "  $f"
    done
    exit 1
}

if [ $# -ne 1 ] ; then
    usage
fi

DTS_FILE=$1
if [[ ! $DTS_FILE =~ \.dts$ ]] ; then
    echo "This does not seem to be a *.dts file".
    usage
fi

# ... there must be a simpler way to extract this...
CAPE_NAME=$(sed 's/.*part-number\s*=\s*"\([^"]*\)".*/\1/p;d' < $DTS_FILE)

if [ -z "$CAPE_NAME" ] ; then
    echo "Didn't find any part-number in $DTS_FILE ?"
    exit
fi

DTBO_FILE=$(echo "$DTS_FILE" | sed "s/.dts$/-$DT_VERSION.dtbo/")

make $DTBO_FILE
if [ $? -ne 0 ] ; then
    echo "Failed to produce $DTBO_FILE"
fi

# Make this an absolute filename
DTBO_FILE=$(readlink -f $DTBO_FILE)

BOOT_PARAM="capemgr.enable_partno=$CAPE_NAME"
BOOT_DIR=/boot/uboot
UENV_FILE=$BOOT_DIR/uEnv.txt
INITRD_FILE=$BOOT_DIR/uInitrd

if [ $# -ne 1 ] ; then
    echo "usage: $0 <path-to-dtbo-file>"
    exit 1
fi

# First: set up UENV_FILE.
echo "* Setting up $UENV_FILE"
# Note, this only works properly if optargs is not set yet.
if grep $BOOT_PARAM $UENV_FILE > /dev/null ; then
    echo "  * Already configured in $UENV_FILE"
else
    echo "  * Adding $BOOT_PARAM to $UENV_FILE"
    echo "optargs=$BOOT_PARAM" >> $UENV_FILE
fi

echo "* Storing the $DTBO_FILE in $INITRD_FILE"
TEMP_BASE=/tmp/new-boot.$$

# Unpack uInitrd, copy dtbo file into it.
echo "* Unpacking initrd and adding $DTBO_FILE to its lib/firmware"
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
