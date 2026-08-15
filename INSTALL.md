# INSTALLATION

## Get one of the latest linux Debian images

Download one of the latest debian images provided by the following [**list**](https://beagleboard.org/latest-images). This installation guide refers to
[AM3358 Debian 10.3 2020-04-06 4GB SD IoT](https://debian.beagleboard.org/images/bone-debian-10.3-iot-armhf-2020-04-06-4gb.img.xz)

Always use the most minimal image you can find, e.g. no graphical user
interface etc.

## Flash the SD card; Boot; SSH into your Beaglebone

This documentation is provided elsewhere already, see
https://beagleboard.org/getting-started

In short: unpack the xz-packed image, and place as-is on the SD card (On
Linux, use `dd`, on other platforms they have more complicated graphical tools).

Then boot the Beaglebone and connect via ssh to it (
`ssh debian@beaglebone.local`, default password `temppwd`).

## Prepare the environment

### If you have an old Beaglebone

If you have your Beaglebone lying around in a drawer for a while, it is probably
good to update the uboot on the eMMC to the latest version.

```
sudo apt install bb-u-boot-am335x-evm
sudo /opt/u-boot/bb-u-boot-am335x-evm/install-mmcblk0.sh
sudo /opt/u-boot/bb-u-boot-am335x-evm/install-mmcblk1.sh
```

### Enable PRU
To enable the PRU the way we use it, we need to `/boot/uEnv.txt` and
enable the correct `uboot_overlay_pru` line.

We need to _disable_ the line containing `PRU-RPROC` (add a `#` in front) and
_enable_ the line containing the `PRU-UIO` (remove `#` in front); so it will
look like this:

```
###PRUSS OPTIONS
###pru_rproc (4.4.x-ti kernel)
#uboot_overlay_pru=/lib/firmware/AM335X-PRU-RPROC-4-4-TI-00A0.dtbo
###pru_uio (4.4.x-ti, 4.14.x-ti & mainline/bone kernel)
uboot_overlay_pru=/lib/firmware/AM335X-PRU-UIO-00A0.dtbo
```

### Alternative: images without uio_pruss (PRU_BACKEND=genirq)

Recent BeagleBone Debian images (mainline kernel) ship neither the
`uio_pruss` driver nor the `PRU-UIO` overlay above. There, build
BeagleG with `make PRU_BACKEND=genirq`: it needs no PRU kernel driver
at all — the PRU memories and the motion interrupt are exposed to
BeagleG by the kernel's generic UIO driver, so PRU access is governed
by the permissions of `/dev/uio0` alone.

Which backend do I need? The kernel flavor suffix alone does not tell
you. The `uio_pruss` driver was removed from mainline Linux in 2024
("uio: pruss: Remove this driver"), so kernel series 6.12 and newer
lack it on either flavor, while series branched earlier still ship it
(TI kernels up to 5.10, `-bone` kernels up to 6.6). Check your kernel
directly:

```
ls /lib/modules/$(uname -r)/kernel/drivers/uio/
```

If `uio_pruss.ko*` is listed, the default `uio` backend and the
`PRU-UIO` overlay above work. If it is not — e.g. the 6.12 kernels of
current images — use `genirq`; it works on any kernel >= 5.10, either
flavor.

One-time setup: compile the overlay that exports the PRU to userspace

```
sudo apt install device-tree-compiler
sudo dtc -@ -o /lib/firmware/BEAGLEG-PRU-IRQ.dtbo dts/BEAGLEG-PRU-IRQ.dts
```

then in `/boot/uEnv.txt` enable u-boot overlays, select ours, and
allow `uio_pdrv_genirq` to bind to it (append to the existing
`cmdline=` line):

```
enable_uboot_overlays=1
uboot_overlay_pru=/lib/firmware/BEAGLEG-PRU-IRQ.dtbo
cmdline=coherent_pool=1M net.ifnames=0 quiet uio_pdrv_genirq.of_id=generic-uio
```

Use the `uboot_overlay_pru` slot specifically, not a generic
`uboot_overlay_addrN` one: overlays merge last-writer-wins, and
u-boot applies the PRU slot after all the generic cape slots, so this
overlay's claim on PRU0 (it disables the PRU0 remoteproc node so the
kernel and BeagleG never fight over the core) cannot be silently
overridden by a cape overlay.

After a reboot, `cat /sys/class/uio/uio0/name` should print
`beagleg_pru_irq` (newer kernels append the node's `@4a300000`
unit-address). BeagleG needs read/write access to that `/dev/uio0`
device node.

## Install BeagleG


Let's start from fetching the BeagleG repository with:

```
git clone --recursive https://github.com/hzeller/beagleg.git
```

change directory into the repository and run `make`.

The resulting `machine-control` binary will be in the toplevel directory. You
can `sudo make install` it, or run it right there.

Then [set up your hardware](./hardware/) and possibly create the necessary
systemd configuration for a set-up that starts on boot.

# TROUBLESHOOTING

In general, make sure to have the latest Beaglebone Debian image; most of
earlier images have various problems that are not covered here for brevity.

In any case of trouble, make sure to have your system up-to-date

```
sudo apt-get update
sudo apt-get upgrade
```

... and have a fresh kernel

```
cd /opt/scripts/tools/
git pull
sudo ./update_kernel.sh
```

In particular if you see **`prussdrv_open() failed`** in the logs, this might
indicate that either the wrong PRU config is enabled; check `/boot/uEnv.txt`.

## System locks up

If you have some older debian image, then you might run into this: Some
4.4 linux kernel versions do not have the timers drivers enabled which results
in a kernel panic when BeagleG initializes these.
In order to be able to use the PWM you would need to recompile the kernel with
`CONFIG_OMAP_DM_TIMER=y`.

Alternatively, in order to use BeagleG without the PWM TIMERS support, you
can compile beagleg with:

```
CONFIG_FLAGS=-D_DISABLE_PWM_TIMERS make
```

## Empty am335x_pru_package folder

When compiling, you might encounter the following error:
```
make -e -C src all
make[1]: Entering directory '/home/debian/beagleg/src'
g++ -std=c++11 -Wall -I. -I../am335x_pru_package/pru_sw/app_loader/include -I../hardware/BUMPS -D_XOPEN_SOURCE=500 -mtune=cortex-a8 -march=armv7-a -O3 -DCAPE_NAME='"BUMPS"' -DBEAGLEG_VERSION='"2018-06-16 (commit=51db5c7)"'   -c  machine-control.cc -o machine-control.o
g++ -std=c++11 -Wall -I. -I../am335x_pru_package/pru_sw/app_loader/include -I../hardware/BUMPS -D_XOPEN_SOURCE=500 -mtune=cortex-a8 -march=armv7-a -O3 -DCAPE_NAME='"BUMPS"' -DBEAGLEG_VERSION='"2018-06-16 (commit=51db5c7)"'   -c  motor-operations.cc -o motor-operations.o
g++ -std=c++11 -Wall -I. -I../am335x_pru_package/pru_sw/app_loader/include -I../hardware/BUMPS -D_XOPEN_SOURCE=500 -mtune=cortex-a8 -march=armv7-a -O3 -DCAPE_NAME='"BUMPS"' -DBEAGLEG_VERSION='"2018-06-16 (commit=51db5c7)"'   -c  sim-firmware.cc -o sim-firmware.o
g++ -std=c++11 -Wall -I. -I../am335x_pru_package/pru_sw/app_loader/include -I../hardware/BUMPS -D_XOPEN_SOURCE=500 -mtune=cortex-a8 -march=armv7-a -O3 -DCAPE_NAME='"BUMPS"' -DBEAGLEG_VERSION='"2018-06-16 (commit=51db5c7)"'   -c  pru-motion-queue.cc -o pru-motion-queue.o
make -C ../am335x_pru_package
make[2]: Entering directory '/home/debian/beagleg/am335x_pru_package'
make[2]: *** No targets specified and no makefile found.  Stop.
make[2]: Leaving directory '/home/debian/beagleg/am335x_pru_package'
Makefile:141: recipe for target '../am335x_pru_package/pru_sw/utils/pasm' failed
make[1]: *** [../am335x_pru_package/pru_sw/utils/pasm] Error 2
make[1]: Leaving directory '/home/debian/beagleg/src'
Makefile:32: recipe for target 'all' failed
make: *** [all] Error 2
```

this means that you most probably forgot to clone the repository with the `--recursive` flag.

If that's the case, you can simply run inside the repository folder
`git submodule update --init --recursive` and run `make` again.
