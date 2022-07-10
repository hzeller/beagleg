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
indicate that either the wrong kernel is enabled (if you type `uname -r`, the
returned name needs to contain `bone` (like `4.19.232-bone-rt-r75`)) or if the
`uboot_overlay_pru` setting in `/boot/uEnv.txt` is not properly enabled.
(see 'Prepare the Environment' above).

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
