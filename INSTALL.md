# INSTALLATION

## Get one of the latest linux Debian images

Download one of the latest debian images provided by the following [**list**](https://beagleboard.org/latest-images). This installation guide refers to [*bone-debian-9.3-iot-armhf-2018-03-05-4gb.img.xz*](http://debian.beagleboard.org/images/bone-debian-9.3-iot-armhf-2018-03-05-4gb.img.xz).

If necessary, check the integrity of the downloaded image by matching its sha256sum hash and the one provided on the website (on a linux terminal,
just run `sha256sum bone-debian-9.3-iot-armhf-2018-03-05-4gb.img.xz`).

Always use the most minimal image you can find, e.g. no graphical user
interface etc.

## Flash the SD card
**WARNING**: **Be careful when selecting the device to be flashed, as you may end up losing data inside it.**

Connect an empty SD card to your PC and extract and copy the image over your sd card.

Based on the platform this can be done via the following methods:

#### **Linux**
On a terminal, save the SD card device path by inspecting `lsblk`. The SD card should be under a path similar to `/dev/mmcblkN`. Run on the terminal, `xzcat bone-debian-9.3-iot-armhf-2018-03-05-4gb.img.xz | sudo dd of=/dev/mmcblkN`.
Before extracting the SD card remember to run `sync` in order to flush any remaining buffered I/O operation.

#### **Linux / Mac OS / Windows**
Using the graphical tool **https://etcher.io/**. Select the downloaded image , the target device and then click on **Flash**.


## Boot

1. Connect the beaglebone to your local LAN with DHCP enabled or via USB.
2. Insert the SD card on the beaglebone slot.
3. Turn on the beaglebone.

### Connect via SSH

If you connected the beaglebone via USB, a virtual Ethernet interface should appear on your PC.


#### **Linux / Mac OS**

Open a terminal and run: `ssh debian@beaglebone.local`
the default password is usually `temppwd`.

#### **Windows**

Download the ssh client from https://www.putty.org/.
use as hostname `debian@beaglebone.local` and connect using the usual `temppwd` password.


## Prepare the environment

If the following one-liner:
```
if lsmod | grep "uio_pruss" &> /dev/null ; then echo "The kernel is BeagleG ready"; else echo "uio_pruss module not present"; fi`
```

will return `The kernel is BeagleG-ready`, it means that you can skip this section, otherwise you will need some additional steps.

Beagleg as it is now, requires a kernel module called `uio_pruss` (see [this](https://elinux.org/Ti_AM33XX_PRUSSv2#Communication)).

To enable it, you will need to edit your `/boot/uEnv.txt`
and change:

```
###PRUSS OPTIONS
###pru_rproc (4.4.x-ti kernel)
#uboot_overlay_pru=/lib/firmware/AM335X-PRU-RPROC-4-4-TI-00A0.dtbo
###pru_uio (4.4.x-ti, 4.14.x-ti & mainline/bone kernel)
#uboot_overlay_pru=/lib/firmware/AM335X-PRU-UIO-00A0.dtbo
###

```

uncommenting (removing the `#` in front) the line `uboot_overlay_pru=/lib/firmware/AM335X-PRU-UIO-00A0.dtbo`.

Before rebooting, you will also need to have an updated version of your kernel as you may suffer a bug that will not correctly load the uio_pruss module and device tree overlay.

To do so, run `sudo apt-get update; sudo apt-get upgrade` and on completion,
execute a reboot.

## Install BeagleG


Let's start from fetching the BeagleG repository with:

```
git clone --recursive https://github.com/hzeller/beagleg.git
```

change directory into the repository and run `make`.

The resulting `machine-control` binary will be in the toplevel directory. You
can `sudo make install` it, or run it right there.

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

## uio_pruss not loaded

In some older kernels, you might need to manually

```
sudo modprobe uio_pruss
```

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
