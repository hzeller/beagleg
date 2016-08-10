VGEN5 hardware definition
==========================

This cape uses the [beaglebone-universal-io] device tree overlay and support
scripts to allow more flexiblity in the I/O mapping without editing dts files
or rebuilding the kernel.

This overlay file allows all the pins needed by the VGEN5 cape to be used.

  * cape-universal  Exports all pins not used by HDMIN and eMMC (including audio)

## Build notes

To support the VGEN5 cape, BeagleG must be built using the correct hardware target:

```
make BEAGLEG_HARDWARE_TARGET=VGEN5
```

## Overlay and I/O configuration

The config-pin utility can be used to load the overlay.

```
# Load the cape-universal overlay
config-pin overlay cape-universal
```

At this point, the various devices are loaded and all the gpio have been
exported. All the pins currently default to gpio inpus, with pull up or
pull down resistors set the same as when the AM335x comes out of reset.

The I/O pins can then be configured using the config-pin utility. Refer to the
[beaglebone-universal-io] for details.

The pin configuration can also be setup from a file such as
[vgen5.pins](./hardware/VGEN5/vgen5.pins).

```
# Configure the I/O pins
config-pin -f vgen5.pins
```

## Machine control

The VGEN5 cape can then control a machine with G-Code using the `machine-control`
binary. The [vgen5.config](./hardware/VGEN5/vgen5.config) config file matches the
pin configuration setup above.

```
sudo ./machine-control -c vgen5.config [options] [<gcode-filename>]
```

## Machine Configuration

The [vgen5.config](./hardware/VGEN5/vgen5.config) config file should be edited
to match your machine configuration.

The [vgen5.pins](./hardware/VGEN5/vgen5.pins) pin configuration may also need
to be edited if any of the input or output mappings are changed.

### Default Motor mapping

The VGEN5 cape supports 5 Pololu style drivers. Each driver has a **STEP** and
**DIR** signal. All the drivers use a common **ENABLE** signal.

The drivers are all configured for 1/16 microstep operation.

|Motors                |MOTOR_1  |MOTOR_2   |MOTOR_3  |MOTOR_4   |MOTOR_5  |MOTOR_ENABLE|
|---------------------:|:-------:|:--------:|:-------:|:--------:|:-------:|:----------:|
|Mapped Axis           |**X**    |**Y**     |**Z**    |          |         |            |
|**STEP** GPIO         |`GPIO_20`|`GPIO_115`|`GPIO_30`|`GPIO_49` |`GPIO_31`|            |
|...on BBB Header      |P9-41    |P9-27     |P9-11    |P9-23     |P9-13    |            |
|**DIR** GPIO          |`GPIO_7` |`GPIO_112`|`GPIO_60`|`GPIO_117`|`GPIO_48`|            |
|...on BBB Header      |P9-42    |P9-30     |P9-12    |P9-25     |P9-15    |            |
|**ENABLE** GPIO       |         |          |         |          |         |`GPIO_51`   |
|...on BBB Header      |         |          |         |          |         |P9-16       |

### Default Output mapping

There are 6 outputs available, 4 of which can be used as PWM outputs.

|Outputs         |AUX_1    |AUX_2    |AUX_3    |AUX_4    |AUX_5    |AUX_16   |
|---------------:|:-------:|:-------:|:-------:|:-------:|:-------:|:-------:|
|PWM output      |PWM_1    |PWM_2    |PWM_3    |PWM_4    |         |         |
|TIMER           |`TIMER4` |`TIMER7` |`TIMER5  |`TIMER6  |         |         |
|Mapped signal   |vacuum   |mist     |flood    |fan      |         |led      |
|GPIO output     |`GPIO_66`|`GPIO_67`|`GPIO_69`|`GPIO_68`|`GPIO_50`|`GPIO_61`|
|...on BBB Header|P8-7     |P8-8     |P8-9     |P8-10    |P9-14    |P8-26    |

### Default Input mapping

There are 9 inputs available, 6 are intended to be used for endstops.

|Inputs          |IN_1     |IN_2     |IN_3     |IN_4     |IN_5     |IN_6     |IN_7     |IN_8     |IN_9     |
|---------------:|:-------:|:-------:|:-------:|:-------:|:-------:|:-------:|:-------:|:-------:|:-------:|
|Mapped signal   |min_x    |min_y    |max_z    |min_z    |         |         |e-stop   |pause    |start    |
|GPIO input      |`GPIO_47`|`GPIO_46`|`GPIO_45`|`GPIO_44`|`GPIO_23`|`GPIO_26`|`GPIO_27`|`GPIO_65`|`GPIO_22`|
|...on BBB Header|P8-15    |P8-16    |P8-11    |P8-12    |P8-13    |P8-14    |P8-17    |P8-18    |P8-19    |

### Other 

These signals are all routed to test points. They are currently not used.

|Extra           |SPI0_CS0|SPI0_D1 |SPI0_D0 |SPI0_SCLK|UART1_TXD|UART1_RXD|SPI1_CS0  |SPI1_D0   |SPI1_SCLK |
|---------------:|:------:|:------:|:------:|:-------:|:-------:|:-------:|:--------:|:--------:|:--------:|
|Unmapped GPIO   |`GPIO_5`|`GPIO_4`|`GPIO_3`|`GPIO_2` |`GPIO_15`|`GPIO_14`|`GPIO_113`|`GPIO_111`|`GPIO_110`|
|...on BBB Header|P9-17   |P9-18   |P9-21   |P9-22    |P9-24    |P9-26    |P9-28     |P9-29     |P9-31     |

The analog inputs are also routed to a header.

In addition, the VGEN5 cape has these features:

  * An I2C Real-Time Clock with battery backup.
  * A 4 port USB 2.0 Hub with 2 external connectors, 2 internal connectors.
  * Support for a Pololu Simple Motor Controller (24V spindle).
  * A 5V switching power supply (only a 24V power supply is required).
  
[beaglebone-universal-io]: https://github.com/cdsteinkuehler/beaglebone-universal-io