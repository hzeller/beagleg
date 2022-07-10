BUMPS hardware definition
=========================

The BUMPS cape was designed together with BeagleG.

![Bumps board][BUMPS-img]

This cape uses the [beaglebone-universal-io] device tree overlay.

## Overlay and I/O configuration

The I/O pins can then be configured using the config-pin utility. Refer to the
[beaglebone-universal-io] for details.

The pin configuration can also be setup from a file such as
[bumps.pins](./bumps.pins).

```
# Configure the I/O pins
/opt/source/bb.org-overlays/tools/beaglebone-universal-io/config-pin -f bumps.pins
```

## Default configuration

An example default configuration [bumps.config](./bumps.config) has been
provided as a starting point.

## Pinout

These are the GPIO bits associated with the motor outputs on the BUMPS board.
The actual physical pins are all over the place on the Beaglebone Black extension headers
P8 and P9, see table below.

Before we can use all pins, we need to install the device-tree overlay, see the
[hardware page](../) for details.

After that, all pins are mapped to be used by beagleg. This is the pinout

|Motor connector        |  1  |  2  |  3  |   4  |  5  |  6  |  7  |    8  |
|----------------------:|:---:|:---:|:---:|:----:|:---:|:---:|:---:|:-----:|
|**Step** `GPIO-0`      |  4  |  5  |   3 |   7  |  2  |  14 | 15  |   20  |
|      ...on BBB Header |P9-18|P9-17|P9-21|P9-42A|P9-22|P9-26|P9-24|P9-41A |
|**Direction** `GPIO-1` | 14  |  15 |  13 |  16  |  12 |  17 | 18  |   19  |
|      ...on BBB Header |P8-16|P8-15|P8-11| P9-15|P8-12|P9-23|P9-14| P9-16 |

Motor enable for all motors is on `GPIO-1`, bit 28, P9-12
(The mapping right now was done because these are consecutive GPIO pins that
can be used, but the mapping to P9-42A (P11-22) and P9-41A (P11-21) should
probably move to an unambiguated pin)

The mapping of motor number to axis happens in the `[Motor-Mapping]` section
of the configuration file, then the axes can be configured with steps etc.

If you build your own cape: note all logic levels are 3.3V (and assume not more
than ~4mA). The RAMPS driver board for instance only works if you power the
5V input with 3.3V, so that the Pololu inputs detect the logic level properly.

This is how the early experimental manual cape interfacing to a RAMPS adapter
looked like.
![Manual Cape][manual-cape]

At the middle/bottom of the test board you see a headpone connector: many of
the early experiments didn't have yet a stepper motor installed, but just
listening to the step-frequency :)

Other pins:
   * Two AUX outputs on GPIO-0 30, 31. This controls the medium current Aux open drain connectors at the bottom left on the [Bumps board][BUMPS].
   * 3 end-switch inputs on GPIO-0 23, 26, 27
   * The PWM outputs are on GPIO-2 2, 3, 4, 5 which are also pins Timer 4, 5, 6, 7. Plan is to
     use the AM335x Timer functionality in their PWM mode (yeah, but I have not looked at the data
     sheet yet to see if this would actually work). These control the two high current
     PWM outputs (screw terminals top right) and the two medium current open drain pwm connectors
     on the Bumps board (connector top left).w
   * Analog inputs AIN0, AIN1, AIN2 will be used for temperature reading.

[BUMPS]: https://github.com/hzeller/bumps
[BUMPS-img]: ../../img/bumps-connect.jpg
[manual-cape]: ../../img/manual-ramps-cape.jpg
[beaglebone-universal-io]: https://github.com/cdsteinkuehler/beaglebone-universal-io
