Small gcode files that will be converted to an HTML page
with images with the `test-create-html.sh` script.

The script can use the [step-speed-same.config](./step-speed-same.config) or
the [step-speed-different.config)(./step-speed-different.config) configuration.
That latter intentionally has vastly different step/mm settings for each
axis to better illustrate if there are problems due to calculations taking
these into account.

To have things comparable easily, try to keep the output in a 100mm x 100mm
rectangle and use a feedrate of 150mm/s (that way straight
acceleration/deceleration in that space is clearly visible).
