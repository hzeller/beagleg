BeagleG
-------

Experimental motor controller using the PRU capability of the Beaglebone Black.

The host library motor-interface allows to enqueue steps of up to 8 motors that
are controlled in a coordinated move, with real-time controlled steps at rates
that can go beyond 200kHz. The host operating system is just queueing commands.

To test things properly, there is as well a G-code interpreter.

TODO
  * Implement acceleration and jerk.
