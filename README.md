Experimental motor driver using the PRU capability of the beaglebone black.

The host library motor-interface allows to enqueue steps of up to 8 motors that
are controlled in a coordinated move, with real-time controlled steps at rates
that can go beyond 200kHz. The host operating system is just queueing commands.

TODO
  * Implement G-code interpreter
  * Implement acceleration and jerk.
