# MPWM

An Arduino compatible libarary for SAMD51 devices that provides an abstraction
to access MPWM functionality. MPWM mode allows for finer control over the
output frequency of the PWM wave being generated, while giving up PWM output
on some pins. It is only available on a small subset of pins on the chip.

This library builds on top of the Arduino Core by Adafruit for the Metro M4 &
Grand Central M4.

> NOTE: the library will default to `analogWrite()` when an invalid pin is used

> NOTE: if the PWM pin provided is attached to a `TC` module but is can not be
mapped to the appropriate `W[x]` output from the module, then the library will
still change the state of the `TC` modules but there will be no output unless
the correct pin is set to the correct mux via `pinPeripheral()` and the pin mode
is set to `OUTPUT`. This behavior may change; so use valid pins as listed in
the datasheet and the table in `variants.c` in the Arduino Core released by
Adafruit