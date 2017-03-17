# rts
Project code for CSC 460: Real Time Systems

# Building
Some projects can be built from the Arduino IDE, generally those with a `.ino`
file. Just open them using the IDE and press the build button.

Projects in C and Assembly are built using
[Arduino-Makefile](https://github.com/sudar/Arduino-Makefile).  You will need
to install that project and its dependencies. There will be a Makefile in each
directory that needs one; you will need to adjust the `MONITOR_PORT` and
`ARDUINO_DIR` variables, and maybe the path to the `Arduino.mk` file, to suit
your machine. 

Just write `make` in a directory with a `Makefile` to build. `make upload` will
build (if necesssary) and then program the Arduino over USB; `make test` will
do the same, then attach a serial monitor. 

The serial monitor is configured to be `picocom`. If you don't have it, you'll
need to get it through your package manager. On macOS, you can get it from
MacPorts.
