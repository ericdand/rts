# rts
Project code for CSC 460: Real Time Systems

# Building
Some projects can be built from the Arduino IDE, generally those with a `.ino`
file. Just open them using the IDE and press the build button.

Some projects are built from outside the Arduino IDE. These use the
[Arduino-Makefile](https://github.com/sudar/Arduino-Makefile) project to do
so. You will need to install that project and its dependencies. There will be
a Makefile in each directory that needs one; you will need to adjust the 
`MONITOR_PORT` and `ARDUINO_DIR` variables, and maybe the path to the 
`Arduino.mk` file, to suit your machine. Just write `make` to build.

