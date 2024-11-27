# Arduino/ATmega based programmer for SST39SF010

## Part 1: the programmer

The programmer is based around an ATmega. I used an ATmega88 in DIP-28, but theoretically any ATmega should do. You'll need two 74595 shift registers, two 100nF capacitors, a 10kOhm resistor, and a SPDT switch. I also recommend a ZIF socket for the actual flash chip, to prevent trashing the legs. If you're using an Arduino you can omit the capacitors, resistor and switch, because those are used for controlling when the chip resets.

I don't have any PCB layouts available because my version was done on stripboard, but I'm sure you'll be able to figure out a layout yourself.

Flash prog_v3 to the chip (if you're using a standalone AVR you'll want to install [Minicore](https://github.com/MCUdude/MiniCore) and follow the guides there).

## Part 2: the host program

The host program is only tested on Linux, because I don't have a Windows machine, and probably won't work out of the box on Windows due to serial port nonsense. The command syntax is `flashpoint [-p PORT] {write -f FILE [-o OFFSET] | read -f FILE [-o OFFSET] [-l LENGTH]}`. PORT defaults to /dev/ttyUSB0, OFFSET (the offset into the flash) defaults to 0, and LENGTH defaults to 128kB, the size of a 39SF010.

Adapting the programmer and host program to larger flashes shouldn't be too much work for a good programmer.
