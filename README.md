# pico_dcc
 RP2040 PIO DCC Generation

---

This contains MicroPython functions and classes for Model Railway DCC generation as part of simple
Command Station based on an Raspberry Pi Pico.

An RP2040 Peripheral Input Output (PIO) block is used to generate the DCC signal. A 'booster'
is required to enable to track to be powered.  One of the many DC motor H-bridge chips (e.g. TI DRV8874) should be suitable.

The MicroPython file contains the original version of PIO assembler program as MicroPython source. For reference the PIOASM version is provided in the C directory.