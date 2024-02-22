# pico_dcc

## RP2040 PIO DCC Generation

---

This contains MicroPython functions and classes for Model Railway DCC generation as part of simple
Command Station based on an Raspberry Pi Pico.

The MicroPython file contains the original version of PIO assembler program as MicroPython source. For reference the PIOASM version is provided in the C directory.

An RP2040 Peripheral Input Output (PIO) block is used to generate the DCC signal. A 'booster'
is required to enable to track to be powered.  One of the many DC motor H-bridge chips with a PHASE/ENABLE interface capability where the PHASE input pin controls the direction and the ENABLE pin enables the H-Bridge should be suitable. Chips with two symetric IN pins (one for each side of the H-bridge) are less suitable.  These will either require an addition external inverter gate or changes to the driver to control two pins.

## API

---

Class **DCCGen** *(sm_num, DCC_pn, power_pn)*

DCC Output Generator

This singleton class manages the generation of DCC command packets.

Parameters

- *sm_num* PIO statemachine number to be used.

- *DCC_pn* Pin number allocated for DCC output.

- *power_pn* Pin number allocated to the booster for powering the track.

---

Class Method **get_instance** *()*

Get the DCC generator instance.

Returns

- The DCC generator instance

---

**power** *(p=None)*

DCC Power On/Off

Parameters

- *p* 1 for power on, 0 for power off, None for get power status

Returns

- power status

---

**set_fg1** *(address, f_num, state)*

Set Function Group 1

This sets or clears a function in group 1. The forward light is usually function number 0.

See NMRA S-9.2.1 Section 2.3.4

Parameters

- *address* the address of the decoder - may be short or long

- *f_num* function number to set or clear

- *state* 1 for set, 0 for clear

Returns

- True if validation is passed and the packet is scheduled for transmission. False if validation fails.

---

**set_speed** *(address, dir, speed=0)*

Set Speed (including direction)

The packet generated will be for a 128 step speed setting and decoders must be configured for 28/128 speed steps.

See NMRA S-9.2.1 Section 2.3.2.1

Parameters

- *address* the address of the decoder - may be short or long

- *dir* the direction - forward or reverse

- *speed* the speed to be set - range 0 to 127 - default 0

Returns

- True if validation is passed and the packet is scheduled for transmission. False if validation fails.

---

Available constants are:

```py
# Forward direction
FWD = const(1)

# Reverse Direction
REV = const((-1))

# Power Off
OFF = const(0)

# Power On
ON = const(1)
```
