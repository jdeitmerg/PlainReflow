# PlainReflow

PlainReflow is a minimalist controller for a toaster oven that's misused
for reflow soldering. The electronics are meant to be cheap (10$ range if
ordered from China) and functional and not fancy. Similarily, the Arudino
code is kept to a minimum, no bells and whistles included.

## Status

* Electronics: Mostly build but untested
* Firmware: Basics only
* Control design: Not even started
* Documentation: Good enough for now

All in all, we're miles from a working prototype :/

## Electronics

![Schematic](https://user-images.githubusercontent.com/7459936/32291142-7d529ce0-bf3c-11e7-81d5-0cea35ba904e.png)

The electronics consist of:

* an Arduino Nano running the control logic
* a 4 channel relay module for switching the 230V heating rods of the oven
* a power supply for the Arduino and relay module. Anything between 8V and
  15V DC should work
* a PT100 thermistor for measuring the temperature. Has to stand at least
  300°C
* a few additional components

### Resistance measurement

To determine the current temperature, we have to measure the resistance
of the PT100 thermistor. This is typically done by feeding a constant
reference current (<1mA) through the thermistor and measuring the
voltage across it (100mV range). An Arduino has the maximum of its analog
input in the 1V to 5V range. In order to use the whole analog resolution,
the thermistor voltage has to be amplified. Way too complicated.

We'll use the bruteforce solution: Apply a big reference current (close to
5mA) and use the smallest voltage reference of the Arduino (1.1V). There are
two problems with using such a large current: The thermistor will heat
itself up, distorting the measurement. Secondly, a current bigger than the
maximum specified in the datasheet may destroy the thermistor.

Both problems can be solved by applying the reference current only for
very short periods and not very often. To keep it simple, there is a small
N-channel MOSFET parallel to the PT100. It is turned on most of the time,
letting 5mA flow through R1. During the temperature measurement, it is
turned off and the current flows through the thermistor.
