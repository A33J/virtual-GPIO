#!/usr/bin/python
# -*- coding: utf-8 -*-

import time

import virtGPIO as GPIO

led = 6   # put a LED on pin 6

# Note object GPIO.AVR for MCU register access is pre-defined for us in virtGPIO.py
# and pre-defined register names are in virtGPIO.py

GPIO.setup(led,GPIO.OUT)

c=0

while True:
    c += 1
    GPIO.output(led,c & 1)   # LED blink
    print ("pin 6 input by raw atmega238 access %d " % ((GPIO.AVR.read8(GPIO.AVR.PIND) & 0x40) > 0))
    # Guided by 328 MCU manual, we read 8-bit register "PIND" and select bit 6
    # See  <http://www.atmel.com/Images/doc8161.pdf> page 93
    # and <http://arduino.cc/en/Hacking/PinMapping168>

    time.sleep(2)


# As another example (example only),
# the lines below show the virtGPIO.py internal implementation of analogHiSpeed(),
# which changes the clock prescaler of ADC convertor in the 328.
# Refer 328 MCU manual, pages 253 and 264.
# If you are out of your depth, you don't need any of the raw GPIO.AVR functions!
"""
def analogHiSpeed():
        # This moves ADC clock from divider 128 to divider 16
        # Not very useful for single analog reads, but useful for analogReadAll()
        AVR.bitClear(AVR.ADCSRA, AVR.ADPS0);
        AVR.bitClear(AVR.ADCSRA, AVR.ADPS1);
        AVR.bitSet(AVR.ADCSRA, AVR.ADPS2);
"""
