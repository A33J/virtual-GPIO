#!/usr/bin/python
# -*- coding: utf-8 -*-

# Demonstrate:
# setting pin5 as MCU-based counter (no interrupts, no arduino counting code!)
# Put a LED on pin 5 to see


import time

import virtGPIO as GPIO

led = 5
GPIO.setup(led,GPIO.OUT)

counter5 = GPIO.HWcounter()
# set up pin 5 as a hardware counter (rising edge). It can STILL do any other GPIO job!

c=0

while True:
    print (c)

    c = (c+1)
    GPIO.output(led,c & 1)   # LED blink on pin 5
    print ("Dig read 5 = %d" % GPIO.input(led))   # read pin 5 straight back to confirm

    # But pin 5 is ALSO doing a counter job. Read the current count
    print ("HW ctr: %x" % counter5.read()) #

    if not (c%100):  # every 100 passes
        counter5.zero()

    time.sleep(0.5)
