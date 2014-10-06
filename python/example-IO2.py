#!/usr/bin/python

# This is the same script as example-IO.py
# except that by importing differently we can eliminate the "GPIO." prefix on the defined I/O words like OUTPUT

# We COULD of course use "from virtGPIO import *"
# to allow EVERYTHING from virtGPIO.py to be use without any "GPIO." prefix.
# Namespace purgatory. You are on your own.

import time
import virtGPIO as GPIO
from vGPIOconstants import *             # <<<<<<< change

led = 9

# Demonstrate classic digital read/write, & analog read, and pwm

# Setup: a LED on pin 9,
#        plus a jumper from pin 9 to pin A2

GPIO.setup(led,OUTPUT)                 # <<<<< change
c = 0

while True:

    c = (c+21)
    GPIO.output(led,c & 1)   # LED blink
    print "Written LED as %d" % (c&1)
    ## GPIO.digitalWrite(led, c&1)   would work also

    # read it back - rPi style & arduino style
    print "pin%d=%x" % (led, GPIO.digitalRead(led))
    print "pin%d=%x" % (led, GPIO.input(led))

    # Analog 2 is tied to LED, so should go from 0000 to 03FF  (0 to 1023)
    print "anl2 = %04x" % GPIO.analogRead(A2)                                 # <<<< change

    time.sleep(1)

    # pin 9 is also a native PWM pin on arduino:
    print "Changing pwm brightness to %d" % (c%255)
    GPIO.pwmWrite(led, c % 256)

    time.sleep(1)
