#!/usr/bin/python

# Setup: a Led on pin 2
# A pin assigned as interrupt counter may still have any other input or output job to do.
# Each appropriate transition (rising or falling or both) accumulates to the 16-bitcounter
# Interrupt driven counters available pins 2 and 3, but we show just pin 2 here.

import time

import virtGPIO as GPIO

led = 2


c=0
counter2 = GPIO.Intcounter(2, GPIO.Intcounter.FALLING)
GPIO.setup(led,GPIO.OUT)

while True:
    c += 1

    GPIO.output(led,c & 1)   # LED blink
    print ("Pin 2 = %d" % (c&1))

    print ("Read counters (pins 2 and 3 time-matched pair) %s" % counter2.read())
    # note if we had another counter on pin3, counter3.read() would give same result.

    if not (c%20):
        print ("Read & zero %s" % counter2.read(True))   # optional parameter to command a clear on the count

    time.sleep(0.5)
