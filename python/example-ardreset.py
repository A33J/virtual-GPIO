#!/usr/bin/python

#  After a few loops, press the arduino reset.
# This script will recognise the arduino reset, and will reboot both ends and start over.

import time

import virtGPIO as GPIO

led = 9   # we have a LED in pin 9 that we will blink

c=0  # simple loop counter
# Next line is the key. If we clear the arduino's reset flag,
# then we can later test if arduino was (accidentally) reset later on.
GPIO.clearResetFlag()
GPIO.setup(led,GPIO.OUT)
print ("If you press Arduino reset, this script will detect that, and take action ...")
while True:
    print ("Loop %d" % c)
    c = (c+1)

    GPIO.output(led,c & 1)   # LED blinking

    if not(c%4):    # every 4th loop we test if arduino was reset (& working again, approximately)
        # If YOU had pressed reset. then our python will spot it now
        if (GPIO.resetFlagIsOn()):
            print ("Arduino is reset! This program going down for restart ...")
            GPIO.restart_program()

    time.sleep(3)
