#!/usr/bin/python


#  Setup:  a led on 9
#          alternatively (better!), a piezo buzzer on 9

import time
led = 9
import virtGPIO as GPIO

mypwm = GPIO.PWMplus()

while True:

    print ("8 Hz for LED")
    mypwm.tone(led, 8)      # on pin 9, a 8 Hz tone (forever ...)
    time.sleep(2)        # long enough?
    mypwm.noTone(led)      # ... until we stop
    # Note: all pwmplus functions are non-blocking. Arduino will carry on until told stop.

    print ("800 Hz for Piezo Buzzer")
    mypwm.tone(led,800)      # now 800 Hz
    time.sleep(2)
    mypwm.noTone(led)

    # We wont try 80,000 Hz, you can't hear it or see it, but device should do it!

    time.sleep(4)
