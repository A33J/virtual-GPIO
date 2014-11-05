#!/usr/bin/python


#  Setup:  a led on each of 9 and 10

import time

import virtGPIO as GPIO

# pins are autoconfigured as output by initPin() below

mypwm = GPIO.PWMplus()
c=0
while True:

    c = (c+1)
    print ("both blink at 1 Hz")
    mypwm.setPeriod(1000000)    # period (for BOTH TOGETHER) from 1 uSec to 1,000,000 uSec
    mypwm.initPin(9, 800)       # start pin9 at 800/1023 duty cycle
    mypwm.initPin(10, 100)      # and pin10 at 100/1023
    time.sleep(4)

    print ("change duty cycle on pin10")
    mypwm.changeDuty(10, 600)
    time.sleep(4)

    print ("change frequency to 2 Hz. Preserve duty cycles")
    mypwm.setPeriod(500000)      # 200,000 uSec period = 5 Hz
    time.sleep(4)

    print ("stop pin10")
    mypwm.releasePin(10)
    time.sleep(4)

    print ("pin9 1uSec pulse every 1mSec")
    print ("you wont see the pulses, but you might see a pwm glow!!")
    mypwm.changeDuty(9,1)        # 1 in 1023
    mypwm.setPeriod(1000)        # 1000 uSec period
    time.sleep(4)

    print ("stop pin9")
    mypwm.releasePin(9)
    time.sleep(4)
