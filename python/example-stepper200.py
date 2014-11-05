#!/usr/bin/python


import time
import virtGPIO as GPIO


steppr = GPIO.Stepper(0,4, 4, 200)
# 200 steps/rev type
# 4 wire,   pins 4 5 6 7

# (This example is good for watching lights twinkle on the stepper outputs)


while True:
    steppr.setSpeedRPM(1)
    print ("Crawl to the right ...")
    steppr.step(25)
    print ("... and crawl to the left")
    steppr.step(-30)
    time.sleep(1)
    print ("Now one revolution faster ...")
    steppr.setSpeedRPM(20)     # up the ante.
    steppr.step(200)         # a whole revolution at output shaft
    print ("... done.")
    time.sleep(8)
