#!/usr/bin/python


import time
import virtGPIO as GPIO

steppr = GPIO.Stepper(0,4, 4, 4096)
# those cheap uln2003 board+stepper from eBay. motor 64steps/rev, plus another gearing of 1:64 (=1:4096)
# 4 wire,   pins 4 5 6 7



while True:
    steppr.setSpeedRPM(2)     # for this tortoise, 2 RPM still has the motor moving right along
    print ("Twitch to the right ...")
    steppr.step(50)
    print ("... and twitch to the left")
    steppr.step(-60)
    print ("Did you miss them?")
    time.sleep(1)
    print ("Now one revolution ...")
    steppr.setSpeedRPM(8)     # up the ante.  8RPM is speeding along
    steppr.step(4096)         # a whole revolution at output shaft
    print ("... done.")
    time.sleep(1)
    print ("And back")
    steppr.step(-4096, False) # similar call, backwards, but no blocking at python end.
    time.sleep(0.4)           # we wait a moment
    print ("... Interim peek at how many steps still to go: %d" % steppr.stepsLeft())
    steppr.waitToFinish()     # but we will wait on the finish anyway!
    print (" ... Done.")
    time.sleep(8)
