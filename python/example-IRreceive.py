#!/usr/bin/python

import time
import virtGPIO as GPIO


IR = GPIO.InfraRedRx(8)

while True:

    irchr = IR.read()
    if irchr:
        print ("IR Remote: %x %d" % (IR.device(), irchr))

    time.sleep(1)
