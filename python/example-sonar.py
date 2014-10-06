#!/usr/bin/python

# setup:  sr04 4-pin ultrasonic device, trigger pin4, echo pin6, supply = 5V
# (3.3V not adequate for SR04)

import time
import virtGPIO as GPIO
sonar1 = GPIO.Sonar(4,6)

while True:
    print "sonar: %d" % sonar1.ping()
    time.sleep(1)
