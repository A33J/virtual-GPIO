#!/usr/bin/python


import time
import virtGPIO as GPIO

myServo = GPIO.Servo(7)   # servo on pin 7
c=0L

while True:
    print c
    c += 5
    c = c & 0x7f  # limit to 127

    myServo.write(c)   # sweeps slowly 0 - 127 degrees then returns

    time.sleep(1)

    myServo.stop()    # detaches (You DONT have to do it this way)

    time.sleep(0.2)
