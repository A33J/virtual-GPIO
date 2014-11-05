#!/usr/bin/python


import time
import virtGPIO as GPIO

led = 9

# Demonstrate classic digital read/write, & analog read, and pwm

# Setup: a LED on pin 9,
#        plus a jumper from pin 9 to pin A2

GPIO.setup(led,GPIO.OUT)
c = 0

while True:

    c = (c+21)
    GPIO.output(led,c & 1)   # LED blink
    print ("Written LED as %d" % (c&1))

    # read it back
    print ("pin%d=%x" % (led, GPIO.input(led)))


    # Analog 2 is tied to LED, so should go from 0000 to 03FF  (0 to 1023)
    print ("anl2 = %04x" % GPIO.analogRead(GPIO.A2))

    time.sleep(5)

    # pin 9 is also a native PWM pin on arduino:
    print ("Changing pwm brightness to %d" % (c%255))
    GPIO.pwmWrite(led, c % 256)

    time.sleep(6)
