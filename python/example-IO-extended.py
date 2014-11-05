#!/usr/bin/python


# Demonstration: extended digital and analog I/O functions:
#   digitalReadAll()     VccRead()     precise DigitalRead()     pulseIn()
#   pulseOut()           analogReadAll()      analogHiSpeed()

# setup: put a LED on pin 7.

import time

import virtGPIO as GPIO

led = 7

GPIO.setup(led,GPIO.OUT)

print ("VCC = %f Volts" % GPIO.VccRead())
c=0
while True:
    c += 1
    GPIO.output(led,c & 1)   # LED
    print ("Read (as list of int16) all analogs: %s" % GPIO.analogReadAll())
    print ("Read (as int16) all digitals 0 to 13 %s" % bin(GPIO.digitalReadAll()))
    print ("Precision digital read pin 7 (LED?): %d" % GPIO.digitalPreciseRead(7))
    print ("Precision digital read pin A0: %d" % GPIO.digitalPreciseRead(GPIO.A0))
    print ("(precision read is from analog comparator against internal 1.1V reference.)")
    if c==10:
        GPIO.analogHiSpeed()
        print ("Increasing analog read clockspeed: AnalogReadAll() is a lot faster from now.")
    print ("")
    time.sleep(1)
    print ("pulseIn, a 'blocking' call: in this routine, will timeout by lack of a pulse:")
    print (GPIO.pulseIn(led, GPIO.HIGH, 500, 1000))   # allow 500 uSEc to start, then 1000 uSec max pulselength.
    # NOTE: those high value returns correspond to "error codes" set in VirtGPIO.ino.
    # Examine Pulse_in function there, and the "case 'I':" code.
    # By testing these codes you can see the precise reason your pulsein failed.
    print("65530 timeout long pulse, 65533 not idle,  65534 timeout start")
    print ("")
    time.sleep(1)
    if not (c&1):  # ie, is this an OFF time for our led?
        time.sleep(1)
        print ("pulseOut. 250 uSec. Can you catch it by eye??")
        GPIO.pulseOut(led, GPIO.HIGH, 250)


    time.sleep(1)
