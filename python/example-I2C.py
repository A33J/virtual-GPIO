#!/usr/bin/python

# Uses the PCF8591 cheap module from eBay.   I2C address is 0x48

import time

import virtGPIO as GPIO

i2c = GPIO.I2C()
i2c.detect()    # Diagnostic: displays the addresses of any I2C devices connected


while True:
    print "I2C read ",
    print i2c.writeRead(0x48, 0x03, 2)
    # This is sequence for reading the POT an Ain3
    # address 0x48.  Write 0x03 (means Ain3).  Read 2 samples.
    # Twiddle the pot and watch the output change here
    time.sleep(3)
