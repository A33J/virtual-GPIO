#!/usr/bin/python

# Uses the PCF8591 cheap module from eBay.   I2C address is 0x48

import time

import virtGPIO as GPIO

i2c = GPIO.I2C()
i2c.detect()    # Diagnostic: displays the addresses of any I2C devices connected

print ("2 readings each time.")

while True:
    print ("I2C read "),
    r = i2c.writeRead(0x48, 0x03, 2)
    print (r)
    if r[0] == 0xf3:
        print ("The first byte is a success/error code. 243 = fail.")
    # This is sequence for reading the POT an Ain3
    # address 0x48.  Write 0x03 (means Ain3).  Read 2 samples.
    # Twiddle the pot and watch the output change here
    time.sleep(3)
