#!/usr/bin/python

# Demonstrates:  How to configure and use up to 4 SPI devices.

# Setup:  Jumper from pin 11 (MOSI) to pin 12 (MISO)
#         That sends all SPI serial output (MOSI) back into the input (MISO)
#         (We will just pretend we have real SPI devices connected to our bus.)


import time

import virtGPIO as GPIO

print "Read comments in source file"

GPIO.setActivityLed(0)   # optional
# The SpiDev() will fail if all 4 needed pins are not free (11, 12, 13, plus CE pin)
# Pin13 is the default activityLed. Pin13 is also the SPI clock pin.
# However as "true" owner of pin13, SPI will knock off the activityLed if that is the only problem.
# See example-activityLed.py and example-diagnostic.py

SPI = GPIO.SpiDev(10)   # Initialise one SPI channel using pin 10 as its CE pin
                        # "CE" is variously known as SS (slave select), CS (chip select) or CE (chip enable)
                        # and also often written with a "*" or a "N" (eg SS* or CSN) denoting -ve logic: LOW active

SPI2 = GPIO.SpiDev(8)   # So let's prepare for another SPI device using pin 8 as CE

while True:
    print "SPI xfer on CE=10", SPI.xfer2([55,6,6,6,6,6,6,6,6,7,6,6,6,6,6,6,6,6,6,0,0,8])

    print "SPI xfer on CE=8", SPI2.xfer2([74])

    time.sleep(8)

    # So?  Expect to simply see those (byte) number lists as above returned from the arduino.
    # (Returned character count) = (sent character count). Up to 33 characters handled (enough for a RF24 radio)
    # The whole packet went from rPi to Arduino, out the MOSI pin, back into the MISO, & thence back to us.

    # For a REAL SPI scenario (real devices connected), each separate device (responding to its own assigned CE pin)
    #    accepts the MOSI character stream and feeds its own results back to MISO.


    # Refer to example-NRF24-tx.py (&rx) for something more exciting.
