#!/usr/bin/python


# Setup:   A TTL serial connection 2400 baud to either pin A4 or pin 6 (different output each)
#          A terminal program (eg hyperterminal or cutecom) is ideal. 2400N.

# Too difficult?  Put a LED each on pins A3 and 6. You should see them twinkle (quickly) with data.
# Best way on this occasion is led&resistor from pin to +5V rather than pin to GND.
# (You COULD also recompile VirtGPIO at arduino for 75 baud [file BBSerial.h] to see slow data twinkles.)


import time

import virtGPIO as GPIO

COM1 = GPIO.SerialTx(0, GPIO.A3)
COM2 = GPIO.SerialTx(3, 6)

# Port 0, custom-buffered, delayed transmit, (larger buffer) port:
COM1.Print("\nCOM1 (port0, pin A3): Hello. I am ")  # one parameter per call. This is a string parameter.
COM1.Print(17.4)   # note float or integer parameters are automatically converted and sent as character strings.
COM1.Print(" years young!\n")  # Depending on your final receiver of characters, linefeed may be acknowledged.
time.sleep(1)

# Now, being cautious of limited buffer capacity in port 3.
# (The limit is an arduino internal limit.)
COM2.Print("\nCOM2 (port3 pin6): ")
time.sleep(0.01)
COM2.Print("Don't believe the other guy.")

print ("Done. We sent 2400 baud text to buffered COM port 0 on pin A3, and 2400 direct COM port 3 on pin 6.")
print ("Study the python code.")
