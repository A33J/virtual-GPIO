# Demonstrates QUAD-ENCODER
#
# Setup - nothing needed,  (pins 2-5 will be used as outputs)

# Discussion.  QUADencoder is a version of INTcounter.
# The counting interrupt occurs on pin 2 (first counter) or pin3 (second).
# The "quad paired" pin (pin4 for first, pin5 for second) is examined at interrupt time,
# and the counter counts either UP or DOWN depending on the quad pin.
# Commonly used for shaft/wheel encoders where 2 sensors are adjusted to be "out of phase",
# allowing rotation direction to be sensed.

# Using pins as Intcounter, or its quadencoder variant, does not preclude any ordinary use of those pins.
# Here we set them as outputs, and blink then as fast as we easily can.
# On one pair, the main pin "leads", on the other encoder the quad pin leads.
# So one encoder counts up, the other down.  (16bit unsigned)

import time

import virtGPIO as GPIO

counter2 = GPIO.Intcounter(2, GPIO.Intcounter.QUAD)
# Quad encoder oper
GPIO.setup(2,GPIO.OUT)
GPIO.setup(4,GPIO.OUT)

counter3 = GPIO.Intcounter(3, GPIO.Intcounter.QUAD)
GPIO.setup(3,GPIO.OUT)
GPIO.setup(5,GPIO.OUT)
print()
print ("Initial counter readings: "),
print ( counter2.read())
print ("We will now count one UP and one DOWN.")
print ("1000 pulses. (Quad encoder counts at both rising and falling edges) ...")
for k in range(1,2001):
    GPIO.output(4,(k&1))   # quad pin leads on first counter
    GPIO.output(2,(k&1))

    GPIO.output(3,(k&1))   # main pin leads on other
    GPIO.output(5,(k&1))

print ("... Done")
print ("Now read both (16-bit unsigned) counters: "),
print (counter2.read())
