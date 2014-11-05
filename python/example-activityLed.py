# virtGPIO uses one LED as an "activity" indicator.
# By default, it starts at LED13, the usual inbuilt Led.
# When a serial command arrives at arduino, the activity led goes ON.
# When the command has finished processing, the Led goes OFF.
# Most commands process in way below a millisecond,
# but a "blocking" function [ie, pulseIn()] leaves the Led on for longer, until finished.

# Observing that the activity Led is blinking shows the arduino is taking commands.
# The activity Led may be easily assigned to another pin, or switched off (pin = 0).
# Note pin13's NATIVE use on arduino is SPI port, but we still need to shift the activity pin if needing SPI.

# Setup:  put a LED on pin 9.

import time
import virtGPIO as GPIO

led = 9

GPIO.setup(led,GPIO.OUT)
c=0

print ("Activity LED = 13")

while True:
    c+=1
    GPIO.output(led,c & 1)   # LED Blink


    if (c==100):   # after 10 secs
        print ("Activity Led changed to 0 (=off)")
        GPIO.setActivityLed(0)

    if (c==200):    # after 20 secs
        print ("Activity Led changed to %d" % led)
        print ("Note blink function has been killed, now just activity indicator.")
        GPIO.setActivityLed(led)

    if (c==300):   # after 30 secs
        time.sleep(1)
        print ("Observe (on LED) slightly longer blocking function (= fail to start pulse?)")
        print ("pulseIn returns (error?) code %x" % GPIO.pulseIn(4, GPIO.HIGH, 200, 10000))
        # it aint gonna happen: that pin is doing nothing.
        print ("Fin")
        break;

    time.sleep(0.1)
