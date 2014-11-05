# Demonstration: using I2C bus in linux "SMBus" syntax

# Hardware: cheap PCF8591 analog module from eBay. 4 inputs, 1 output. 3 inputs have jumperable inbuilt sensors.

# Setup: Arduino I2C pins (A4 & A5) to module corresponding pins.
#        Add pullup (to 5V) 10k on each of those pins.
#        module power = GND and 5V from arduino
#        All 3 sensor jumpers left in.
#        Add F-F jumper wire from Aout to Ain2 (the one without inbuilt sensor)

# re Ain2: if left floating, all bets are off. It means nothing.

# A/D in the PCF8591 is @ I2C address 0x48 (fixed by module design)

from time import sleep
import virtGPIO as GPIO

# Analog channel numbering:
AIN0 = 0
AIN1 = 1
AIN2 = 2
AIN3 = 3
LDR = AIN0   # the 3 inbuilt sensors
TEMP = AIN1
POT = AIN3

# How do we know how to program the PCF8591? (PCF8591 is the number of the IC on the module doing all the work.)
# Primary info = "PCF8591 Data Sheet"  Easy to find a PDF version using Google.
# On our module, 3 of the 4 inputs have built-in functions. But pull out their jumpers, and they become normal spare inputs:
#     Ain0   Jumper P5   Light dependent resistor
#     Ain1   Jumper P4   Temp dependent resistor
#     Ain3   Jumper P6   Potentiometer (knob)

# I2C address for PCF8591 module
ADDR_8591 = 0x48                  # I2C address to find module on SMBus system. (All pins A0 A1 A2 on the chip are fixed grounded -> address 0x48)
OUT_ENABLED    = 0b01000000       # This "bit" in PCF8591 control register turns on Dig-to-Anl output. We want Aout working.
AUTO_INCREMENT = 0b00000100       # This bit sets AIN channel# (0-3) to step along automatically at each read. We will use this mode.


# create I2C smbus object ("bus") to talk over I2C wires:

bus = GPIO.I2C()

print("Read the A/D")
print("Ctrl C to stop")
sleep(1)

print ("\033[2J")
# "Ansi Sequence" to clear screen. Look it up on Wikipedia,
#     or http://www.isthe.com/chongo/tech/comp/ansi_escapes.html
print ("\033[2;0HThis script makes Aout track the POTENTIOMETER (knob) reading.")
print (" You could (very carefully) read the Aout pin with a voltmeter.")
print (" A non-metallic screwdriver is best for the POT: it can get shorted!")
print ("If you connect Aout back into (spare) Ain2 by wire,\n then Ain2 will also track the KNOB.")

while(True): # do forever
    print ("\033[7;0H\033[32m")    # Re-position cursor to known spot on screen (Line7, Col0). (& colour the font)
    # The reading sequence takes some getting head around!  We need to TRIGGER A CAPTURE on the chip.
    # The READ command does that triggering. The capture process take a (small) time to process, and we don't wait around for it to finish.
    # But that read call actually returns the PREVIOUSLY requested capture, now waiting to be collected.
    # So we need to read a second time to get what we really want !!!!! We will use auto-increment mode to streamline the job.
    bus.write_byte(ADDR_8591, OUT_ENABLED | AUTO_INCREMENT | AIN0 )   # Control byte to prepare channel 0 for reading. (& enable Aout=on)
    reading_d = bus.read_byte(ADDR_8591)          # Dummy read - return prev capture, start new capture (Ain0), and incr channel# to Ain1
    # throw away the dummy reading
    # Meantime, capturing on Ain0 is done very quickly ...
    reading0 = 255 - bus.read_byte(ADDR_8591)    # Read (now ready) Ain0 capture, start new capture (on Ain1), and incr channel# to Ain2
    # read value is 0 - 255. Actually 255 = dark, and 0 = very bright. Non-intuitive to humans - turn this upside down so 0 means dark.
    print ("Ain0:  Light:   " + str(reading0) + "  (0-255. How could we 'calibrate' that?)    ")

    reading1 = 255 - bus.read_byte(ADDR_8591)    # Read the Ain1 capture, start new capture (on Ain2), and incr channel# counter to Ain3
    print ("Ain1:  Temp:    " + str(reading1) + "  (0-255. But what degrees does that mean??)   ")

    reading2 = bus.read_byte(ADDR_8591)          # ... etc
    print ("Ain2:           " + str(reading2)  + "  (i.e., Volts=" + str(round(reading2 * 5.0 // 255, 2))  + ")    ")
    # readings 0 to 255 scale to the voltage range 0v to 5.0v (or whatever voltage the module is running at)

    knobReading = 255 - bus.read_byte(ADDR_8591)
    print ("Ain3:  Knob:    " + str(knobReading) + "  (hex: %X  bin: " % knobReading + bin(knobReading) + ")  ")


    # Simply push the POTENTIOMETER KNOB reading (0-255) back out the Analog Output pin (0-255)
    Aout = knobReading
    bus.write_byte_data(ADDR_8591, OUT_ENABLED, Aout);
    print ("\n\033[33mAout:  Output:  " + str(Aout) + "  (0-255 means 0.0V to 5.0V)")
    print ("\n\033[34mCTRL-C to exit ...")

    sleep(0.5)


# A sensible refinement would be to use the GPIO.readVcc() function to get exact arduino voltage,
# rather than the hardcoded "5.0" used above
