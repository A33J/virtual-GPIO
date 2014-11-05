#!/usr/bin/python

# Demonstrates: various technical tips and tricks.

# Setup: If you have an I2C device connected to pins A4/A5 (with 10k pullups to 5V on each), address will be printed
#         Put a LED on pin 9


import time

import virtGPIO as GPIO


print ("Arduino VCC = %f Volts" % GPIO.VccRead())

print ("Serial COM port names that this virtGPIO.py tested for")
print (GPIO.portlist)
print ("Serial baudrates that this virtGPIO.py tested for")
print (GPIO.baudlist)

print ("Serial baudrate used")
print (GPIO.Serial.baudrate)
print ("Serial port used")
print (GPIO.Serial.port)

GPIO.printCompileDate()   # arduino compile/upload

print ("Observe that only pin13 is initially 'assigned'. The activity LED pin...")
GPIO.readFreePins()
time.sleep(5)

led = 9
print ("We set pin %d as OUTPUT. That does not assign the pin for any special use!" % led)
GPIO.setup(led,GPIO.OUT)
time.sleep(4)

print ("Turning off the 'activity LED'. That should release pin13")
GPIO.setActivityLed(0)
time.sleep(4)

print ("Any I2C devices detected??")
i2c = GPIO.I2C()
i2c.detect()    # Displays the addresses of any I2C devices connected
print ("But starting I2C object does assign A4 and A5")
time.sleep(5)

print ("Assigning pin8 for infrared receiver")
IR = GPIO.InfraRedRx(8)
time.sleep(4)

print ("Now trying to assign stepper motor across pins 6 7 8 9")
steppr = GPIO.Stepper(0,4, 6, 200)
time.sleep(6)

print ("That should have failed, as pin 8 is reserved for IR.")
print ("Examine free pins list again ...")
GPIO.readFreePins()
time.sleep(7)
print ("")
print ("Also, reading the 'FLAGS' shows a pin dispute reported")
print ("by stepper system at Arduino ...")
print ("Flags %s" % bin(GPIO.readFlags()))
time.sleep(4)
print ("Flag bits are defined near top of VirtGPIO.ino. Most refer to conflict when assigning pins.")
print ("\033[32m")  # green text
print ("b0 resetFlag, b1 SPI, b2 Servo, b3 IR, b4 I2C, b5 INTctr, b6 pwmplus, b7 stepper")
print ("b8 actLed, b9 SpiXferWithoutOpen,  b10 COMport  b15 unexpectedCharsArrived")
print ("\033[00m")  # normal
print ("Observe b0 = arduino reset flag has not been cleared, b7 = stepper pins conflict")
print
time.sleep(10)

print ("10 LED toggles high speed")
for k in range(10):
    GPIO.output(led, k&1) # toggle led.
print ("Done")
time.sleep(3)

print ("5000 LED toggles high speed")
for k in range(5000):
    GPIO.output(led, k&1) # toggle led.
print ("Done")
time.sleep(4)

print ("About to cause a problem: (hard) resetting the Arduino!!")
GPIO.hardResetARD()
time.sleep(5)

print ("Destroy the world. Restart this whole program ...")
print
time.sleep(3)

print ("\033[2J") # "Ansi Sequence" to clear screen. Look it up on Wikipedia
GPIO.restart_program()
