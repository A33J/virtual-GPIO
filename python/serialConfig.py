# setSerial.py

# This is an optional import file for virtGPIO.py
# It allows custom selection of attempted baudrates and serial ports without modifying "virtGPIO.py"


# OPTION 1:
# If this file setSerial.py is absent,
# virtGPIO.py uses its default coded list of serial ports and baudrates to look for the arduino.



# If this file is present, either or both of baudlist and portlist may be forced here:

# OPTION 2:

# A. Force the baudlist:
# (comment out here to simply use the default list in virtGPIO.py. Uncomment here to use new option.)
#baudlist = [250000, 115200]

# B. Force the portlist:
#portlist = ['/dev/ttyUSB4', '/dev/ttyUSB5']
#portlist = ['/dev/ttyAMA0']



# OPTION 3:
# This option asks virtGPIO.py to attempt a "smart" interrogation method to find connected serial ports.
# "SMART" may not always work. Some OTHER serial device may get selected first.
portlist = "SMART"



# OTHER:
# Enable messages during serial port setup:
#verboseSetup = True


# To disable the functions in this optional file, either remove the file, or comment out all the options above



if __name__ == '__main__':
    print ("serialConfig.py is an optional importable module for virtGPIO package")
    exit()

# V0.9.0.2
