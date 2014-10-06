
# virtGPIO.py   V0.9

"""
This is the PC end of "virtual-gpio", ie using arduino as a GPIO device on PC (python on linux).
This module uses USB serial to power and control the arduino.

"""

# PC instructions (Linux or Windows or Mac):
# You require python 2.7 installed, and "pyserial".
# The arduino simply stays connected to PC by USB cable.
# On Windows, you may need to experiment with COM port numbers assigned to Arduino.

# Raspberry Pi instructions:
# You require "pyserial" installed on the rPi.
# You may choose: (1) USB cable between, or (2) TTL wires from rPi uart pins to arduino uart pins.
# Here is how to configure for using the rPi's inbuilt uart:
# (sudo) edit /boot/cmdline.txt to remove the two phrases "console=ttyAMA0,115200  kgdboc=ttyAMA0,115200"
# (sudo) edit /etc/inittab to comment out the line containing "/dev/ttyAMA0" (the last line?)
# (sudo) edit /boot/config.txt by adding a new line "init_uart_clock=16000000"
# Reboot the rPi...
# Now, (1) uart is free and available for virtGPIO (rx -> tx and tx -> rx),
#      (2) and it can go at baudrate speeds way over the usual 115200,
# Configured as above, the rPi's own GPIO-uart works quite well with virtGPIO arduino at 500000 baud.
# But see WARNING below on interfacing tx and rx wires.

# Probably you can power the arduino from rPi's 5V supply, even using UART connection.
# But any substantial current requirement (eg servos beyond one small test one) needs provision of proper supply.

# WARNING: rPI UART 3.3V TX and RX LINES MUST NOT CONNECT DIRECTLY TO 5V RX and TX OF ARDUINO ...
#   ... You must use some form of voltage level shifting to 5V arduino, or use a 3.3V version arduino.

# ADDITIONALLY ...
# Want to load the arduino sketch via this same rPi uart line?  Refer to notes at top of "VirtGPIO.ino"



if __name__ == '__main__':
    print "virtGPIO.py is an importable module for PC or Raspberry Pi:"
    print "...  import virtGPIO as GPIO"
    print "virtGPIO.py talks by USB-serial with ARDUINO-ftdi loaded with VirtGPIO.ino"
    print
    exit()

def hardResetARD():
    Serial.setDTR(False)
    time.sleep(0.3)
    Serial.setDTR(True)
    time.sleep(3)   # and wait for it

def softResetARD():
    # reset Arduino in software. (AVR registers may not be initialised to full reset)
    Serial.write("0-");
    time.sleep(2.1)



import time
import serial
from vGPIOconstants import *


STDTIMEOUT = 0.8

baudlist = [500000, 115200, 250000]
portlist = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyUSB2', '/dev/ttyAMA0', 'COM1', "COM3", "COM8", "COM9"]
# (these lists may be edited. 500000 baud recommended, list it first.)
# (I'm not a Mac person. So of course, I haven't tested a Mac.
#           Try "ls /dev/tty.*" in terminal to help find your Mac serial port name.)

for k in range(len(portlist)):
    try:
        Serial = serial.Serial(portlist[k], 9600, timeout=STDTIMEOUT, dsrdtr=False)
        break;
    except:
        pass
try:
    #print "Port %s " % Serial.port
    pass
except:
    print "Failed to initialise Serial Port."
    print "Tried %s." % portlist
    print "Check your port name??"
    exit()

if not Serial.port == "/dev/ttyAMA0":
    # NOT for Raspberry Pi UART port: it has no DTR line
    #print "Resetting arduino ..."
    hardResetARD()

# Now, we know that the serial port exists. Try certain baudrates to talk with arduino ...
for k in range(len(baudlist)):
    Serial.setBaudrate(baudlist[k])   # Try each baudrate in turn from our list
    time.sleep(0.1)
    #print "ping @", Serial.baudrate
    Serial.write("=")  # The sync() call used as a "ping"
    ping = Serial.read()   # Fetch one reply character
    if len(ping)>0:
        if ping == "=":    # Does this look like correct response?
            break;
    if k==(len(baudlist)-1):   # tried & failed on all options
            print "Arduino not in contact. Tried baudrates", baudlist
            exit()
#print "Baudrate %d " % Serial.baudrate

if Serial.port == "/dev/ttyAMA0":   # Raspberry Pi GPIO uart?
    # on rPi UART mode we needed to establish serial contact before reset: this reset is a software command.
    #print "Resetting arduino ..."
    softResetARD()

# END OF SETUP

# All below is for user functions

######################################################################

def pinMode(dpin, mode):
    setup(dpin, mode)
def setup(dpin, mode):
    Serial.write("s" + chr(dpin) + chr(mode));

def output(dpin, hilo):    # rPi style dig write
    digitalWrite(dpin,hilo)
def digitalWrite(dpin, hilo):  # arduino style
    Serial.write("w" + chr(dpin) + chr(hilo))


def input(dpin):    # rPi style       dig input
    return digitalRead(dpin)
def digitalRead(dpin):  # regular arduino style digitalRead
    Serial.write("r" + chr(dpin))
    return _serialread1int8(0xff)

def digitalPreciseRead(pin):
    # Use analog comparator on to compare the pin's ANALOG volts against 1.1V chip reference.
    # ONLY for pins d7 and A0-A7
    AVR.bitClear(AVR.ADCSRA, 7)   # disable ADC
    AVR.write8(AVR.ACSR, 0x40);  # set internal ref 1.1 for comparator
    if pin == 7:
        AVR.write8(AVR.ADCSRB, 0)  # set d7 as input
    elif (pin>= A0 and pin <= A7):
        AVR.write8(AVR.ADCSRB, (pin-14) + 0x40)  # set adc mux address 0-7 as input
    else:
        AVR.bitSet(AVR.ADCSRA, 7)   # re-enable ADC
        return 0xf4   # == f4 bad pin
    r = ((~AVR.read8(AVR.ACSR)) >> 5) & 1   # comparator output bit
    AVR.bitSet(AVR.ADCSRA, 7)   # re-enable ADC
    return r


def digitalReadAll():
    return (AVR.read8(AVR.PIND) & 0xf6) +  ((AVR.read8(AVR.PINB)&0x3f)<<8)
    # d2 to d7, and d8 to d13

def pwmWrite(dpin, dcycle):
    Serial.write("p" + chr(dpin) + chr(dcycle))   # 8bit write
    # p3 p11 Timer2,  p9 p10 Timer1,   p5 p6 Timer0

def analogRead(apin):  # regular arduino style analog read
    # HOWEVER, use the A0 A1 numbering (ie 14-21, not 0-7)
    if apin<=7:
        apin += 14
    Serial.write("a" + chr(apin))
    return _serialread2int16(0xfffa)

def analogReadAll():
    Serial.write("A")
    buf2 = Serial.read(16)
    # pyserial's serial.read() returns STRING (of 16 CHARs)
    # but we want LIST of 8 x 16-bit integers as our return format
    L = len(buf2)
    if L<16:        # timeout?
        buf3 = [0] * 8    # empty dummy of the correct length/format
    else:
        buf3 = []
        for k in range(8):
            buf3.append(ord(buf2[k*2]) + 256*ord(buf2[1+k*2]))
    return buf3

def pulseOut(dpin, mode, usec):   # strobe to 255 uSec
    # Used for CE pin on RF24. Upward pulse.   (downpulse nyi)
    Serial.write("U" + chr(dpin) + chr(mode&1) + chr(usec))

def pulseIn(pin, hilo, timeout1, timeout2):   #  pulsein
    # int timeouts: To start 20 - 5000 usec, length 100uSec - 25000uSec   8 bit
    Serial.write("I" + chr(pin) + chr(hilo) + chr(timeout1/20) + chr(timeout2/100))
    # 16-bit integer return in uSec
    return _serialread2int16(0xfff2)
    # Note return error codes could be FFE3 (pin conflict), FFFF (not idle),
    #    FFFE (not started on time), FFFA (fail to finish in time)
    # These come from pulse_In function or case 'I' section in VirtGPIO.ino


def analogHiSpeed():
        # This moves ADC clock from divider 128 to divider 16
        # Not very useful for single analog reads, but useful for analogReadAll()
        AVR.bitClear(AVR.ADCSRA, AVR.ADPS0);
        AVR.bitClear(AVR.ADCSRA, AVR.ADPS1);
        AVR.bitSet(AVR.ADCSRA, AVR.ADPS2);

def setActivityLed(pin):
  Serial.write ("_" + chr(pin))
  # The Activity Pin defaults as d13. It flashes while arduino is processing one incoming serial packet.
  # Can be anytime reassigned to be a different pin, or to 0 which means disabled.
  # SPI or other need for pin13 needs ActivityPin to have been reassigned first.


###############################################

class _AVR_:    # class not intended to be instantiated by an outside script
    ADCSRA = 0x7a
    ADPS0 = 0
    ADPS1 = 1
    ADPS2 = 2
    TCNT1 = 0x84   # (0x84 lsB    0x85 msB)
    TCCR1A = 0x80
    TCCR1B = 0x81
    CS10 = 1
    CS11 = 2
    CS12 = 4
    PINB = 0x23    # direct register addresses of digital inputs
    PINC = 0x26
    PIND = 0x29
    PORTB = 0x25
    PORTC = 0x28
    PORTD = 0x2b
    DDRB = 0x24
    DDRC = 0x27
    DDRD = 0x2a
    ACSR = 0x50
    ADCSRA = 0x7a
    ADCSRB = 0x7B

    # add to this list from 328 manual as needed

    def __init__(self):
        pass


    def read8(self, register):
       Serial.write("#" + chr(register))
       return _serialread1int8(0)

    def write8(self, register, data8):
        Serial.write("*" + chr(register) + chr(data8))


    def read16(self, register):
       Serial.write("@" + chr(register))
       return _serialread2int16(0)

    def write16(self, register, data16):
        #Serial.write("&" + chr(register) + chr(data16&0xff) + chr(data16>>8))
        Serial.write("&" + chr(register) + str2FromInt(data16))


    def bitSet(self, register, bit):
        Serial.write("[" + chr(register&0xff) + chr(bit));

    def bitClear(self, register, bit):
        Serial.write("]" + chr(register&0xff) + chr(bit));

AVR = _AVR_()
# We pre-define the AVR object here. Outside script can now use this AVR object this way.

###############################################

# for future versions, SONAR class may be removed to external file.

class Sonar:
  def __init__(self, trigger, echo):
      self.trigPin = trigger
      self.echoPin = echo
      digitalWrite(trigger, LOW)  # attempt to pre-set it low before enabling output
      setup(trigger, OUTPUT)
      digitalWrite(trigger, LOW)
      setup(echo, INPUT)


  def ping(self):              # SR04 sonar
      # Using primitives (equiv strobeout/pulsein together). SR04 has abt 450 uSec from trig to echo, plenty of leeway.
      Serial.write("U" + chr(self.trigPin) + chr(1) + chr(10) + "I" + chr(self.echoPin) + chr(HIGH) + chr(30) + chr(220))
      # 10uSec pulse, 30x20=600uSec max wait for echo start, 220x100=22mSec max echo
      Serial.timeout = 1.2
      a = _serialread2int16(0xfff1)    # a is uSec for ultrasonic ping echo to return
      Serial.timeout = STDTIMEOUT
      if a<18750:    # 250 cm
        return a/70   # result in cm
      else:
        return 0
      # In air, sound travels about 1 cm / 29 uSecs. Wikipedia. Return trip = 58 uSec / cm of range.
      # That's the theory! ie result = a/58.
      # However, my empirical calibration gives formula a/70.   YMMV - correct the scaling for yourself!


###############################################



class Servo:
    # Using any servos disables PWM on pins 9-10  (uses Timer1)

    def __init__(self, pin):   #  pin 3 - 10
        setup(pin, OUTPUT)
        self.pin = pin
        pass

    def write(self, value):
        Serial.write("V" + chr(self.pin) + chr(value))  // 2-10, 0-180

    def stop(self):
        Serial.write("v" + chr(self.pin))


###############################################



class SpiDev:
    # minimal emulation of rPi-style spidev
    # Note arduino pin 10 is conventionally used as the CSN/SS* output pin.  It ought be preferred as first CE.
    # This code supports any number of devices on SPI bus,
    # Some of the comments below imply RF24L01 2.4GHz transceiver is attached to SPI ...
    # ... however the SPI should be generic enough anyway.


    def __init__(self, cePin, mode=0):
        self.cePin = cePin
        Serial.write("S" + chr(self.cePin) + chr(mode&3));  # take SPI control of pins 11-13, plus CE-pin
        # Note SPI mode setting takes effect only on first call to SpiDev()


    def xfer2(self, buf):
        #  buf arrives as a list of 8-bit integers, max len 33 (suits NRF24)
        #  buf includes first byte = cmd byte, plus any payload (length 0-32) - RF24
        #  At the arduino, the CE* select is automatically asserted (low) around this block.
        L = len(buf)
        # set it as STRING of CHARS for serial.write:
        Serial.write("X" + chr(self.cePin) + chr(L) + ''.join(chr(e) for e in buf))
        buf2 = Serial.read(L)
        # pyserial's serial.read() returns STRING (of CHARs)
        # but we want LIST of 8-bit integers as our return format
        L2 = len(buf2)
        if L2<L:        # timeout?
            buf3 = [0] * L    # empty dummy of the correct length/format
            buf3[0] = 0xfe    # our fail flag: optionally caller can watch for this.
        else:
            buf3 = []
            for k in range(L):
                buf3.append(ord(buf2[k]))
        return buf3
        # return = same length as input, first byte of return is STATUS (RF24)

    def close(self):
        print "SPI CLOSE not supported"

###############################################

class SerialTx:
    # up to five TTL (5V) "COM ports", transmit only, bit-banged (no interrupts or timers)
    # unbuffered, 2400 baud, blocking (exc port 0) until sent.
    # All ports are at same baudrate, compiled at arduino.
    # (Can recompile to 75 300 2400 9600 19200 38400 57600.)
    # PORT 0 IS DIFFERENT: output at arduino is buffered and transmitted by loop() when idle.
    # Ports 1 to 4 are direct transmit.
    # Port 0 was used for internal sketch diagnostic. Using it here will over-ride diag use.

    # To send binary data, submit it as a character string.

    def __init__(self, port, pin):  # port 0-4, pin any digital-out capable pin
        self.port = port
        Serial.write(chr(0xc0) + chr(port) + chr(pin))

    def Print(self, x):   # single parameter
        if not (type(x) == type ("123")):
                x = str(x)   # Whatever format, coerce it into string
        Serial.write (chr(0xc1) + chr(self.port) + chr(len(x)) + x)
        return _serialread1int8(0)

    def PrintLN(self, x):
        x = str(x) + "\n"
        return self.Print(x)

    def Write(self, x):
        self.Print(chr(x))


###################################################

class SerialRx:
    # pin 8 only
    def __init__(self, baud):
        Serial.write(chr(0xd0) + chr(3))

    def available(self):
        Serial.write(chr(0xd1))
        # returns a count
        return _serialread1int8(0)

    def read(self, count):
        Serial.write(chr(0xd2) + chr(count))
        buf2 = Serial.read(count)
        # pyserial's serial.read() returns STRING (of CHARs)
        # but we want LIST of 8-bit integers as our return format
        L2 = len(buf2)
        if L2<count:        # timeout?
            buf3 = [0] * count    # empty dummy of the correct length/format
        else:
            buf3 = []
            for k in range(count):
                buf3.append(ord(buf2[k]))
        return buf3

###############################################


class I2C:

    def __init__(self):
        Serial.write("!")

    # 3 primary calls:

    def write(self, port, txbuf):
        # txbuf = list of integers
        L = len(txbuf)
        Serial.write("W" + chr(L) + chr(port)  + ''.join(chr(e) for e in txbuf));


    def read(self, port, count):
        # returns list, with result code then []*count
        Serial.write("c" + chr(count) + chr(port))
        buf2 = Serial.read(count)
        count2 = len(buf2)
        buf3 = [0] * (count+1)    # empty dummy of the correct length/format
        if count2<count:        # timeout?
            buf3[0] = 0xf3    # our fail flag: optionally caller can watch for this.
        else:
            for k in range(count):
                buf3[k+1] = ord(buf2[k])
        return buf3

    def writeRead(self, port, txchr, rxcount):   # accepts ONE byte of tx before rx
        self.write(port, [txchr])
        return self.read(port, rxcount)


    def detect(self):
        Serial.write("?")
        time.sleep(0.05)
        while(Serial.inWaiting()):   # anything to report
            print "I2C detected at: \\0x%02x " % ord(Serial.read())



    # FOLLOWING ARE SMBus STYLE COMMANDS (derived):

    def read_byte(self, addr):
        r=self.read(addr, 1)
        return r[1]

    def read_byte_data(self, addr, cmd):
        r = self.writeRead(addr, cmd, 1)
        return r[1]

    def read_word_data(self, addr, cmd):
        pass  # nyi

    def write_byte(self, addr, cmd):
        self.write(addr, [cmd])

    def write_byte_data(self, addr, cmd, data):
        self.write( addr, [cmd, data])

    def write_word_data(self, addr, cmd, data):
        pass  # nyi



###############################################

class Stepper:
    WIRE2 = 2
    WIRE4 = 4
    def __init__(self, id, wires, pin1, stepsPerRev):   # wires 2 or 4.
        Serial.write(chr(0xb1) + chr(id & 1) + chr(wires & 7)+ chr(pin1)+ str2FromInt(stepsPerRev))
        self.id = id&1
        self.SPR = stepsPerRev

    def setSpeedRPM(self, speedRPM):   #  integer speed in RPM (positive number)
        Serial.write(chr(0xb2) + chr(self.id) + str2FromInt(speedRPM))
        self.speedRPM = abs(speedRPM)

    def step(self, steps, wait=True):   # steps negative for reverse rotation +-32000
        self.steps = abs(steps)
        Serial.write(chr(0xb3) + chr(self.id) + str2FromInt(steps) )
        if wait:
            self.waitToFinish()

    def stepsLeft(self):
        Serial.write(chr(0xb4) + chr(self.id) )
        return _serialread2int16(0)

    def waitToFinish(self):
        wtime = ((1.3 * self.steps) / self.speedRPM) * 60 / self.SPR
        #print "wait time %f sec" % wtime
        t1 = time.time()
        r=0
        while (time.time() - t1) < wtime:
            if self.stepsLeft() == 0:
                r=1
                break
        #print "time taken: %f   ret= %d" % ((time.time() - t1), r)
        return r    # optional return code 0=timeout   1=normal finish



###############################################

class Intcounter:   # two counters using INT0 and INT1
    # These counters operate only on pins d2/d3. (This is a 328 chip hardware constraint.)
    # if QUAD encoder selected, d2 pairs with d4,  d3 pairs with d5

    RISING = 0x03   # AVR code
    FALLING = 0x02  # AVR code
    CHANGE =  0x01  # AVR code
    QUAD = 0x05     # change + 0x04

    def __init__(self, pin, mode):    # pin 2 or 3, mode as above
        if pin==2 or pin == 3:
            Serial.write("2" + chr(pin) + chr(mode))


    def read(self, andZero=False):   # optional counter clear, read as time-matched pair.
      Serial.write("," + chr(andZero &1))
      c0 = _serialread2int16(0xffff)
      c1 = _serialread2int16(0xffff)
      return [c0, c1]



###############################################

class HWcounter:    # true (one-only) 16bit MCU hardware counter: no interrupts.
    # This counter operates only on pin d5. (This is a 328 chip hardware constraint.)
    # This function kills any other timer1 operation (or PWM on pins 9, 10)


    def __init__(self):
      AVR.write8(AVR.TCCR1A, 0)  # reset timer/counter control register A
      AVR.write8(AVR.TCCR1B, AVR.CS10+AVR.CS11+AVR.CS12) # Counter on external pin, rising edge
      self.zero()
      # once initialised, the counter always is active

    def zero(self):   # CLEAR the counter to 0x0000
      Serial.write("&" + chr(AVR.TCNT1) + chr(0) + chr(0))  # zero the counter register

    def read(self):
      Serial.write("@" + chr(AVR.TCNT1))
      return _serialread2int16(0xffff)


###############################################


class InfraRedRx:
    # Modified IRremote library is compiled, IR pin can be controlled from host (here)
    # Uses Timer2, so PWM on p3, p11 disabled

    def __init__(self, pin):
      self.devicenum = 0   # ID of hand remote
      Serial.write("F" + chr(pin & 0x0f))

    def read(self):     # fetch IR code from remote
        Serial.write("f");
        ir = _serialread4int32(0xffffffff)
        self.devicenum = ir & 0xFFFF
        return (ir >>16)&0xff

    def device(self):    # read remote device number of latest read
        return self.devicenum
        # only valid immediately after a read


###############################################

class PWMplus:
    # only pins 9 &/or 10.  Uses Timer1

    def __init__(self):
        Serial.write (chr(0x81))
        self.pinBusy = [0,0]

    def setPeriod(self, usec):   # usec from 1 to 1,000,000
        #Serial.write (chr(0x82) + chr(usec & 0x0000ff) + chr((usec & 0x0FF00)>>8) + chr(usec/65536) )
        Serial.write (chr(0x82) + str3FromInt(usec) )

    def initPin(self, pin,duty):   # pin = 9 or 10 only.  duty 0 to 1023
        if pin < 9 or pin > 10:
            return
        if self.pinBusy[pin-9]:
            self.changeDuty(pin, duty)
            return
        self.pinBusy[pin-9] = 1
        #Serial.write (chr(0x83) + chr(pin) + chr(duty & 0x00ff) + chr(duty >>8))
        Serial.write (chr(0x83) + chr(pin) + str2FromInt(duty))

    def changeDuty(self,pin,duty):
        #Serial.write (chr(0x84) + chr(pin) + chr(duty & 0x00ff) + chr(duty >>8))
        Serial.write (chr(0x84) + chr(pin) + str2FromInt(duty))

    def releasePin(self,pin):
        if pin < 9 or pin > 10:
            return
        Serial.write (chr(0x85) + chr(pin) )
        self.pinBusy[pin-9] = 0

    def stop(self):
        Serial.write (chr(0x86) + chr(0))

    def restart(self):
        Serial.write (chr(0x86) + chr(1))

    def tone(self, pin, freq):
        self.setPeriod(1000000 / freq)
        self.initPin(pin, 511)

    def noTone(self, pin):
        self.releasePin(pin)

###############################################


# DIAGNOSTIC/DEBUG STUFF:

class Trace:   #developer use.
    def __init__(self):
        pass
    def Start(self):
            Serial.write("^")

    def Stop(self):
       Serial.write("<")

    def Print(self):
        Serial.write(">")
        # Will not print if Serial0 (ie COM port 0) is not initialised

def mopup():
    #time.sleep(.003)
    #print "Mop up: " ,
    while(Serial.inWaiting()):   # anything to mopup???
        r=Serial.read()
        print "   {%x}" % ( ord(r)),


###############################################


def sync(timeout = 0.25, printping = False):   # 0.5 - 1.0 mSec when no catchup needed
    # Use in user code after several non-reporting commands, so that sender can not
    # get too many serial characters ahead of slave unit.
    # Also useable as a "ping"
    Serial.write("=")
    Serial.timeout = timeout
    r=_serialread1int8(0xf6)
    Serial.timeout = STDTIMEOUT
    if printping:
        print "Ping response %s %x" % (chr(r),r)    # should report "=" chr
    return r

def readFreePins():
  Serial.write ("q")
  print
  print " GP pins    AAAAAAAA111100000000TR"
  print "  free      76543210321098765432rd"
  print bin(0x80300000 | _serialread4int32(0))
  # top bit: just to align the display. Bits for A6 A7 added for display. They remain always avbl.


def VccRead():
  Serial.write("+")
  return _serialread2int16(0)/1000.0


def printCompileDate():
  # debug only
  Serial.write('t')
  time.sleep(.05)
  print
  print "Compile/Load Date:  " ,
  while(Serial.inWaiting()):
    print (Serial.read()),
  print

def clearResetFlag():       # at slave
    Serial.write("i");

def resetFlagIsOn():        # optional test whether slave has since suffered a reset
    return (readFlags() & 1)

def readFlags():
    # Flag bits (confirm at VirtGPIO.ino):
    # enum {F_reset=0, F_spi, F_svo, F_ir, F_i2c, F_intctr, F_pwm, F_stepr, F_actled, F_nospi, F_badchar=15};
    Serial.write("n");
    return _serialread2int16(0xff)


import sys
import os
def restart_program():
    #Restarts the current program.
    python = sys.executable
    os.execl(python, python, * sys.argv)

################################################################################

# INTERNAL SUBROUTINES

def _serialread4int32(errorflag32):
      # read 4 byte and format as a python integer
      # if read fail, return known error flag
      a = Serial.read(4)
      if len(a) >3 :
          return ord(a[0]) + (ord(a[1])<<8) + (ord(a[2]) << 16) + (ord(a[3]) << 24)
      else:
          return errorflag32

def _serialread2int16(errorflag16):
      # read 2 byte and format as a python integer
      # if read fail, return known error flag
      a = Serial.read(2)
      if len(a) >1 :
          return ord(a[0]) + (ord(a[1])<<8)
      else:
          return errorflag16

def _serialread1int8(errorflag8):
      # read 1 bytes and format as python integer
      a = Serial.read()
      if len(a) :
          return ord(a[0])
      else:
          return (errorflag8 & 0xff)


from ctypes import *
def cByte(i):
        return c_ubyte(i).value
def cInt16(i):
        return c_ushort(i).value
def cInt32(i):
        return c_uint(i).value
def cInt32FromFloat(f):
        return cInt32(int(f))
def str1FromByte(i):
        return chr(cByte(i))
def str2FromInt(i):    # lsB first
        i = cInt16(i)
        return chr(i&0xff) + chr(i>>8)
def str3FromInt(i):  # careful! only suitable for unsigned (non-negative)
        i = cInt32(i)
        return chr(i&0xff) + chr((i>>8) & 0xff) + chr((i>>16) & 0xff)
def str4FromInt(i):
        i = cInt32(i)
        return chr(i&0xff) + chr((i>>8) & 0xff) + chr((i>>16) & 0xff) + chr((i>>24) & 0xff)

# Flagged for possible change by v1.0 [MAYBE!]:
# 1. Flexible 4 pins for stepper rather than consecutive ?
# 2. Runtime override of auto serial port name and baudrate ?
# 3. Free selection of Quad pin for quad encoder (maybe within PORTD ie 2-7) ?
# 4. Sonar to be moved out to a library module, not virtgpio.py itself.
# 5. SPI devices to get individual control of mode?
# 6. Compatibility with both python 2.7 and python 3 ?
# 7. auto detection of serial ports available using serialenum.py (https://github.com/djs/serialenum) ??

"""
 * Copyright (c) 2014 Brian Lavery <vgpio@blavery.com>   http://virtgpio.blavery.com
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
"""
