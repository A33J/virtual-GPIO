// VirtGPIO.ino

// This is the arduino end of virtual-gpio, using arduino as a GPIO device on PC or Raspberry Pi
// This sketch is designed to support specifically the atmega328 (eg Nano or Uno)
// If you use USB between PC and Arduino, be aware the USB/UART is required to work at high baudrate (500000 default).
// For me, FTDI types have worked flawlessly. CH340 has failed miserably. YMMV.


// Want to connect your Raspberry Pi via Rpi UART instead of USB?  Refer to notes at top of "virtGPIO.py"

// Want additionally to use arduino IDE on rPi, with sketch upload via rPi UART instead of USB cable?
// (Perhaps you use Pro-Mini arduino with no USB function?)
// Install "arduino" software on rPi using apt-get or synaptic.
// At rPi terminal enter:     ~$    "sudo ln /dev/ttyAMA0 /dev/ttyS1"
// Then Arduino IDE will recognise the UART serial port under alias of "/dev/ttyS1"
// That is good for current session. To make the alias permanent, look here:
// <http://www.linuxcircle.com/2013/04/23/install-and-test-arduino-ide-with-raspberry-pi-and-gertboard/>
// You'll need to learn (a) to be patient while recompiling - it's slow
//                      (b) to become expert in releasing the reset button EXACTLY at upload start time



#define NUM_TXCOMPORTS 5
#define BAUDRATE 500000
// Recommended 500000, other options 250000 and 115200.  Compare auto-find settings in virtGPIO.py

#define DEVELOPMENT
// DEVELOPMENT VERSION USES PIN A3 AS OUTPUT FOR Serial0 / COMport0

// Libraries expected in default Arduino IDE installation:
#include <SPI.h>
#include <Servo.h>
#include <Wire.h>
#include "Arduino.h"

// OPTIONAL library - this one should be installed conventionally in .../sketchbook/libraries/
//#include <MemoryFree.h>
// Optional, use at development, not included in production

// Custom and 3rd-party libraries. Note ALL the following are NOT in "...sketchbook/libraries/",
// rather they are in virtGPIO project space. That way, you don't need to install them in libraries folder,
// and in any case nearly all of them are specially tweaked for virtGPIO.
#include "IRremote_vg.h"
#include "TimerOne_vg.h"
#include "Stepper_vg.h"
#include "Serial0.h"
#include "AltSoftSerial_vg.h"

// Create various device objects. They don't really do anything yet:
BBSerial Serial1[NUM_TXCOMPORTS];   //  (we will waste [0])
AltSoftSerial Serial2;
IRrecv  IR(0);   // dummy pin# for original constructor
decode_results IRresult;
Stepper stepper[2];
Servo servos[12];   // for pins 2 - 11 - we will keep those matched. Bottom 2 wasted
int activityLED = 13;


volatile unsigned int d2_pulses=0, d3_pulses=0; // counters for INTcounter on 2 or 3
int quadpin[2];   // the pin #s (if existing) for quad pins of QuadEncoder (ie INTcounter)
unsigned char buf[40];  // for SPI
bool SPI_on = false;
bool IR_on = false;
bool I2C_on = false;
unsigned int flags = 1;  // b0 is resetFlag.  16 bit flags to log pin collisions etc.
long int IRvalue = 0L;   // one value IR read buffer
enum {
  F_reset=0, F_spi, F_svo, F_ir, F_i2c, F_intctr, F_pwm, F_stepr, F_actled, F_nospi, F_TxCom, F_RxCom, F_badchar=15};
// bits in this flag register can be fetched to PC. logs failures to assign pins to devices.

// Some shortcut macros used later:
#define SERIALWRITE2(v)        {Serial.write(v & 0xff);Serial.write((v >> 8)&0xff);}
#define SERIALWRITE3(v)        {Serial.write(v & 0xff);Serial.write((v >> 8)&0xff);Serial.write((v>>16)&0xff);}
#define LOGFAIL(x)             bitSet(flags, x)


//------------------------------------------------------------------------------------


// TRACE: stored diagnostic data (bytes!) that can be DUMPed to Serial0 for debug purposes.
// "Trace" is debugging tool used by author.
// Careful that trace buffer is 3x smaller than Serial0 buffer!!
#define TRACEBUFSIZE 70
#define TRACE2(v)              {Trace(v & 0xff);Trace((v >> 8)&0xff);}
#define TRACE3(v)              {Trace(v & 0xff);Trace((v >> 8)&0xff); Trace((v >>16) & 0xff);}
#define TRACE4(v)              {Trace(v & 0xff);Trace((v >> 8)&0xff); Trace((v >>16) & 0xff);Trace(v>>24);}
unsigned char traceBuf[TRACEBUFSIZE];
int tracing = 0, traceCtr = 0;
void Trace(char t)
{
  // this buffer accumulates until DUMP time
  if ((traceCtr < TRACEBUFSIZE) && tracing)
    traceBuf[traceCtr++] = t;
  // intended for simple byte or integer data displayable in hex on external terminal
}


//------------------------------------------------------------------------------------

// Reserved Pins control:

long GPpins = 0b00011111111111111111100 ;
// pins d0(lsb) - d13 - a0 - a5(bit19) bit pattern of "good" pins, ie still available for general I/O
// pins d0,d1 (tx & rx) are OFF, also everything above A5.  Remaining pins are "good"


void reservePin(int pin)
{
  GPpins &= ( ~ (1L<<(pin&0x1f)));    // use _BV?
  // no longer available as general I/O
}

void releasePin(int pin)
{
  GPpins |= ( 1L<<(pin&0x1f));
}

bool pinGood(int pin)
{
  return (GPpins & (1L<<(pin&0x1f))) > 0L ;
  // is this pin still available for general I/O?
}


//------------------------------------------------------------------------------------

// interrupt handlers for INT0 and INT1:
void countD2Pulses() {
  char pind = PIND;  // read both pin conditions
  char pinlevel = (pind & _BV(2)) > 0;
  char qlevel = pinlevel;   // default for non-quad
  if (quadpin[0] >0)
    qlevel = (pind & _BV(quadpin[0])) > 0;
  d2_pulses = d2_pulses + ((pinlevel == qlevel) ? 1 : (-1));
}

void countD3Pulses() {
  char pind = PIND;
  char pinlevel = (pind & _BV(3)) > 0;
  char qlevel = pinlevel;   // default for non-quad
  if (quadpin[1] >0)
    qlevel = (pind & _BV(quadpin[1])) > 0;
  d3_pulses = d3_pulses + ((pinlevel == qlevel) ? 1 : (-1));

}


void (*softReset)() = 0;
// "dirty reset" - peripherals & registers not reset.

// objects created by now: spi, wire, IRrecv + decode_results struct, encoders[], servos[], COM ports,
// but no "activity" or code for those yet.


//------------------------------------------------------------------------------------


void setup(void)
{
  for(int k=2; k<=19; k++)
    pinMode(k, INPUT);
  reservePin(activityLED);
  Serial.begin(BAUDRATE);
  pinMode(activityLED,OUTPUT);
  digitalWrite(activityLED,HIGH);
  delay(30);
  digitalWrite(activityLED,LOW);
  delay(100);
  digitalWrite(activityLED,HIGH);
  delay(30);
  digitalWrite(activityLED,LOW);  // simply a reset marker of 2 flashes
  quadpin[0]=0;
  quadpin[1]=0;


#ifdef DEVELOPMENT
  delay(150);
  digitalWrite(activityLED,HIGH);
  delay(90);
  digitalWrite(activityLED,LOW);  // DEVELOPMENT VERSION =  3 flashes
  Serial0.begin(17);
  reservePin(17);
  Serial0Printf_begin();
  printf("This is debug port for Virtual GPIO\n");    // - buffered
  Serial0.println("\n\nHello:");  // this prints first!!  - immediate
#ifdef MEMORY_FREE_H
  printf("Free mem %d\n", freeMemory());
#endif
  printf("Serial %ld baud\nV0.9 Compiled %s\nReady ...", BAUDRATE, __DATE__);
#endif


}



void loop ()
{
  static int SPI_modes[4] = {
    SPI_MODE0, SPI_MODE1, SPI_MODE2, SPI_MODE3    };
  unsigned int pin, pin2, mode , bufIndex, SPR;
  unsigned int speed, aVal, dVal, ln, icount, k, registAddr, quad, id, port;
  long int usec;
  int  steps;
  unsigned char cmd, c;
  unsigned long timeo1, timeo2;
  static unsigned long tim;
  static unsigned int idleCounter = 0;



  if (Serial.available () > 0)
  // ie non-blocking at this point
  {

    cmd = serGetcharB ();
    idleCounter = 0;
    if(activityLED)
      digitalWrite(activityLED,HIGH);

    switch (cmd)
    {

    case 'S':
      // SPI open/init
      pin = serGetchar() & 0x1f;    //    CE pin
      mode = SPI_modes[serGetchar() & 3];
      if (!pinGood(pin))
      {
        LOGFAIL(F_spi);
        break;   // CE pin not free?   (& this inhibits a 2nd "open" for that CE# too)
      }
      if ((!SPI_on) && (!(pinGood(MISO) && pinGood(MOSI)) ))
      {
        LOGFAIL(F_spi);
        break;
      }
      if ((!SPI_on) && !(pinGood(SCK) ) )   // SCK is pin13
      {
        if(activityLED != SCK)
        {
          // ActivityLed wasn't the reason
          LOGFAIL(F_spi);
          break;
        }
        // Ah, the reason pin13 is reserved is that it is activity LED. Kick it out!
        activityLED = 0;
        digitalWrite(13,0); // turn 13 off
        pinMode(13, INPUT);   // then let SPI system sort 13 out.
      }
      pinMode(pin, OUTPUT);
      digitalWrite(pin, HIGH);
      reservePin(pin);
      if(!SPI_on)   // first "open"?
      {
        SPI.begin();   // start the SPI engine
        SPI.setBitOrder(MSBFIRST);
        SPI.setDataMode(mode);
        SPI.setClockDivider(SPI_CLOCK_DIV4);
        reservePin(SCK);  // 13
        reservePin(MOSI); // 11
        reservePin(MISO); // 12
        delay(1);
      }
      SPI_on = true;
      break;

    case 'X':
      // SPI transfer block. CSN/CEx asserted down around the block of transfers
      //  timings: 100 uSec @ 1 char,   1700 uSec @ 32 char
      pin = serGetchar() & 0x1f;
      //  actual pin# for CE arrives here  (= 10-7)
      ln = serGetchar() & 0x1f;
      for (k=0; k<ln; k++)
        buf[k] = serGetchar();
      if (!SPI_on)
      {
        LOGFAIL(F_nospi);
        break;    // yeah! the caller is going to timeout waiting!!
      }
      digitalWrite(pin, LOW);
      for (bufIndex=0; bufIndex<ln; bufIndex++)
        Serial.write(SPI.transfer(buf[bufIndex]));
      // exact reply from SPI - send it straight out
      digitalWrite(pin, HIGH);
      break;


    case 's':
      // set digital pin mode to INPUT/OUTPUT
      pin = serGetchar () & 0x1f;
      mode = serGetchar() & 0x03;
      if (pinGood(pin))
        pinMode (pin, mode) ;
      break ;

    case 'w':
      // digital write  hi lo
      pin = serGetchar () &0x1f ;
      mode = serGetchar() & 0x0003;
      if (pinGood(pin))
        digitalWrite (pin, mode) ;
      break ;

    case 'U':
      // Upwards strobe pulse on digital pin
      pin = serGetchar () &0x1f ;
      mode = serGetchar() & 1;   // 0 or 1: HI or LO pulse?
      ln = serGetchar() & 0x00ff;
      if (pinGood(pin))
      {
        digitalWrite (pin, mode) ;
        delayMicroseconds(ln);
        digitalWrite (pin, 1-mode) ;
      }
      break ;

    case 'I':
      // pulseIn   - custom version, not arduino native
      pin = serGetchar() & 0x0f;
      mode = serGetchar() & 1;
      timeo1 = serGetchar() * 20;   // 1-255 = 20uSec - 5mSec
      timeo2 = serGetchar() * 100;  // 1-255 = 100uSec - 25mSec
      if (pinGood(pin))
        dVal = (int)pulse_In(pin, mode, timeo1, timeo2);
      else
        dVal = 0xFFE3;  // = error
      SERIALWRITE2(dVal);   // returning only 16 bit. (to 65 mSec)
      break ;


    case 'i':
      // Kill the "has been reset" flag
      bitClear(flags, F_reset);            // resetflag
      break;

    case 'n':
      // Read all the Flags register
      SERIALWRITE2(flags);
      break;


    case 'r':
      // digital read
      pin = serGetchar () & 0x1f ;
      if (!pinGood(pin))
        dVal = 0x00fe;  // = error
      else
        dVal = digitalRead (pin) ;
      Serial.write (dVal ) ;
      break ;


    case 'a':
      // analog read
      // expects pins A0 - A7 ie 14 - 21 (not 0-7)
      pin = serGetchar () & 0x1f ;
      if ((pinGood(pin) && pin>= 14) || pin==20 || pin==21)  // A6 A7 always useable
        aVal = analogRead (pin) ;
      else
        aVal = 0xFFEE; // = error
      SERIALWRITE2(aVal);
      break ;

    case 'A':
      // 8 x analog read ALL
      for (pin=14; pin<=21; pin++)
      {
        SERIALWRITE2(analogRead (pin));
      }
      break ;

    case 'p':
      // PWM write to digital pin - pin 2 up to pin 11
      pin  = serGetchar () & 0x0f ;
      dVal = serGetchar () ;
      if (pin > 11 || !pinGood(pin) || pin==4 || pin==7 || pin==8)
        break;
      analogWrite (pin, dVal& 0xff) ;
      break ;

    case 'V':
      // attach & run servo
      pin = serGetchar() & 0x0f;
      dVal = serGetchar();
      if (!pinGood(pin))
      {
        LOGFAIL(F_svo);
        break;
      }
      if (!servos[pin].attached())
        servos[pin].attach(pin);
      servos[pin].write(dVal);
      reservePin(pin);
      break;

    case 'v':
      // stop a servo
      pin = serGetchar() & 0x0f;
      if (servos[pin].attached())
      {
        servos[pin].detach();
        releasePin(pin);
      }
      break;

    case 'F':
      // infrared start
      pin = serGetchar();
      if (IR_on)
        break;
      IR.blink13(false);  // we not use blink
      if(!pinGood(pin))
      {
        LOGFAIL(F_ir);
        break;
      }
      reservePin(pin);
      IR.enableIRIn(pin);   // here is where we over-ride the preset pin#
      IR_on = true;
      break;

    case 'f':
      // infrared rx
      SERIALWRITE2(IRvalue >> 16);
      SERIALWRITE2(IRvalue & 0x0000FFFF);
      IRvalue = 0L;
      break;

    case '=':
      // sync / ping
      Serial.write('=');
      break;


    case '_':
      // reassign the activity led         or 0 = off
      pin2 = serGetchar() & 0x1f;
      pin = activityLED;
      if (pin == pin2)
        break;
      if (pin2 != 0)
      {
        if (!pinGood(pin2))
        {
          LOGFAIL(F_actled);
          break;
        }
        reservePin(pin2);
        pinMode(pin2, OUTPUT);
        digitalWrite(pin2, LOW);
      }
      activityLED = pin2;
      if (pin != 0)
      {
        digitalWrite(pin, LOW);
        releasePin(pin);
        pinMode(pin,INPUT);
      }
      break;

    case 't':
      // compilation date
      Serial.print(__DATE__);
      Serial.print("  VirtGPIO V0.9");
      break;

    case '+':
      // Read Arduino Supply volts VCC
      SERIALWRITE2((int)readVcc());
      break;

    case 'q':
      // Read set of reserved pins
      SERIALWRITE2((int)(GPpins & 0xffff));
      SERIALWRITE2((int)(GPpins >>16));
      break;

    case '!':
      // i2c begin()
      if (I2C_on || !pinGood(18) || !pinGood(19))
      // A4 & A5 are i2c pins
      {
        LOGFAIL(F_i2c);
        break;
      }
      Wire.begin();
      I2C_on = true;
      reservePin(18);
      reservePin(19);
      break;

    case 'W':
      // Write byte(s) to i2c
      // simply not going to work if i2c failed to start
      icount = serGetchar();
      Wire.beginTransmission(serGetchar());
      for (k=0; k<icount; k++)
        Wire.write(serGetchar());
      Wire.endTransmission();
      break;

    case 'c':
      // Read byte(s) from i2c
      // simply not going to work if i2c failed to start
      icount = serGetchar();
      Wire.requestFrom(serGetchar(), icount);
      for (k=0; k<icount; k++)
      {
        while(!Wire.available())    /////////////////// timeout logic not ok
          delayMicroseconds(5);
        Serial.write(Wire.read());
      }
      break;

    case '?':
      // find I2C addresses
      // "scanner" code
      if(! I2C_on)
      {
        Serial.write(0xfe); // = error
        break;
      }
      for(k = 1; k < 127; k++ )
      {
        Wire.beginTransmission((byte)k);
        if (Wire.endTransmission() ==0)
          Serial.write (k);
      }
      break;


    case '@':
      // AVR register read16
      registAddr = serGetchar();
      SERIALWRITE2(  *(volatile uint16_t *) registAddr );
      break;

    case '#':
      // AVR register read8
      registAddr = serGetchar();
      Serial.write(  *(volatile uint8_t *) registAddr );
      break;

    case '*':
      // AVR register write8
      registAddr = serGetchar();
      dVal = serGetchar();
      *(volatile uint8_t *) registAddr = dVal;
      break;

    case '&':
      // AVR register write16
      registAddr = serGetchar();
      dVal = serGetchar();
      dVal = (dVal<<8) + serGetchar();   // check endian ?!!!
      *(volatile uint16_t *) registAddr = dVal;
      break;

    case '[':
      // AVR register8 bitset
      registAddr = serGetchar();
      k = serGetchar() & 0x07;  // the bit
      c = *(volatile uint8_t *) registAddr;
      *(volatile uint8_t *) registAddr = ( c | _BV(k));
      break;


    case ']':
      // AVR register8 bitclear
      registAddr = serGetchar();
      k = serGetchar() & 0x07;
      c = *(volatile uint8_t *) registAddr;
      *(volatile uint8_t *) registAddr = ( c & (~ _BV(k)));
      break;


    case '2':
      // d2/d3 pulses by int0 / int1 - INIT
      pin = serGetchar() & 3;    // 2 or 3
      mode = serGetchar() & 7;   //  00 low (bad!), 01 change, 10 falling, 11 rising,  101 quad_changing
      if (pin<2 || pin>3 || !pinGood(pin) || (mode&3) ==0)  // bad pin#, bad LOW mode, pins busy
      {
        LOGFAIL(F_intctr);
        break;
      }
      quad = mode >>2;    // test isquad bit
      if (quad && !pinGood(pin+2))   // quad pin busy?
      {
        LOGFAIL(F_intctr);
        break;
      }
      mode &= 3;   // only 01  10  11 now
      if(quad)
      {
        //reservePin(pin+2);  // NO don't reserve
        quadpin[pin-2] = pin+2;
      }
      //reservePin(pin);
      attachInterrupt(pin&1, (pin&1)? countD3Pulses : countD2Pulses, mode);
      break;

    case ',':
      // int0 and int1 counter reads (together), Optional clear
      mode = serGetchar() & 1;
      noInterrupts();
      SERIALWRITE2(d2_pulses);
      SERIALWRITE2(d3_pulses);  // return both
      if (mode)   // clear? true/false
      {
        d2_pulses = 0;
        d3_pulses = 0;
      }
      interrupts();
      break;


    case 0x81:
      // init hi def pwm
      Timer1.initialize();
      break;

    case 0x82:
      // set pwm period
      usec = serGetchar();
      usec += (serGetchar()<<8);
      usec += (((long int)serGetchar())<<16);
      Timer1.setPeriod(usec);
      break;

    case 0x83:
      // init pwm pin
      pin = serGetchar() & 0x1F;
      mode = serGetchar();
      mode += (serGetchar()<<8);
      if (pinGood(pin))
      {
        Timer1.pwm(pin, mode);
        reservePin(pin);
      }
      else
        LOGFAIL(F_pwm);
      break;

    case 0x84:
      // change pwm duty
      pin = serGetchar() & 0x1F;
      mode = serGetchar();
      mode += (serGetchar()<<8);
      Timer1.setPwmDuty(pin, mode);
      break;

    case 0x85:
      // release pwm pin
      pin = serGetchar() & 0x1F;
      Timer1.disablePwm(pin);
      releasePin(pin);
      break;

    case 0x86:
      mode = serGetchar() & 1;
      if (mode)
        Timer1.restart();
      else
        Timer1.stop();
      break;


    case 0xb1:
      // stepper init
      id = serGetchar()&1;
      mode = serGetchar()&6;  // wires 2 or 4
      pin = serGetchar() & 0x1f;    // first of 2 or 4 pins
      SPR = serGetchar();
      SPR += (serGetchar()<<8);    // Steps/revolution
      if (!pinGood(pin) || !pinGood(pin+1))   // 2 pin
      {
        LOGFAIL(F_stepr);
        break;
      }
      if ((!pinGood(pin+2) || !pinGood(pin+3)) &&  (mode ==4))   // 4 pin
      {
        LOGFAIL(F_stepr);
        break;
      }
      reservePin(pin);
      reservePin(pin+1);
      if (mode == 4)
      {
        stepper[id].init((int)SPR, pin, pin+1, pin+2, pin+3);
        reservePin(pin+2);
        reservePin(pin+3);
      }
      else
        stepper[id].init((int)SPR, pin, pin+1);
      break;


    case 0xb2:
      // set stepper speed (in RPM)   positive only
      id = serGetchar()&1;
      speed = serGetchar();
      speed += (serGetchar()<<8);    // int16
      stepper[id].setSpeed((long)speed);
      break;

    case 0xb3:
      // stepper move
      id = serGetchar()&1;
      steps = serGetchar();
      steps += (serGetchar()<<8);    // + or -  int16
      stepper[id].step(steps);

      break;

    case 0xb4:
      // get steps left
      id = serGetchar()&1;
      SERIALWRITE2(stepper[id].stepsLeft());

    case 0xc0:
      // set up COM port
      port = serGetchar() & 0x0f;
      pin = serGetchar() & 0x1f;
      if (port >= NUM_TXCOMPORTS)
        break;

      if (!pinGood(pin))
      {
        // uh oh that pin currently lists as "reserved"
        for (k=0; k<=NUM_TXCOMPORTS; k++)   // scan all serial ports
          if (pin == (k ? Serial1[k].txpin : Serial0.txpin)) // that pin already used for serial??
            break;
        if (k >= NUM_TXCOMPORTS)   // ie we found no pin match among the serial ports?
        {
          LOGFAIL(F_TxCom);
          break;  // fail to install new port: reserved pin
        }
        // that pin is used for COM.  Re-use is OK, so continue on ...
      }
      if (port ==0)
      {
        Serial0.begin(pin);
        // this MAY rewrite the pin# for Serial0.  Legal.  Bad luck for any developer diags.
        Serial0Printf_begin();
      }
      else
        Serial1[port].begin(pin);
      reservePin(pin);
      break;

    case 0xc1:
      // write to TxCOM port
      port = serGetchar() & 0x0f;
      ln = serGetchar();
      // note, if port not set up, the write function of port will return without action
      for (k=0; k<ln; k++)
      {
        if (port ==0)
          Serial0BufWrite(serGetchar());
        else
          Serial1[port].write(serGetchar());
      }
      Serial.write(ln);
      break;

    case 0xd0:
      // Initialise RXSerial port
      mode = serGetchar();
      if (pinGood(8))
      {
        Serial2.begin(2400);
        reservePin(8);
      }
      else
        LOGFAIL(F_RxCom);
      break;

    case 0xd1:
      // RxSerial   chrs available?
      Serial.write( Serial2.available());

      break;

    case 0xd2:
      // return characters from RxSerial
      icount = serGetchar();
      for (k=0; k<= icount; k++)
        Serial.write(Serial2.read() & 0xff);
        // NOTE: requesting more than in buffer will result in FF characters to fill the quota!

      break;

    case '^':
      // trace on
      tracing = 1;
      //traceCtr = 0;
      break;
    case '<':
      // trace off
      tracing = 0;
      Serial0BufPrintf("OK %d\n", 44);
      break;

    case '>':
      // trace off & dump
      tracing = 0;
      if(Serial0.txpin == 0)
        break;   // Serial0 is not initialised. abort.

      // whole trace buffer is now sent to Serial0's buffer for transmission/display:
      Serial0.print("< ");
      for (int k=0; k<traceCtr; k++)
      {
        Serial0.print(traceBuf[k], HEX);
        Serial0.print(" ");
      }
      Serial0.print('>');
      traceCtr = 0;
      break;

    case '0':
      // Arduino reset
      mode = serGetchar();
      if (mode == '-')
        softReset();
      break;



    default:
      // unknown/unexpected cmd codes are discarded
      LOGFAIL(F_badchar);   // can be detected at PC by fetching flags register
      break;



    }


    if(activityLED)
      digitalWrite(activityLED,LOW);

    //TRACE4((micros()-tim));
    // End of CMD processing
  }
  else
  {
    // this loop was idle, no command was processed
    idleCounter ++;
  }


  tim = micros();

  // Every pass, if IR chr came in (under interrupt!), stow it, & resume looking.
  // No multi-chr buffer.  A new code overwrites earlier one if not retrieved by user.
  if (IR_on)
    if(IR.decode(&IRresult))
    {
      if(IRresult.value != 0xFFFFFFFF)   // "fast repeat chr" code  ignore
        IRvalue = IRresult.value;
      IR.resume();
    }


  // every pass, if Steppers are installed, service them:
  for (id=0; id<=1; id++)
    stepper[id].run();

  if (idleCounter>10)   // are we looking fairly "idle"?
    printf_run();  // transmit 1 buffered character to Serial0, if available

}

// End of loop()
//------------------------------------------------------------------------------------


long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC))
    ;
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  ADMUX = (DEFAULT << 6);     // Aref back to default for every future read
  return result;
}

long int pulse_In(int pin, int level, unsigned long timeout1, unsigned long timeout2)
{
  unsigned long t2, t1 = micros();
  if (digitalRead(pin) == level)
    return 0xFFFFFFFd;     // fail code: not idle, or missed the start
  while (digitalRead(pin) != level)
    if ((micros()-t1) > timeout1)
      return 0xFFFFFFFE;    // fail code . Failed to start on time
  t2=micros();
  while (digitalRead(pin) == level)
    if ((micros()-t2) > timeout2)
      return 0xFFFFFFFA;    // fail code : overlong pulse - Failed to finish on time
  return  (micros()-t2);
}



int serGetcharB ()
{
  // blocking
  int x ;
  while ((x = Serial.read ()) == -1)
    ;
  return x ;
}

int serGetchar ()
{
  // nonblocking
  long tim0 = micros();
  int x ;
  while (((x = Serial.read ()) == -1) && ((micros()-tim0) < 100L ))
    ;
  return x ;
}

//------------------------------------------------------------------------------------

/*
 *  * Copyright (c) 2014 Brian Lavery <vgpio@blavery.com>   http://virtgpio.blavery.com
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
 * Library functions used alongside this file retain (as always) the copywrite
 * or free software provisions of their respective authors.
 *
*/
