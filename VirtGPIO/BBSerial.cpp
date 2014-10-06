/* An Bit-Bang Software Serial Transmit Function - specifically for Virtual GPIO project
 * Brian Lavery  Oct 2014    http://virtgpio.blavery.com

 *
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
 */



// BIT-BANG CODE ACKNOWLEDGMENT TO:
//     http://arduining.com/2014/01/22/serial-bit-banging-with-arduino-trinket/

#include <Arduino.h>
#include "BBSerial.h"

void BBSerial::begin(uint8_t transmitPin)
{
    txpin = transmitPin;
    pinMode(txpin, OUTPUT);   // initialize the TXDATA pin as an output.
    digitalWrite(txpin,HIGH); // TXDATA = 5V when is resting.

}

size_t BBSerial::write(uint8_t c)
{
    if(txpin == 0)             // not set up???
        return 0;
    //noInterrupts();  // NO-OOOO!
  delayMicroseconds(BITTIME*2);          // wait 2 Stop bits before sending the char
  digitalWrite(txpin,LOW);              // low the line
  delayMicroseconds(BITTIME);            // wait Start bit
  for (int i=0; i<8;i++)
  {
    digitalWrite(txpin,bitRead(c, i));  // bit out.
    delayMicroseconds(BITTIME);          // wait bit
  }
   digitalWrite(txpin,HIGH);            //Return TXDATA pin to "1".
  // interrupts();

  return 1;
}


// dummy & unwanted (for Stream class)
int BBSerial::read() { return 0;}
int BBSerial::available(){ return 0;}
void BBSerial::flush(){;}
int BBSerial::peek(){return 0;}
