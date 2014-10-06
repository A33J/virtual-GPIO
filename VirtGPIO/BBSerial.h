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



#ifndef BBSerial_h
#define BBSerial_h

#include <inttypes.h>
#include <Stream.h>

#define BAUD_75     13325
#define BAUD_300    3325
#define BAUD_2400   410
#define BAUD_9600   93
#define BAUD_19200  45
#define BAUD_38400  19
#define BAUD_57600  10
// Expect higher baudrates to suffer unacceptable bit timing issues ( = character corruption)
//            due to interrupt events.

#define BITTIME     BAUD_2400    //       HARDCODE - dont want "calculations" during timing



class BBSerial : public Stream
{

public:
  BBSerial(uint8_t dummy = 0){txpin = 0;};
  ~BBSerial(){;};
  void begin(uint8_t transmitPin);
  uint8_t txpin;
  virtual size_t write(uint8_t byte);
  using Print::write;
   // we can now use Arduino's STREAM functions write()  print() and println() - just like real "Serial"


private:   // & mostly unwanted
  virtual int peek();
  virtual int read();
  virtual int available();
  virtual void flush();

};


#endif
