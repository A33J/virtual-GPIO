// Wrapper for BBSerial.
// pre-instantiates Serial0
// provides missing printf() (to Serial0)
// adds buffer to printf(), delayed output
// supplies missing ftoa()



/* This file specifically for Virtual GPIO project
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


#include "BBSerial.h"

// Pre-instantiates object Serial0 (bit-banged  output, baudrate hardcoded)
// Serial0.write(),  Serial0.print(),   Serial0.println() supported, Arduino Stream style.
// Also, after call Serial0Printf_begin() then conventional printf() functions redirect through Serial0.

// (Arduino printf has no float formatting. Use ftoa() from here in lieu.)

// output to Serial0.print() & Serial0.println() & Serial0.write() (Arduino's Stream) is immediate,
// output to printf() is buffered, as well as Serial0BufXXX()


#define BUFSIZE         256
 // must be power of 2
int _In = 0;   // buffer indexes
int _Out = 0;
char _Buffer[BUFSIZE];

BBSerial Serial0;   // instantiate object Serial0


void _putc(char c)
{
    // is there buffer room?
    if ((((BUFSIZE+_Out)-_In) & (BUFSIZE-1)) != 1)
    {
        _Buffer[_In] = c;
        if (++_In == BUFSIZE)
            _In = 0;      // rollover
    }

}

int _getc(void)
{
    //    exit with code if buffer empty
    if ((_In == _Out))
        return 0xFFFF;
    // extract from buffer.
    int tmp = _Buffer[_Out];
    if (++_Out == BUFSIZE)
          _Out = 0;
    return tmp;

}

void printf_run()
{
    if (Serial0.txpin ==0)
        return;
    int c = _getc();
    if (c < 0)
        return;
    Serial0.write((char)c);



}

int _serial_putc( char c, FILE * )
{
  // testing only
  Serial0.write( c );
  return c;
}

int _buffer_putc( char c, FILE * )
{
    _putc(c);
    return c;
}

// these routines Not yet implemented

void Serial0Printf_begin(void)
{

   //Assign where printf() data goes to:

  //fdevopen( &_serial_putc, 0 );    // immediate Serial0 output - testing only
  fdevopen( &_buffer_putc, 0 );      // or to buffer, waiting on printf_run() to get out
}

void Serial0BufWrite(char c)
{
    _putc(c);
}

void Serial0BufPrint(int i)
{
   //?? _putc(c);
}

void Serial0BufPrint(char * str)
{
    //??  _putc(c);
}

void Serial0BufPrint(float f)
{
   //? _putc(c);
}

#define Serial0BufPrintf   printf

// EXAMPLE USE OF FTOA():   printf("float %s", ftoa(1.9864, 2));

char _fltbuf[20];

char *_ftoa(char *a, double f, int precision)
{
  //   http://forum.arduino.cc/index.php/topic,44262.0.html
  long p[] = {0,10,100,1000,10000,100000,1000000,10000000,100000000};

  char *ret = a;
  long heiltal = (long)f;
  itoa(heiltal, a, 10);
  while (*a != '\0')
        a++;
  *a++ = '.';
  long desimal = abs((long)((f - heiltal) * p[precision]));
  itoa(desimal, a, 10);
  return ret;
}

char *ftoa(double f, int precision)
{
    return _ftoa(_fltbuf, f, precision);
}
