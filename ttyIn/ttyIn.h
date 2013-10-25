/*
  Inspired by David A. Mellis JohnOH then hacked it about 2013-10-25
  
  ttyIn.h - Software serial library
  Copyright (c) 2006 David A. Mellis.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef ttyIn_h
#define ttyIn_h

#include <inttypes.h>

uint16_t whackDelay2(uint16_t delay);

static void recv(void);

class ttyIn
{
  private:
    long _baudRate;
    void printNumber(unsigned long, uint8_t);
    
  public:
    ttyIn(uint8_t);
    void setRX(uint8_t);
    void begin();
    int read();
    uint8_t available(void);
};

#endif

