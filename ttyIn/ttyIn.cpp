/*
  Inspired by David A. Mellis JohnOH then hacked it about 2013-10-25
  
  SoftwareSerial.cpp - Software serial library
  Copyright (c) 2006 David A. Mellis.  All right reserved. - hacked by ladyada 

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

/******************************************************************************
 * Includes
 ******************************************************************************/
#include <JeeLib.h>
#include <avr/interrupt.h>
#include "ttyIn.h"
/******************************************************************************
 * Definitions
 ******************************************************************************/

#define AFSS_MAX_RX_BUFF 2  // Was 64

/******************************************************************************
 * Statics
 ******************************************************************************/
static uint8_t _receivePin;
static int _bitDelay;

static char _receive_buffer[AFSS_MAX_RX_BUFF]; 
static uint8_t _receive_buffer_index;

void whackDelay(uint16_t delay) { 
  uint8_t tmp=0;

  asm volatile("sbiw    %0, 0x01 \n\t"
	       "ldi %1, 0xFF \n\t"
	       "cpi %A0, 0xFF \n\t"
	       "cpc %B0, %1 \n\t"
	       "brne .-10 \n\t"
	       : "+r" (delay), "+a" (tmp)
	       : "0" (delay)
	       );
}
//#endif

/******************************************************************************
 * Interrupts
 ******************************************************************************/

SIGNAL(SIG_PIN_CHANGE0) 
  {
        recv();
  }

void recv(void) { 
  char i, d = 0; 
  if (digitalRead(10)) 
    return;       // not ready! 
  whackDelay(_bitDelay - 8);
  for (i=0; i<8; i++) { 
    //PORTB |= _BV(5); 
    whackDelay(_bitDelay*2 - 6);  // digitalread takes some time
    //PORTB &= ~_BV(5); 
    if (digitalRead(10)) 
      d |= (1 << i); 
   } 
  whackDelay(_bitDelay*2);
  if (_receive_buffer_index >=  AFSS_MAX_RX_BUFF)
    return;
  _receive_buffer[_receive_buffer_index] = d; // save data 
  _receive_buffer_index++;  // got a byte 
} 
  


/******************************************************************************
 * Constructors
 ******************************************************************************/

ttyIn::ttyIn(uint8_t receivePin)
{
  _receivePin = receivePin;
  _baudRate = 19200;
}

void ttyIn::setRX(uint8_t rx) {
  _receivePin = rx;
}

/******************************************************************************
 * User API
 ******************************************************************************/

void ttyIn::begin()
{
  pinMode(_receivePin, INPUT); 
  digitalWrite(_receivePin, HIGH);  // pullup!
/*
  _baudRate = speed;
  switch (_baudRate) {
  case 19200:
    _bitDelay = 54; break;
  case 9600:
    _bitDelay = 113; break;
  case 4800:
    _bitDelay = 232; break;
  case 2400:
    _bitDelay = 470; break;

  default: */
    _bitDelay = 54;   // 9k6 @ 8MHz, 19k2 @16MHz
//  }    
#if defined(__AVR_ATtiny84__)
  PCMSK0 |= (1<<PCINT0);// tell pin change mask to listen to PA0
  GIMSK  |= (1<<PCIE0); // enable PCINT interrupt in the general interrupt mask
#endif
  whackDelay(_bitDelay*2); // if we were low this establishes the end
}

int ttyIn::read(void)
{
  uint8_t d,i;

  if (! _receive_buffer_index)
    return -1;

  d = _receive_buffer[0]; // grab first byte
  // if we were awesome we would do some nifty queue action
  // sadly, i dont care
  for (i=0; i<_receive_buffer_index; i++) {
    _receive_buffer[i] = _receive_buffer[i+1];
  }
  _receive_buffer_index--;
  return d;
}

uint8_t ttyIn::available(void)
{
  return _receive_buffer_index;
}

