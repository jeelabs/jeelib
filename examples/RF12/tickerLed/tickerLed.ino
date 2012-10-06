/// @dir tickerLed
/// Driver for the Conrad "lichtkrant" unit, i.e. scrolling LED ticker display.
// 2011-10-25 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

#define WAIT '\0'

char buffer [1500];
word next, fill;
byte check;

static void addBuf (char c) {
  buffer[fill++] = c;
  if (fill > sizeof buffer)
    fill = 0; // there is no buffer overflow checking
  check ^=c;
}

static void addBuf (const char* str, char len =-1) {
  if (len < 0)
    len = strlen(str);
  while (--len >= 0)
    addBuf(*str++);
}

static void addMsg (const char* str, char len =-1) {
  addBuf("<ID01>");
  check = 0;
  addBuf(str, len);
  byte v = check;
  addBuf("0123456789ABCDEF"[v>>4]);
  addBuf("0123456789ABCDEF"[v&15]);
  addBuf("<E>");
  addBuf(WAIT);
}

void setup () {
  Serial.begin(9600);
  addBuf("<ID>01<E>"); addBuf(WAIT);
  addMsg("<D*>"); addBuf(WAIT); addBuf(WAIT);
  addMsg("<L1><PA><FE><MA><WC><FE>Hello from JeeLabs");
  addMsg("<RPA>");
  rf12_initialize(17, RF12_868MHZ, 4);
}

void loop () {
  if (next != fill && bitRead(UCSR0A, UDRE0)) {
    char c = buffer[next++]; 
    if (next >= sizeof buffer)
      next = 0;
    if (c != WAIT)
      UDR0 = c;
    else
      delay(100);
    delay(10);
  }
  
  if (rf12_recvDone() && rf12_crc == 0)
    addMsg((const char*) rf12_data, rf12_len);
}
