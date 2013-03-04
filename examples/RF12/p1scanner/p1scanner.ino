/// @dir p1scanner
/// Parse P1 data from smart meter and send as compressed packet over RF12.
/// @see http://jeelabs.org/2013/01/02/encoding-p1-data/
// 2012-12-31 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>
#include <SoftwareSerial.h>

#define DEBUG 0   // set to 1 to use fake data instead of SoftwareSerial
#define LED   9   // set to 0 to disable LED blinking

const char* data = 
"/XMX5XMXABCE000046099\n"
"\n"
"0-0:96.1.1(30313337323430332020202020202020)\n"
"1-0:1.8.1(00003.540*kWh)\n"
"1-0:1.8.2(00011.199*kWh)\n"
"1-0:2.8.1(00000.000*kWh)\n"
"1-0:2.8.2(00004.667*kWh)\n"
"0-0:96.14.0(0002)\n"
"1-0:1.7.0(0000.35*kW)\n"
"1-0:2.7.0(0000.00*kW)\n"
"0-0:17.0.0(999*A)\n"
"0-0:96.3.10(1)\n"
"0-0:96.13.1()\n"
"0-0:96.13.0()\n"
"0-1:96.1.0(3131323838323030303336383037303132)\n"
"0-1:24.1.0(03)\n"
"0-1:24.3.0(121129160000)(00)(60)(1)(0-1:24.2.0)(m3)\n"
"(00014.684)\n"
"0-1:24.4.0(2)\n"
"!\n"
;

#define FORMAT 1
#define NTYPES (sizeof typeMap / sizeof *typeMap)

// list of codes to be sent out (only compares lower byte!)
const byte typeMap [] = {
  181, 182, 281, 282, 96140, 170, 270, 2410, 2420, 2440
};

SoftwareSerial mySerial (7, 17, true); // inverted logic

byte type;
uint32_t value;
uint32_t readings[NTYPES+1];
byte payload[5*NTYPES], *fill;

static bool p1_scanner (char c) {
  switch (c) {
    case ':':
      type = 0;
      value = 0;
      break;
    case '(':
      if (type == 0)
        type = value; // truncates to lower byte
      value = 0;
    case '.':
      break;
    case ')':
      if (type)
        return true;
      break;
    default:
      if ('0' <= c && c <= '9')
        value = 10 * value + (c - '0');
  }
  return false;
}

static void addValue (uint32_t v, byte mask =0x80) {
  uint32_t w = v >> 7;
  if (w)
    addValue(w, 0);
  *fill++ = (v & 0x7F) | mask;
}

static void collectData (bool empty =false) {
  fill = payload;
  addValue(FORMAT);
  if (!empty)
    for (byte i = 0; i < NTYPES; ++i)
      addValue(readings[i]);
  byte n = fill - payload;
  
  if (DEBUG) {
    for (byte i = 0; i < n; ++i) {
      Serial.print(' ');
      Serial.print(payload[i], HEX);
    }
    Serial.println();
  }
  
  rf12_sendNow(0, payload, n);
  rf12_sendWait(1);
}

void setup () {
  if (DEBUG) {
    Serial.begin(57600);
    Serial.println("\n[p1scanner]");
  }
  mySerial.begin(9600);
  // digitalWrite(7, 1); // enable pull-up

  if (LED) {
    pinMode(LED, OUTPUT);
    digitalWrite(LED, 1);
  }

  rf12_initialize(18, RF12_868MHZ, 5);
  collectData(true); // empty packet on power-up
}

void loop () {
  int c;
  if (DEBUG) {
    c = *data;
    if (c)
      ++data;
  } else {
    c = mySerial.read();
    if (c > 0) {
      c &= 0x7F;
      // Serial.write(c);
    }
  }
  switch (c) {
    case '/':
    if (LED)
        digitalWrite(LED, 0); // LED on
      break;
    case '!':
      collectData();
      memset(readings, 0, sizeof readings);
      if (LED)
        digitalWrite(LED, 1); // LED off
      break;
    default:
      if (p1_scanner(c)) {
        if (DEBUG) {
          Serial.print(type);
          Serial.print('=');
          Serial.println(value);
        }
        for (byte i = 0; i < NTYPES; ++i)
          if (type == typeMap[i]) {
            readings[i] = value;
            break;
          }
    }
  }
}
