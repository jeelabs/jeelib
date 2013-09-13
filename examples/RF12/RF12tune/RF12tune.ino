/// @dir RF12tune
/// Import RF12Demo eeprom string and save to eeprom 
/// typically used with a Jeenode Micro to store a configuration
/// this sketch then enters a frequency scanning mode to find the
/// centre of frequency offset from its partner ack'ing Jeenode.
/// 2013-09-13 <john<AT>o-hare<DOT>net> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>
#include <util/crc16.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>

#if defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny44__)
#define SERIAL_BAUD 38400
#else
#define SERIAL_BAUD 57600
#endif

const char eeprom[] PROGMEM = 
 "B4D4C64054206932302A206732313220403836382E30303030204D487A0A4F4";
void setup() {
  // put your setup code here, to run once:
  Serial.begin(SERIAL_BAUD);
}

void loop() {
  for (byte i = 0; i < RF12_EEPROM_SIZE; ++i) {
    char c = pgm_read_byte(i);
    Serial.print(c);
  }
 Serial.println(); 
 
 delay(1000);
}
