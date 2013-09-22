/// @dir RF12tune
/// Import RF12Demo eeprom string and save to eeprom 
/// typically used with a Jeenode Micro to store a configuration
/// this sketch then enters a frequency scanning mode to find the
/// centre of frequency offset from its partner ack'ing Jeenode.
/// 2013-09-13 <john<AT>o-hare<DOT>net> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>
#include <util/crc16.h>
#include <avr/eeprom.h>
//#include <avr/pgmspace.h>

#if defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny44__)
#define SERIAL_BAUD 38400
#else
#define SERIAL_BAUD 57600
#endif
char config[] = 
///
/// Highlight the string below and paste in the value obtained from the RF12Demo "0j" command.
   "B4D4C64054206932302A206732313220403836382E30303030204D487A00A4F4";
////0....5....10...5....20...5....30...5....40...5....50...5....60..
///
byte h, w;

void setup() {
  
  Serial.begin(SERIAL_BAUD);
  Serial.println("\n[RF12tune.0]");
}

void loop() {
  for (byte i = 0; i < ((RF12_EEPROM_SIZE * 2)); i+=2 ) {
    w = ChkHex(config[i]);
    if (w) h = (w << 4);         // Move into high nibble
    w = ChkHex(config[(i+1)]);
    if (w) h = h + (w & 0x0F);   // Add in low nibble
    showNibble(h >> 4);
    showNibble(h);

//    Serial.print(h, HEX);
    delay(100);
  }
 Serial.println(); 
 
 delay(32767);
}
static char ChkHex(char c) {
  if ((c > 64) && (c < 71)) return (c + 9);    // "A" to "F"
  if ((c > 47) && (c < 58)) return c;          // "0" to "9"
  Serial.print("\nError in config string '");
  Serial.print(c, HEX);
  Serial.println("'");
  return 0;
}
/// showNibble code below pinched from RF12Demo 2013-09-22
static void showNibble (byte nibble) {
  char c = '0' + (nibble & 0x0F);
  if (c > '9')
    c += 7;
  Serial.print(c);
}
///    
