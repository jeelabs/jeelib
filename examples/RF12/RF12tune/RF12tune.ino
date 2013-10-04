/// @dir RF12tune
/// Import RF12Demo eeprom string and save to eeprom 
/// typically used with a Jeenode Micro to store a configuration
/// this sketch then enters a frequency scanning mode to find the
/// centre of frequency offset from its partner ack'ing Jeenode.
/// 2013-09-13 <john<AT>o-hare<DOT>net> http://opensource.org/licenses/mit-license.php


#define GROUP 212
#define NODE 9

#include <JeeLib.h>
#include <util/crc16.h>
#include <avr/eeprom.h>
#define ACK_TIME   20  // number of milliseconds to wait for an ack
#define RETRY_LIMIT 9  // maximum number of times to retry
#define RADIO_SYNC_MODE 2

const char importedConfig[] PROGMEM = 
///
/// Highlight the string below and paste in the value obtained from the RF12Demo "0j" command.
   "89D1066D49206939206732303920403836382E32323530204D487A0000006893";  //   0 I i9 g209 @868.2250 MHz
//   "B4D4C66D54206932302A206732313220403836382E32323530204D487A00690C";  // C T i20* g212 @868.2250 MHz
//   "B4D4C64054206932302A206732313220403836382E30303030204D487A00A4F4";  // C T i20* g212 @868.0000 MHz
//   "94D4C6405420693230206732313220403836382E30303030204D487A0000FCE4";  // C T i20 g212 @868.0000 MHz
////0....5....10...5....20...5....30...5....40...5....50...5....60..
///

/// @details
/// eeprom layout details
/// byte 0x00 Key storage for encryption algorithm
///      0x1F   "
/// ------------------------------------------------------------------------
/// byte 0x20 Node number in bits                   ***n nnnn                    // 1 - 31
///           Collect mode flag                     **0* ****   COLLECT 0x20     // Pass incoming without sending acks
///           Band                                  00** ****   Do not use       // Will hang the hardware
///             "                                   01** ****   433MHZ  0x40
///             "                                   10** ****   868MHZ  0x80
///             "                                   11** ****   915MHZ  0xC0
/// ------------------------------------------------------------------------
/// byte 0x21 Group number                                11010100    // i.e. 212 0xD4
/// byte 0x22 Flag Spares                                 11** ****   // Perhaps we could store the output in hex flag here
///           V10 indicator                               **1* ****   // This bit is set by versions of RF12Demo less than 11
///           Quiet mode                                  ***1 ****   // don't report bad packets
///           Frequency offset most significant bite      **** nnnn   // Can't treat as a 12 bit integer
/// byte 0x23 Frequency offset less significant bits      nnnn nnnn   //  because of little endian constraint
/// byte 0x24 Text description generate by RF12Demo       "T i20 g0 @868.0000 MHz"
///      0x3D   "                                         Padded at the end with NUL
/// byte 0x3E  CRC                                        CRC of values with offset 0x20
/// byte 0x3F   "                                         through to end of Text string, except NUL's
/// byte 0x40 32 bytes backup space for configuration, "42j" command
///      0x59   "
/// ------------------------------------------------------------------------
/// Useful url: http://blog.strobotics.com.au/2009/07/27/rfm12-tutorial-part-3a/
//
// RF12 configuration setup code
typedef struct {
  byte nodeId;
  byte group;
  int ee_frequency_hi : 4;  // Can't use as a 12 bit integer because of how they are stored in a structure.
  boolean flags : 4;
  int ee_frequency_lo : 8;  //
  char msg[RF12_EEPROM_SIZE-6];
  word crc;
} RF12Config;
//
static RF12Config config;
unsigned int frequency_offset;


#if defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny44__)
#define SERIAL_BAUD 38400
#else
#define SERIAL_BAUD 57600
#endif
byte h, w;

void setup() {
  delay(5000);  // Startup delay to debounce disconnection
  Serial.begin(SERIAL_BAUD);
  showString(PSTR("\n[RF12tune.0] \n\r"));
  
  for (byte i = 0; i < (RF12_EEPROM_SIZE * 2); i+=2 ) {
    w = ChkHex(getString (importedConfig, i));
    if (w) h = (w << 4);         // Move into high nibble
    w = ChkHex(getString (importedConfig, i+1));
    if (w) h = h + (w & 0x0F);   // Add in low nibble
    showNibble(h >> 4);
    showNibble(h);
//    eeprom_write_byte((RF12_EEPROM_ADDR) + (i / 2), h);
    ((byte*) &config)[(i/2)] = h;
  }
  showString(PSTR("\n\r"));

    frequency_offset = (config.ee_frequency_hi << 8) + config.ee_frequency_lo;              // Loose flag nibble to get frequency high order    
}

void loop() {
  unsigned int scan, upLow, upHigh, downLow, downHigh;
    frequency_offset = eeprom_read_byte(RF12_EEPROM_ADDR + 2);
    frequency_offset = ((frequency_offset & 0x0F)  << 8) + (eeprom_read_byte(RF12_EEPROM_ADDR + 3));              // Loose flag nibble to get frequency high order
    if (rf12_config()) {
//      Serial.print("Config Initialized ");
      showString(PSTR("Config Initialized "));
      Serial.println(frequency_offset);
      delay(50); 
    }
    upLow = 0xFFFF;
    upHigh = 0;
  for (scan = (frequency_offset - 50); scan < (frequency_offset + 50); ++scan)
  {
   rf12_control(0xA000 + scan); 
   byte good = probe();
   if (good){
     if (scan > upHigh) upHigh = scan;
     if (scan < upLow) upLow = scan;
     delay(50); 
   }
      else {
        showString(PSTR("No Ack "));
        Serial.print(scan);
        showString(PSTR("\r"));
        delay(50); 
      }
  }
  if ((upHigh == 0) || (upLow == 0xFFFF)) return;  // If nobody answers then restart loop
    showString(PSTR("Scan up complete "));
    Serial.print(upLow);
    showString(PSTR("-"));
    Serial.println(upHigh);
    delay(100);
    downLow = 0xFFFF; 
    downHigh = 0;
  for (scan = (frequency_offset + 50); scan > (frequency_offset - 50); --scan)
  {
   rf12_control(0xA000 + scan); 
   byte good = probe();
   if (good){
     if (scan > downHigh) downHigh = scan;
     if (scan < downLow) downLow = scan;
     delay(50); 
   }
      else {
        showString(PSTR("No Ack "));
        Serial.print(scan);
        showString(PSTR("\r"));
        delay(50); 
      }
  }
  if ((downHigh == 0) || (downLow == 0xFFFF)) return;  // If nobody answers then restart loop
  showString(PSTR("Scan down complete "));
  Serial.print(downLow);
  showString(PSTR("-"));
  Serial.println(downHigh);

        
 frequency_offset = ( ((upLow + downLow) / 2) + ((((upHigh + downHigh) / 2) - ((upLow + downLow)/ 2)) / 2)   );
  showString(PSTR("Centre frequency offset is "));
  Serial.println(frequency_offset);
  delay(50);
 config.ee_frequency_hi = frequency_offset >> 8;
 config.ee_frequency_lo = frequency_offset & 0x00FF;

  config.crc = ~0;
  for (byte i = 0; i < sizeof config - 2; ++i)
    config.crc = _crc16_update(config.crc, ((byte*) &config)[i]);

  // save to EEPROM
  for (byte i = 0; i < sizeof config; ++i) {
    byte b = ((byte*) &config)[i];
    eeprom_write_byte(RF12_EEPROM_ADDR + i, b);
  }
  if (!rf12_config()) {
    showString(PSTR("config save failed"));
//    Serial.println("config save failed");
  }
  else {
    delay(50);
    byte good = probe(); // Transmit new settings
  }
  while(1) // Nothing more
  { 
     delay(32767);
  }
}
static byte getString (PGM_P s, byte i) {
    char c = pgm_read_byte(s + i);
    return c;
}
static void showString (PGM_P s) {
  for (;;) {
    char c = pgm_read_byte(s++);
    if (c == 0)
      break;
    if (c == '\n')
      Serial.print('\r');
    Serial.print(c);
  }
}

static byte probe() 
  {
      for (byte i = 1; i < (RETRY_LIMIT+1); ++i) 
      {
        while (!rf12_canSend())
        rf12_recvDone();
        rf12_sendStart(RF12_HDR_ACK, &config, sizeof config, RADIO_SYNC_MODE);
        byte acked = waitForAck();
        if (acked) {
          return i; // Return number of attempts to successfully transmit
        }
      }
    return 0;
  }

// wait a few milliseconds for proper ACK to me, return true if indeed received
static byte waitForAck() {
    MilliTimer ackTimer;
    while (!ackTimer.poll(ACK_TIME)) {
        if (rf12_recvDone() && rf12_crc == 0 &&
                // see http://talk.jeelabs.net/topic/811#post-4712
                rf12_hdr == (RF12_HDR_DST | RF12_HDR_CTL | (config.nodeId & 0x1F)))
            return 1;
    }
    return 0;
}
static char ChkHex(char c) {
  if ((c > 64) && (c < 71)) return (c + 9);    // "A" to "F"
  if ((c > 47) && (c < 58)) return c;          // "0" to "9"
    showString(PSTR("\nError in importedConfig string '"));
  Serial.print(c);
  showString(PSTR("'\n\r"));
  return 0;
}
/// showNibble code below pinched from RF12Demo 2013-09-22
static void showNibble (byte nibble) {
  char c = '0' + (nibble & 0x0F);
  if (c > '9')
    c += 7;
  Serial.print(c);
}

