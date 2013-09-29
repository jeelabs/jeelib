/// @dir RF12tune
/// Import RF12Demo eeprom string and save to eeprom 
/// typically used with a Jeenode Micro to store a configuration
/// this sketch then enters a frequency scanning mode to find the
/// centre of frequency offset from its partner ack'ing Jeenode.
/// 2013-09-13 <john<AT>o-hare<DOT>net> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>
#include <util/crc16.h>
#include <avr/eeprom.h>
#define ACK_TIME   20  // number of milliseconds to wait for an ack
#define RETRY_LIMIT 9  // maximum number of times to retry
#define RADIO_SYNC_MODE 2

char importedConfig[] = 
///
/// Highlight the string below and paste in the value obtained from the RF12Demo "0j" command.
   "B4D4C66D54206932302A206732313220403836382E32323530204D487A00690C";  // C T i20* g212 @868.2250 MHz
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
unsigned int frequency;


#if defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny44__)
#define SERIAL_BAUD 38400
#else
#define SERIAL_BAUD 57600
#endif
byte h, w;

void setup() {
  
  Serial.begin(SERIAL_BAUD);
  Serial.println("\n[RF12tune.0]");
}

void loop() {
  unsigned int scan;
  for (byte i = 0; i < (RF12_EEPROM_SIZE * 2); i+=2 ) {
    w = ChkHex(importedConfig[i]);
    if (w) h = (w << 4);         // Move into high nibble
    w = ChkHex(importedConfig[(i+1)]);
    if (w) h = h + (w & 0x0F);   // Add in low nibble
    showNibble(h >> 4);
    showNibble(h);
    eeprom_write_byte((RF12_EEPROM_ADDR) + (i / 2), h);
    ((byte*) &config)[(i/2)] = h;
//    Serial.print(h, HEX);
  }
 Serial.println(); 
// Serial.println(config.crc, HEX);
 
///   for (byte i = 0; i < sizeof RF12_EEPROM_SIZE - 2; ++i)
///    config.crc = _crc16_update(config.crc, ((byte*) &config)[i]);

//    uint16_t crc = ~0;
//    for (uint8_t i = 0; i < RF12_EEPROM_SIZE; ++i) {
//        byte e = ((byte*) &config)[i]);
//        crc = _crc16_update(crc, e);//
//    }
//    if (crc != 0) Serial.println("Bad CRC");

    
    Serial.println(config.msg);
    showNibble(config.crc >> 12);
    showNibble(config.crc >> 8);
    showNibble(config.crc >> 4);
    showNibble(config.crc);
    
    
    if (rf12_config()) Serial.println("Config Initialized");
     
    frequency = (config.ee_frequency_hi << 8) + config.ee_frequency_lo;              // Loose flag nibble to get frequency high order
  for (scan = (frequency - 50); scan < (frequency + 50); ++scan)
  {
   rf12_control(0xA000 + scan); 
   Serial.print("Sending "); 
   Serial.println(scan);
   delay(50); 
   byte acked = probe();
   if (acked){
     Serial.print("Received "); 
     Serial.println(scan);
   }
      else {
        Serial.print("No Ack ");
        Serial.println(scan);
      }
  }   
 delay(32767);
}

static byte probe() 
  {
      for (byte i = 0; i < RETRY_LIMIT; ++i) 
      {
        while (!rf12_canSend())
        rf12_recvDone();
        rf12_sendStart(RF12_HDR_ACK, &config, sizeof config, RADIO_SYNC_MODE);
        byte acked = waitForAck();
        if (acked) {
          return 1;
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
  Serial.print("\nError in importedConfig string '");
  Serial.print(c);
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
