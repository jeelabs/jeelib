/// @dir RF12tune2
/// This sketch loads a configuration compatible with rf12.cpp and
/// then enters a frequency scanning mode to find the
/// centre of frequency offset from its partner ack'ing Jeenode.
/// 2013-10-04 <john<AT>o-hare<DOT>net> http://opensource.org/licenses/mit-license.php

#define GROUP 209              // Default value 212
#define BAND 128               // 64 = 433Mhz, 128 = 868Mhz, 192 = 915Mhz
#define NODE 9
#define FREQUENCY_OFFSET 1640  // Default value 1600
////
const char NodeDescription[] PROGMEM = 
   "This nodes description123";  
////0....5....10...5....20...5....30...5....40...5....50...5....60..

#include <JeeLib.h>
#include <util/crc16.h>
#include <avr/eeprom.h>
#define ACK_TIME   20  // number of milliseconds to wait for an ack
#define RETRY_LIMIT 9  // maximum number of times to retry
#define RADIO_SYNC_MODE 2

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
#define SERIAL_BAUD 19200
#else
#define SERIAL_BAUD 57600
#endif
byte h, w;

void setup() {
  delay(5000);  // Startup delay to debounce disconnection
  Serial.begin(SERIAL_BAUD);
  showString(PSTR("\n[RF12tune2.0] \n"));
  
  config.nodeId = BAND + NODE;
  config.group = GROUP;
  frequency_offset = FREQUENCY_OFFSET;
  if (sizeof NodeDescription <= RF12_EEPROM_SIZE-6)
  {
    for (byte i = 0; i < sizeof NodeDescription ; i++ ) {
      config.msg[i] = getString(NodeDescription, i);
    }
  }
  setEEProm();  
}

void loop() {
  unsigned int scan, upLow, upHigh, downLow, downHigh;
  showString(PSTR("Scanning started "));
  Serial.println(frequency_offset);
  delay(50); 

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
  setEEProm();
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
static void setEEProm()
  {
   config.ee_frequency_hi = frequency_offset >> 8;
   config.ee_frequency_lo = frequency_offset & 0x00FF;

    config.crc = ~0;
    for (byte i = 0; i < sizeof config - 2; ++i)
      config.crc = _crc16_update(config.crc, ((byte*) &config)[i]);
    // save to EEPROM
    for (byte i = 0; i < sizeof config; ++i) {
      byte b = ((byte*) &config)[i];
      eeprom_write_byte(RF12_EEPROM_ADDR + i, b);
      showNibble(b >> 4);
      showNibble(b);
    }
    showString(PSTR("\n"));
    delay(50);
    if (!rf12_config()) {
      showString(PSTR("Config save failed"));
    }
    else {
      delay(50);
      byte good = probe(); // Transmit new settings
    }
    delay(50);    
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
/// showNibble code below pinched from RF12Demo 2013-09-22
static void showNibble (byte nibble) {
  char c = '0' + (nibble & 0x0F);
  if (c > '9')
    c += 7;
  Serial.print(c);
}

