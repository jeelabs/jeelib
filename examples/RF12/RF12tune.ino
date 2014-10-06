/// @dir RF12tune3
/// This sketch loads a configuration into eeprom compatible with rf12_config.
/// It then enters a frequency scanning mode to find the
/// centre of frequency offset from its partner ack'ing Jeenode.
/// The eeprom contents are then updated such that rf12_config will pick up
/// the centre frequency of the acknowledging node as its operational frequency.
/// 2013-10-04 <john<AT>o-hare<DOT>net> http://opensource.org/licenses/mit-license.php
////

#define i  31
#define g  212
#define o  1600
#define x  0
#define q  1
#define c  1


#define MAJOR_VERSION RF12_EEPROM_VERSION // bump when EEPROM layout changes
#define MINOR_VERSION 3                   // bump on other non-trivial changes
#define VERSION "\n[RF12Tune.13]"         // keep in sync with the above

#if defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny44__)
#define TINY        1
#define SERIAL_BAUD    38400  // can only be 9600 or 38400
#undef  LED_PIN               // do not change
#else
#define TINY        0
#define SERIAL_BAUD 57600     // adjust as needed
#endif

/// Save a few bytes of flash by declaring const if used more than once.
const char INITFAIL[] PROGMEM = "init failed\n";

/// Preserve eeprom settings during programming phase.
/// Recommended fuse settings for T84:
/// lfuse reads as C2
/// hfuse reads as D7
/// efuse reads as FF

#include <JeeLib.h>
#include <util/crc16.h>
#include <avr/eeprom.h>
#define ACK_TIME   50  // number of milliseconds to wait for an ack
#define RETRY_LIMIT 5  // maximum number of times to retry
#define RADIO_SYNC_MODE 2
#define SCAN_WIDTH 30

/// Useful url: http://blog.strobotics.com.au/2009/07/27/rfm12-tutorial-part-3a/
//
volatile unsigned long int_count;

// RF12 configuration area
typedef struct {
    byte nodeId;            // used by rf12_config, offset 0
    byte group;             // used by rf12_config, offset 1
    byte format;            // used by rf12_config, offset 2
    byte output :2;         // 0 = dec, 1 = hex, 2 = dec+ascii, 3 = hex+ascii
    byte collect_mode :1;   // 0 = ack, 1 = don't send acks
    byte quiet_mode   :1;   // 0 = show all, 1 = show only valid packets
    byte spare_flags  :4;
    word frequency_offset;  // used by rf12_config, offset 4
    byte RegPaLvl;          // See datasheet RFM69x Register 0x11
    byte pad[RF12_EEPROM_SIZE-9];
    word crc;
} RF12Config;
static RF12Config config;

typedef struct {
signed int afc;
signed int fei;
byte lna;
byte rssi2;
unsigned int offset_TX;
byte RegPaLvl_TX;
byte RegTestLna_TX;
byte RegTestPa1_TX;
byte RegTestPa2_TX;
} observed;
static observed observedRX;

unsigned int frequency_offset;

#if TINY
// Serial support (output only) for Tiny supported by TinyDebugSerial
// http://www.ernstc.dk/arduino/tinycom.html
// 9600, 38400, or 115200
// hardware\jeelabs\avr\cores\tiny\TinyDebugSerial.h Modified to
// move TinyDebugSerial from PB0 to PA3 to match the Jeenode Micro V3 PCB layout
// Connect Tiny84 PA3 (D7) to USB-BUB RXD for serial output from sketch.
// Jeenode AIO2
//
// With thanks for the inspiration by 2006 David A. Mellis and his AFSoftSerial
// code. All right reserved.
// Connect Tiny84 PA2 (D8) to USB-BUB TXD for serial input to sketch.
// Jeenode DIO2
// 9600 or 38400 at present.

#if SERIAL_BAUD == 9600
#define BITDELAY 54          // 9k6 @ 8MHz, 19k2 @16MHz
#endif
#if SERIAL_BAUD == 38400
#define BITDELAY 12          // 28/5/14 from value 11 // 38k4 @ 8MHz, 76k8 @16MHz
#endif

#define _receivePin 8
static char _receive_buffer;
static byte _receive_buffer_index;

ISR (PCINT0_vect) {
    char i, d = 0;
    if (digitalRead(_receivePin))       // PA2 = Jeenode DIO2
        return;                         // not ready!
    whackDelay(BITDELAY - 8);
    for (i=0; i<8; i++) {
        whackDelay(BITDELAY*2 - 6);     // digitalread takes some time
        if (digitalRead(_receivePin))   // PA2 = Jeenode DIO2
            d |= (1 << i);
    }
    whackDelay(BITDELAY*2);
    if (_receive_buffer_index)
        return;
    _receive_buffer = d;                // save data
    _receive_buffer_index = 1;          // got a byte
}

// TODO: replace with code from the std avr libc library:
//  http://www.nongnu.org/avr-libc/user-manual/group__util__delay__basic.html
static void whackDelay (word delay) {
    byte tmp=0;

    asm volatile("sbiw      %0, 0x01 \n\t"
                 "ldi %1, 0xFF \n\t"
                 "cpi %A0, 0xFF \n\t"
                 "cpc %B0, %1 \n\t"
                 "brne .-10 \n\t"
                 : "+r" (delay), "+a" (tmp)
                 : "0" (delay)
                 );
}

static byte inChar () {
    byte d;
    if (! _receive_buffer_index)
        return -1;
    d = _receive_buffer; // grab first and only byte
    _receive_buffer_index = 0;
    return d;
}
#endif

byte h, w, command, enough = 0, loc = 0, newNodeId;
unsigned int value = 0;
byte parameters = 0;

void setup() {
  Serial.begin(SERIAL_BAUD);
  showString(PSTR("\n[RF12tune.0]\n"));
  delay(1000);  // Startup delay to debounce disconnection
} 
void loop() {
  unsigned int scan, upLow, upHigh, downLow, downHigh;
  showString(PSTR("Scanning up started "));
  Serial.print(frequency_offset);
  showString(PSTR(" +/- "));
  Serial.println(SCAN_WIDTH);

    upLow = 0xFFFF;
    upHigh = 0;
    newNodeId=0;
  for (scan = (frequency_offset - SCAN_WIDTH); scan < (frequency_offset + SCAN_WIDTH); ++scan)
  {
   rf12_control(0xA000 + scan); 
   byte good = probe();
   if (good){
     if (scan > upHigh) upHigh = scan;
     if (scan < upLow) upLow = scan;
     if (good != 1) {
       showString(PSTR("\nGood "));
       Serial.print(int(good));
       showString(PSTR(" "));
       Serial.println(int(scan));
     }
     delay(50); 
   }
   else {
     showString(PSTR("No Ack "));
     Serial.print(scan);
     showString(PSTR("\r"));
     delay(50); 
   }
  }
  
  Serial.print("\n");
  Serial.print(int(upLow));
  Serial.print(" ");
  Serial.println(int(upHigh));
  
  if ((upHigh == 0) || (upLow == 0xFFFF)) return;  // If nobody answers then restart loop
  showString(PSTR("Scan up complete "));
  Serial.print(upLow);
  showString(PSTR("-"));
  Serial.println(upHigh);
  delay(50);
  downLow = 0xFFFF; 
  downHigh = 0;
  showString(PSTR("Scanning down started "));
  Serial.print(frequency_offset);
  showString(PSTR(" +/- "));
  Serial.println(SCAN_WIDTH);
  for (scan = (frequency_offset + SCAN_WIDTH); scan > (frequency_offset - SCAN_WIDTH); --scan)
  {
   rf12_control(0xA000 + scan); 
   byte good = probe();
   if (good){
     if (scan > downHigh) downHigh = scan;
     if (scan < downLow) downLow = scan;
     if (good != 1) {
       showString(PSTR("\nGood "));
       Serial.print(int(good));
       showString(PSTR(" "));
       Serial.println(int(scan));
     }
     delay(50); 
   }
   else {
     showString(PSTR("No Ack "));
     Serial.print(scan); 
     showString(PSTR("\r"));
     delay(50);
   }
  }
  
  Serial.print("\n");
  Serial.print(int(downLow));
  Serial.print(" ");
  Serial.println(int(downHigh));
  
  if ((downHigh == 0) || (downLow == 0xFFFF)) return;  // If nobody answers then restart loop
  showString(PSTR("Scan down complete "));
  Serial.print(downLow);
  showString(PSTR("-"));
  Serial.println(downHigh);
        
 frequency_offset = ( ((upLow + downLow) / 2) + ((((upHigh + downHigh) / 2) - ((upLow + downLow)/ 2)) / 2)   );
  showString(PSTR("Centre frequency offset is "));
  Serial.println(frequency_offset);
  delay(50);
  if (newNodeId) {
    config.nodeId = (config.nodeId & 0xE0) + (newNodeId & 0x1F);
    showString(PSTR("\rNew Node Number is "));
    Serial.println(int(newNodeId));
    delay(10);
  }
  setEEProm();
  Serial.println(int_count);
  Serial.println("Delaying 32,767");
  delay(32767);
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
   config.frequency_offset = frequency_offset;

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
    if (!rf12_configSilent()) {
      showString(PSTR("Config save failed"));
    }
    else {
      delay(50);
      byte good = probe(); // Transmit settings
    }
    delay(50);    
  }
  static void saveConfig () {
      config.format = MAJOR_VERSION;
      config.crc = calcCrc(&config, sizeof config - 2);
      eeprom_write_block(&config, RF12_EEPROM_ADDR, sizeof config);

      if (rf12_configSilent())
          rf12_configDump();
      else
        showString(INITFAIL);
  }
static uint16_t calcCrc (const void* ptr, uint8_t len) {
    uint16_t crc = ~0;
    for (byte i = 0; i < len; ++i)
        crc = _crc16_update(crc, ((const byte*) ptr)[i]);
    return crc;
}


static byte probe() 
  {
      for (byte i = 1; i < (RETRY_LIMIT+1); ++i) 
      {
        while (!rf12_canSend())
        rf12_recvDone();
        rf12_sendStart(RF12_HDR_ACK, &config, sizeof config);
        byte acked = waitForAck();
        if (acked) {
          if ((rf12_len == 1) && ((rf12_data[0] & 0xE0) == 0xE0)) { 
            if (!newNodeId) {
              newNodeId = rf12_data[0] & ~0xE0;
              showString(PSTR("Node Allocation "));
              Serial.println(int(newNodeId));
            }
          }
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
              rf12_hdr == (RF12_HDR_DST | RF12_HDR_CTL | (config.nodeId & 0x1F))) {
              showString(PSTR("Ack "));
              Serial.print(int((ACK_TIME - ackTimer.remaining())));
              showString(PSTR("ms  \r"));
             return 1;
              }
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


