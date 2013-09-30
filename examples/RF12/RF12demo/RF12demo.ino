/// @dir RF12demo
/// Configure some values in EEPROM for easy config of the RF12 later on.
// 2009-05-06 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

// this version adds flash memory support, 2009-11-19
// Adding frequency features. 2013-09-05 JohnOH
#include <JeeLib.h>
#include <util/crc16.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <util/parity.h>

#define debug
// ATtiny's only support outbound serial @ 38400 baud, and no DataFlash logging

#if defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny44__)
#define SERIAL_BAUD 38400
#define JNMicro
#else
#define SERIAL_BAUD 57600

#define DATAFLASH 0          // Enabling flash for none flash equipped nodes causes odd problems - particularly non-Jee kit
// check for presence of DataFlash memory on JeeLink
#define FLASH_MBIT  16  // support for various dataflash sizes: 4/8/16 Mbit

#define LED_PIN   9 // activity LED, comment out to disable

#endif

static unsigned long now () {
  // FIXME 49-day overflow
  return millis() / 1000;
}

static void activityLed (byte on) {
#ifdef LED_PIN
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, !on);
#endif
}
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
// 4 bit
#define QUIET   0x1      // quiet mode
#define V10     0x2      // Indicates a version of RF12Demo after version 10.
// ----------------
// 8 bit
#define COLLECT 0x20     // collect mode, i.e. pass incoming without sending acks

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

unsigned int frequency;
static RF12Config config;
char revP = 94; // Symbol ^ to indicate direction of frequency offset
static char cmd;
static byte value, stack[RF12_MAXDATA+4], top, sendLen, dest, sticky, revF = 0;
static byte testbuf[RF12_MAXDATA], testCounter, useHex;
static byte band;

void displayVersion(uint8_t newline );

static void showNibble (byte nibble) {
  char c = '0' + (nibble & 0x0F);
  if (c > '9')
    c += 7;
  Serial.print(c);
}

static void showByte (byte value) {
  if (useHex) {
    showNibble(value >> 4);
    showNibble(value);
  } else
    Serial.print((int) value);
}

static void addCh (char* msg, char c) {
  byte n = strlen(msg);
  msg[n] = c;
}

static void addInt (char* msg, word v) {
  if (v >= 10)
    addInt(msg, v / 10);
  addCh(msg, '0' + v % 10);
}

static void saveConfig () {
  // set up a nice config string to be shown on startup
  memset(config.msg, 0, sizeof config.msg);
  config.flags  &= ~V10;               // Indicate v11 and upwards, unset the eeprom+2 0x20 bit !
  config.ee_frequency_hi = frequency >> 8;
  config.ee_frequency_lo = frequency & 0x00FF;
  byte id = config.nodeId & 0x1F;
  addCh(config.msg, '@' + id);
  strcat(config.msg, " i");
  addInt(config.msg, id);
  if (config.nodeId & COLLECT)
    addCh(config.msg, '*');
  
  strcat(config.msg, " g");
  addInt(config.msg, config.group);
  
  strcat(config.msg, " @");
  static word bands[4] = { 0, 430, 860, 900 }; // 315, 433, 864, 915 Mhz    
  band = config.nodeId >> 6;
  long wk = frequency;                                        // 96 - 3903 is the range of values supported by the RFM12B
  wk = wk * (band * 25);                                             // Freqency changes larger in higher bands
  long characteristic = wk/10000;
  addInt(config.msg, characteristic + bands[band]);
  byte pos = strlen(config.msg);
  addInt(config.msg, ((10000 + (wk - (characteristic * 10000)))));; // Adding 10,000 the digit protects the leading zeros
  config.msg[pos] = '.';                                            // Loose the 10,000 digit
  strcat(config.msg, " MHz");
  
  config.crc = ~0;
  for (byte i = 0; i < sizeof config - 2; ++i)
    config.crc = _crc16_update(config.crc, ((byte*) &config)[i]);

  // save to EEPROM
  for (byte i = 0; i < sizeof config; ++i) {
    byte b = ((byte*) &config)[i];
    eeprom_write_byte(RF12_EEPROM_ADDR + i, b);
  }
  if (!rf12_config())
    Serial.println("config save failed");
}

static byte bandToFreq (byte band) {
   return band == 4 ? RF12_433MHZ : band == 8 ? RF12_868MHZ : band == 9 ? RF12_915MHZ : 0;
}

#if not defined(__AVR_ATtiny84__) || not defined(__AVR_ATtiny44__)
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// OOK transmit code

// Turn transmitter on or off, but also apply asymmetric correction and account
// for 25 us SPI overhead to end up with the proper on-the-air pulse widths.
// With thanks to JGJ Veken for his help in getting these values right.
static void ookPulse(int on, int off) {
  rf12_onOff(1);
  delayMicroseconds(on + 150);
  rf12_onOff(0);
  delayMicroseconds(off - 200);
}

static void fs20sendBits(word data, byte bits) {
  if (bits == 8) {
    ++bits;
    data = (data << 1) | parity_even_bit(data);
  }
  for (word mask = bit(bits-1); mask != 0; mask >>= 1) {
    int width = data & mask ? 600 : 400;
    ookPulse(width, width);
  }
}

static void fs20cmd(word house, byte addr, byte cmd) {
  byte sum = 6 + (house >> 8) + house + addr + cmd;
  for (byte i = 0; i < 3; ++i) {
    fs20sendBits(1, 13);
    fs20sendBits(house >> 8, 8);
    fs20sendBits(house, 8);
    fs20sendBits(addr, 8);
    fs20sendBits(cmd, 8);
    fs20sendBits(sum, 8);
    fs20sendBits(0, 1);
    delay(10);
  }
}

static void kakuSend(char addr, byte device, byte on) {
  int cmd = 0x600 | ((device - 1) << 4) | ((addr - 1) & 0xF);
  if (on)
    cmd |= 0x800;
  for (byte i = 0; i < 4; ++i) {
    for (byte bit = 0; bit < 12; ++bit) {
      ookPulse(375, 1125);
      int on = bitRead(cmd, bit) ? 1125 : 375;
      ookPulse(on, 1500 - on);
    }
    ookPulse(375, 375);
    delay(11); // approximate
  }
}
#endif
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// DataFlash code

#if DATAFLASH

#define DF_ENABLE_PIN 8     // PB0

#if FLASH_MBIT == 4
// settings for 0.5 Mbyte flash in JLv2
#define DF_BLOCK_SIZE 16      // number of pages erased at same time
#define DF_LOG_BEGIN  32      // first 2 blocks reserved for future use
#define DF_LOG_LIMIT  0x0700    // last 64k is not used for logging
#define DF_MEM_TOTAL  0x0800    // 2048 pages, i.e. 0.5 Mbyte
#define DF_DEVICE_ID  0x1F44    // see AT25DF041A datasheet
#define DF_PAGE_ERASE 0x20    // erase one block of flash memory
#endif

#if FLASH_MBIT == 8
// settings for 1 Mbyte flash in JLv2
#define DF_BLOCK_SIZE 16      // number of pages erased at same time
#define DF_LOG_BEGIN  32      // first 2 blocks reserved for future use
#define DF_LOG_LIMIT  0x0F00    // last 64k is not used for logging
#define DF_MEM_TOTAL  0x1000    // 4096 pages, i.e. 1 Mbyte
#define DF_DEVICE_ID  0x1F45    // see AT26DF081A datasheet
#define DF_PAGE_ERASE 0x20    // erase one block of flash memory
#endif

#if FLASH_MBIT == 16
// settings for 2 Mbyte flash in JLv3
#define DF_BLOCK_SIZE 256     // number of pages erased at same time
#define DF_LOG_BEGIN  512     // first 2 blocks reserved for future use
#define DF_LOG_LIMIT  0x1F00    // last 64k is not used for logging
#define DF_MEM_TOTAL  0x2000    // 8192 pages, i.e. 2 Mbyte
#define DF_DEVICE_ID  0x2020    // see M25P16 datasheet
#define DF_PAGE_ERASE 0xD8    // erase one block of flash memory
#endif

// structure of each page in the log buffer, size must be exactly 256 bytes
typedef struct {
  byte data [248];
  word seqnum;
  long timestamp;
  word crc;
} FlashPage;

// structure of consecutive entries in the data area of each FlashPage
typedef struct {
  byte length;
  byte offset;
  byte header;
  byte data[RF12_MAXDATA];
} FlashEntry;

static FlashPage dfBuf;   // for data not yet written to flash
static word dfLastPage;   // page number last written
static byte dfFill;     // next byte available in buffer to store entries

static byte df_present () {
  return dfLastPage != 0;
}

static void df_enable () {
  // digitalWrite(ENABLE_PIN, 0);
  bitClear(PORTB, 0);
}

static void df_disable () {
  // digitalWrite(ENABLE_PIN, 1);
  bitSet(PORTB, 0);
}

static byte df_xfer (byte cmd) {
  SPDR = cmd;
  while (!bitRead(SPSR, SPIF))
    ;
  return SPDR;
}

void df_command (byte cmd) {
  for (;;) {
    cli();
    df_enable();
    df_xfer(0x05); // Read Status Register
    byte status = df_xfer(0);
    df_disable();
    sei();
    // don't wait for ready bit if there is clearly no dataflash connected
    if (status == 0xFF || (status & 1) == 0)
      break;
  }  

  cli();
  df_enable();
  df_xfer(cmd);
}

static void df_deselect () {
  df_disable();
  sei();
}

static void df_writeCmd (byte cmd) {
  df_command(0x06); // Write Enable
  df_deselect();
  df_command(cmd);
}

void df_read (word block, word off, void* buf, word len) {
  df_command(0x03); // Read Array (Low Frequency)
  df_xfer(block >> 8);
  df_xfer(block);
  df_xfer(off);
  for (word i = 0; i < len; ++i)
    ((byte*) buf)[(byte) i] = df_xfer(0);
  df_deselect();
}

void df_write (word block, const void* buf) {
  df_writeCmd(0x02); // Byte/Page Program
  df_xfer(block >> 8);
  df_xfer(block);
  df_xfer(0);
  for (word i = 0; i < 256; ++i)
    df_xfer(((const byte*) buf)[(byte) i]);
  df_deselect();
}

// wait for current command to complete
void df_flush () {
  df_read(0, 0, 0, 0);
}

static void df_wipe () {
  Serial.println("DF W");
  
  df_writeCmd(0xC7); // Chip Erase
  df_deselect();
  df_flush();
}

static void df_erase (word block) {
  Serial.print("DF E ");
  Serial.println(block);
  
  df_writeCmd(DF_PAGE_ERASE); // Block Erase
  df_xfer(block >> 8);
  df_xfer(block);
  df_xfer(0);
  df_deselect();
  df_flush();
}

static word df_wrap (word page) {
  return page < DF_LOG_LIMIT ? page : DF_LOG_BEGIN;
}

static void df_saveBuf () {
  if (dfFill == 0)
    return;

  dfLastPage = df_wrap(dfLastPage + 1);
  if (dfLastPage == DF_LOG_BEGIN)
    ++dfBuf.seqnum; // bump to next seqnum when wrapping
  
  // set remainder of buffer data to 0xFF and calculate crc over entire buffer
  dfBuf.crc = ~0;
  for (byte i = 0; i < sizeof dfBuf - 2; ++i) {
    if (dfFill <= i && i < sizeof dfBuf.data)
      dfBuf.data[i] = 0xFF;
    dfBuf.crc = _crc16_update(dfBuf.crc, dfBuf.data[i]);
  }
  
  df_write(dfLastPage, &dfBuf);
  dfFill = 0;
  
  // wait for write to finish before reporting page, seqnum, and time stamp
  df_flush();
  Serial.print("DF S ");
  Serial.print(dfLastPage);
  Serial.print(' ');
  Serial.print(dfBuf.seqnum);
  Serial.print(' ');
  Serial.println(dfBuf.timestamp);
  
  // erase next block if we just saved data into a fresh block
  // at this point in time dfBuf is empty, so a lengthy erase cycle is ok
  if (dfLastPage % DF_BLOCK_SIZE == 0)
    df_erase(df_wrap(dfLastPage + DF_BLOCK_SIZE));
}

static void df_append (const void* buf, byte len) {
  //FIXME the current logic can't append incoming packets during a save!

  // fill in page time stamp when appending to a fresh page
  if (dfFill == 0)
    dfBuf.timestamp = now();
  
  long offset = now() - dfBuf.timestamp;
  if (offset >= 255 || dfFill + 1 + len > sizeof dfBuf.data) {
    df_saveBuf();

    dfBuf.timestamp = now();
    offset = 0;
  }

  // append new entry to flash buffer
  dfBuf.data[dfFill++] = offset;
  memcpy(dfBuf.data + dfFill, buf, len);
  dfFill += len;
}

// go through entire log buffer to figure out which page was last saved
static void scanForLastSave () {
  dfBuf.seqnum = 0;
  dfLastPage = DF_LOG_LIMIT - 1;
  // look for last page before an empty page
  for (word page = DF_LOG_BEGIN; page < DF_LOG_LIMIT; ++page) {
    word currseq;
    df_read(page, sizeof dfBuf.data, &currseq, sizeof currseq);
    if (currseq != 0xFFFF) {
      dfLastPage = page;
      dfBuf.seqnum = currseq + 1;
    } else if (dfLastPage == page - 1)
      break; // careful with empty-filled-empty case, i.e. after wrap
  }
}

static void df_initialize () {
  // assumes SPI has already been initialized for the RFM12B
  df_disable();
  pinMode(DF_ENABLE_PIN, OUTPUT);
  df_command(0x9F); // Read Manufacturer and Device ID
  word info = df_xfer(0) << 8;
  info |= df_xfer(0);
  df_deselect();

  if (info == DF_DEVICE_ID) {
    df_writeCmd(0x01);  // Write Status Register ...
    df_xfer(0);     // ... Global Unprotect
    df_deselect();

    scanForLastSave();
    
    Serial.print("DF I ");
    Serial.print(dfLastPage);
    Serial.print(' ');
    Serial.println(dfBuf.seqnum);
  
    // df_wipe();
    df_saveBuf(); //XXX
  }
}

static void discardInput () {
  while (Serial.read() >= 0)
    ;
}

static void df_dump () {
  struct { word seqnum; long timestamp; word crc; } curr;
  discardInput();
  for (word page = DF_LOG_BEGIN; page < DF_LOG_LIMIT; ++page) {
    if (Serial.read() >= 0)
      break;
    // read marker from page in flash
    df_read(page, sizeof dfBuf.data, &curr, sizeof curr);
    if (curr.seqnum == 0xFFFF)
      continue; // page never written to
    Serial.print(" df# ");
    Serial.print(page);
    Serial.print(" : ");
    Serial.print(curr.seqnum);
    Serial.print(' ');
    Serial.print(curr.timestamp);
    Serial.print(' ');
    Serial.println(curr.crc);
  }
}

static word scanForMarker (word seqnum, long asof) {
  word lastPage = 0;
  struct { word seqnum; long timestamp; } last, curr;
  last.seqnum = 0xFFFF;
  // go through all the pages in log area of flash
  for (word page = DF_LOG_BEGIN; page < DF_LOG_LIMIT; ++page) {
    // read seqnum and timestamp from page in flash
    df_read(page, sizeof dfBuf.data, &curr, sizeof curr);
    if (curr.seqnum == 0xFFFF)
      continue; // page never written to
    if (curr.seqnum >= seqnum && curr.seqnum < last.seqnum) {
      last = curr;
      lastPage = page;
    }
    if (curr.seqnum == last.seqnum && curr.timestamp <= asof)
      lastPage = page;
  }
  return lastPage;
}

static void df_replay (word seqnum, long asof) {
  word page = scanForMarker(seqnum, asof);
  Serial.print("r: page ");
  Serial.print(page);
  Serial.print(' ');
  Serial.println(dfLastPage);
  discardInput();
  word savedSeqnum = dfBuf.seqnum;
  while (page != dfLastPage) {
    if (Serial.read() >= 0)
      break;
    page = df_wrap(page + 1);
    df_read(page, 0, &dfBuf, sizeof dfBuf); // overwrites ram buffer!
    if (dfBuf.seqnum == 0xFFFF)
      continue; // page never written to
    // skip and report bad pages
    word crc = ~0;
    for (word i = 0; i < sizeof dfBuf; ++i)
      crc = _crc16_update(crc, dfBuf.data[i]);
    if (crc != 0) {
      Serial.print("DF C? ");
      Serial.print(page);
      Serial.print(' ');
      Serial.println(crc);
      continue;
    }
    // report each entry as "R seqnum time <data...>"
    byte i = 0;
    while (i < sizeof dfBuf.data && dfBuf.data[i] < 255) {
      if (Serial.available())
        break;
      Serial.print("R ");
      Serial.print(dfBuf.seqnum);
      Serial.print(' ');
      Serial.print(dfBuf.timestamp + dfBuf.data[i++]);
      Serial.print(' ');
      Serial.print((int) dfBuf.data[i++]);
      byte n = dfBuf.data[i++];
      while (n-- > 0) {
        Serial.print(' ');
        Serial.print((int) dfBuf.data[i++]);
      }
      Serial.println();
    }
    // at end of each page, report a "DF R" marker, to allow re-starting
    Serial.print("DF R ");
    Serial.print(page);
    Serial.print(' ');
    Serial.print(dfBuf.seqnum);
    Serial.print(' ');
    Serial.println(dfBuf.timestamp);
  }
  dfFill = 0; // ram buffer is no longer valid
  dfBuf.seqnum = savedSeqnum + 1; // so next replay will start at a new value
  Serial.print("DF E ");
  Serial.print(dfLastPage);
  Serial.print(' ');
  Serial.print(dfBuf.seqnum);
  Serial.print(' ');
  Serial.println(millis());
}

#else // DATAFLASH

#define df_present() 0
#define df_initialize()
#define df_dump()
#define df_replay(x,y)
#define df_erase(x)

#endif 

#if not defined(__AVR_ATtiny84__) || not defined(__AVR_ATtiny44__)
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

const char helpText1[] PROGMEM = 
  "\n"
  "Available commands:" "\n"
  "  <nn> i     - set node ID (standard node ids are 1..30)" "\n"
  "  <n> b      - set MHz band (4 = 433, 8 = 868, 9 = 915)" "\n"
  "  <nnn> o    - Change frequency offset within the band above" "\n"
  "               values > 99 are sticky, 255 changes direction" "\n"
  "  <nnn> g    - set network group (RFM12 only allows 212, 0 = any)" "\n"
  "  <n> c      - set collect mode (advanced, normally 0)" "\n"
  "  t          - broadcast max-size test packet, request ack" "\n"
  "  ...,<nn> a - send data packet to node <nn>, request ack" "\n"
  "  ...,<nn> s - send data packet to node <nn>, no ack" "\n"
  "  <n> l      - turn activity LED on PB1 on or off" "\n"
  "  <n> q      - set quiet mode (1 = don't report bad packets)" "\n"
  "  <n> x      - set reporting format (0 = decimal, 1 = hex)" "\n"
  "  123 z      - total power down, needs a reset to start up again" "\n"
  "Remote control commands:" "\n"
  "  <hchi>,<hclo>,<addr>,<cmd> f     - FS20 command (868 MHz)" "\n"
  "  <addr>,<dev>,<on> k              - KAKU command (433 MHz)" "\n"
;
#endif

#if DATAFLASH
const char helpText2[] PROGMEM = 
  "Flash storage (JeeLink only):" "\n"
  "  d                                - dump all log markers" "\n"
  "  <sh>,<sl>,<t3>,<t2>,<t1>,<t0> r  - replay from specified marker" "\n"
  "  123,<bhi>,<blo> e                - erase 4K block" "\n"
  "  12,34 w                          - wipe entire flash memory" "\n"
;
#endif
#if not defined(__AVR_ATtiny84__) || not defined(__AVR_ATtiny44__)
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
#endif

#if not defined(__AVR_ATtiny84__) || not defined(__AVR_ATtiny44__)
static void showHelp () {
    showString(helpText1);
#endif
#if DATAFLASH
    if (df_present())
      showString(helpText2);
#endif
#if not defined(__AVR_ATtiny84__) || not defined(__AVR_ATtiny44__)
    Serial.println("Current configuration:");
#endif
    rf12_config();
}

static void handleInput (char c) {
  if ('0' <= c && c <= '9')
    value = 10 * value + c - '0';
  else if (c == ',') {
    if (top < sizeof stack)
      stack[top++] = value;
    value = 0;
  } else if ('a' <= c && c <='z') {
    Serial.print("> ");
    for (byte i = 0; i < top; ++i) {
      Serial.print((int) stack[i]);
      Serial.print(',');
    }
    Serial.print((int) value);
    Serial.println(c);
    word crc = ~0;  
    switch (c) {
      default:
        showHelp();
        break;
      case 'i': // set node id
        if ((value > 0) & (value < 32)) {
           config.nodeId = (config.nodeId & 0xE0) + (value & 0x1F);
           saveConfig();
        } else {
           showHelp();
        }
        break;
      case 'b': // set band: 4 = 433, 8 = 868, 9 = 915
        value = bandToFreq(value);
        if (value) {
         config.nodeId = (value << 6) + (config.nodeId & 0x3F);
         frequency = 1600;
         saveConfig();
        } else {
            showHelp();
        }
        break;
      case 'o': // Increment frequency within band
          if (value == 255) { 
            revF = !revF;
            revP = revP ^ 40;   // Flip the indicator
            value=0;
          } 
          
          Serial.print(frequency);
          Serial.print(revP);
///
/// It is important that you keep within your countries ISM spectrum management guidelines
/// i.e. allowable frequencies and their use when selecting your operating frequencies.
///
          if ((value) || (sticky)) {
            if (!value) value = sticky;
            if (value > 99) sticky = value; else sticky=0;     // Make values over 99 sticky
            if (!revF) frequency = frequency + value; else frequency = frequency - value; 
            if (frequency < 96) frequency = 3903;  // 96 - 3903 is the range of values supported by the RFM12B
            if (frequency > 3903) frequency = 96;
            Serial.println(frequency);
            saveConfig();           
          } else            
          Serial.println();
        break;
      case 'g': // set network group
        config.group = value;
        saveConfig();
        break;
      case 'c': // set collect mode (off = 0, on = 1)
        if (value)
          config.nodeId |= COLLECT;
        else
          config.nodeId &= ~COLLECT;
        saveConfig();
        break;
      case 't': // broadcast a maximum size test packet, request an ack
        cmd = 'a';
#if not defined(__AVR_ATtiny84__) || not defined(__AVR_ATtiny44__)
        sendLen = RF12_MAXDATA;
#else
        sendLen = RF12_MAXDATA - 50;    // Conserve RAM
#endif
        dest = 0;
        for (byte i = 0; i < RF12_MAXDATA; ++i)
          testbuf[i] = i + testCounter;
        Serial.print("test ");
        Serial.println((int) testCounter); // first byte in test buffer
        ++testCounter;
        break;
      case 'a': // send packet to node ID N, request an ack
      case 's': // send packet to node ID N, no ack
        cmd = c;
        sendLen = top;
        dest = value;
        memcpy(testbuf, stack, top);
        break;
      case 'l': // turn activity LED on or off
        activityLed(value);
        break;
#if not defined(__AVR_ATtiny84__) || not defined(__AVR_ATtiny44__)
      case 'f': // send FS20 command: <hchi>,<hclo>,<addr>,<cmd>f
        rf12_initialize(0, RF12_868MHZ, 0);
        activityLed(1);
        fs20cmd(256 * stack[0] + stack[1], stack[2], value);
        activityLed(0);
        rf12_config(0);
        break;
      case 'k': // send KAKU command: <addr>,<dev>,<on>k
        rf12_initialize(0, RF12_433MHZ, 0);
        activityLed(1);
        kakuSend(stack[0], stack[1], value);
        activityLed(0);
        rf12_config(0);
        break;
      case 'd': // dump all log markers
        if (df_present())
          df_dump();
        break;
      case 'r': // replay from specified seqnum/time marker
        if (df_present()) {
          word seqnum = (stack[0] << 8) || stack[1];
          long asof = (stack[2] << 8) || stack[3];
          asof = (asof << 16) | ((stack[4] << 8) || value);
          df_replay(seqnum, asof);
        }
        break;
      case 'e': // erase specified 4Kb block
        if (df_present() && stack[0] == 123) {
          word block = (stack[1] << 8) | value;
          df_erase(block);
        }
        break;
      case 'w': // wipe entire flash memory
        if (df_present() && stack[0] == 12 && value == 34) {
          df_wipe();
          Serial.println("erased");
        }
        break;
      case 'z': // put the ATmega in ultra-low power mode (reset needed)
        if (value == 123) Sleep;
        break;
#endif
        case 'q': // turn quiet mode on or off (don't report bad packets)
        if (value) config.flags |= QUIET;
          else config.flags &= ~QUIET;
        saveConfig();
        break;
      case 'x': // set reporting mode to hex (1) or decimal (0)
        useHex = value;
        break;
      case 'v': //display the interpreter version
        displayVersion(1);
        break;
      case 'j':
        for (byte i = 0; i < RF12_EEPROM_SIZE; ++i) {
            byte b = eeprom_read_byte(RF12_EEPROM_ADDR + i);
            showNibble(b >> 4);
            showNibble(b);
            if (value == 42) { 
             eeprom_write_byte((RF12_EEPROM_ADDR + RF12_EEPROM_SIZE) + i, b);
            }
        }            
        Serial.println();
        if (value == 42) Serial.println("Backed Up");

        if (value == 123) {
          for (byte i = 0; i < RF12_EEPROM_SIZE; ++i) {     // Check CRC to be restored
	        crc = _crc16_update(crc, eeprom_read_byte(RF12_EEPROM_ADDR + RF12_EEPROM_SIZE + i)); 
          }
          if (crc) { 
            Serial.println("Bad CRC"); 
          } else {
              for (byte i = 0; i < RF12_EEPROM_SIZE; ++i) {
                byte b = eeprom_read_byte((RF12_EEPROM_ADDR + RF12_EEPROM_SIZE) + i);
                showNibble(b >> 4);
                showNibble(b);
                eeprom_write_byte((RF12_EEPROM_ADDR) + i, b);
              }
            Serial.println();
            Serial.println("Restored");
            }
          if (rf12_config()) initialize(); else Serial.println("Initialize failed");
          }
        break;
#if defined debug
      case 'n': // Clear eeprom
        if (value == 123) {
          for (byte i = 0; i < (RF12_EEPROM_SIZE); ++i) {  // Use to clear config eeprom then backup to clear backup eeprom
            byte b = 255;
            eeprom_write_byte(RF12_EEPROM_ADDR + i, b);
          }
          Serial.println("Cleared");
        }
        break;
/////////////////////
      } // End Switch
#endif      
    value = top = 0;
    memset(stack, 0, sizeof stack);
  } else if (c == '>') {
    // special case, send to specific band and group, and don't echo cmd
    // input: band,group,node,header,data...
    stack[top++] = value;
    rf12_initialize(stack[2], bandToFreq(stack[0]), stack[1], frequency);
    rf12_sendNow(stack[3], stack + 4, top - 4);
    rf12_sendWait(2);
    rf12_config(0);
    value = top = 0;
    memset(stack, 0, sizeof stack);
  } else if (' ' < c && c < 'A')
    showHelp();
}

void displayVersion(uint8_t newline ) {
  Serial.print("\n[RF12demo.11]");
  if(newline!=0)  Serial.println();
}
void Sleep() {
          Serial.println(" sleeping");
          delay(10);
          rf12_sleep(RF12_SLEEP);
          cli();
          Sleepy::powerDown();
}

void setup() {
#if defined(__AVR_ATtiny84__) || not defined(__AVR_ATtiny44__)    
  delay(1000);  // Delay on startup to avoid ISP/RFM12B interference.
//  setPrescaler(0);             // div 1, i.e. speed up to 8 MHz
#endif
  Serial.begin(SERIAL_BAUD);
  displayVersion(0);
  activityLed(0);
#if defined JNMicro2
////////////////////////////////////////////////////////////////////////
/// Need some code to power up the RFM12B on the new Jeenode Micro   ///
////////////////////////////////////////////////////////////////////////

#endif
    if (rf12_config()) initialize();
    else {
    config.nodeId = 0x41; // 433 MHz, node 1
    config.group = 0xD4;  // default group 212
    frequency = 1600;
    config.flags = 0xC;        // Default flags, quiet off and non V10
//    saveConfig();            // Don't save to eeprom until we have changes.
  }

#if DATAFLASH
  df_initialize();
#endif 
#if not defined(__AVR_ATtiny84__) || not defined(__AVR_ATtiny44__)    
  showHelp();
#endif
}
void initialize() {
    config.nodeId = eeprom_read_byte(RF12_EEPROM_ADDR);
    config.group = eeprom_read_byte(RF12_EEPROM_ADDR + 1);
    frequency = eeprom_read_byte(RF12_EEPROM_ADDR + 2);
    config.flags = frequency >> 4;               // Extract the flag nibble
    if (config.flags & V10)                      // Is this a pre v11 eeprom
      frequency = 1600; 
    else 
      frequency = ((frequency & 0x0F)  << 8) + (eeprom_read_byte(RF12_EEPROM_ADDR + 3));              // Loose flag nibble to get frequency high order
}
void loop() {
  if (Serial.available())
    handleInput(Serial.read());

  if (rf12_recvDone()) {
    byte n = rf12_len;
    if (rf12_crc == 0)
      Serial.print("OK");
    else {
      if (config.flags & ~QUIET);
        return;
      Serial.print(" ?");
      if (n > 16) // print at most 16 bytes if crc is wrong
        n = 16;
    }
    if (useHex)
      Serial.print('X');
    if (config.group == 0) {
      Serial.print(" G");
      showByte(rf12_grp);
    }
    Serial.print(' ');
/// Clear string showing ascii interpretation of rf12_data, max length == RF12_MAXDATA JOH
    showByte(rf12_hdr);
    for (byte i = 0; i < n; ++i) {
      if (!useHex)
        Serial.print(' ');
      showByte(rf12_data[i]);
    }
    Serial.println();
    
    if (rf12_crc == 0) {
      activityLed(1);
#if not defined(__AVR_ATtiny84__) || not defined(__AVR_ATtiny44__)    
      if (df_present())
        df_append((const char*) rf12_data - 2, rf12_len + 2);
#endif
      if (RF12_WANTS_ACK && (config.nodeId & COLLECT) == 0) {
        Serial.println(" -> ack");
        rf12_sendStart(RF12_ACK_REPLY, 0, 0);
      }
      
      activityLed(0);
    }
  }

  if (cmd && rf12_canSend()) {
    activityLed(1);

    Serial.print(" -> ");
    Serial.print((int) sendLen);
    Serial.println(" b");
    byte header = cmd == 'a' ? RF12_HDR_ACK : 0;
    if (dest)
      header |= RF12_HDR_DST | dest;
    rf12_sendStart(header, testbuf, sendLen);
    cmd = 0;

    activityLed(0);
  }
}
