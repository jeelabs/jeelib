/// @dir RFxConsole
///////////////////////////////////////////////////////////////////////////////
#define RF69_COMPAT      1   // define this to use the RF69 driver i.s.o. RF12 
///                          // The above flag must be set similarly in RF12.cpp
///                          // and RF69_avr.h
#define BLOCK  0             // Alternate LED pin?
#define INVERT_LED       0   // 0 is Jeenode usual and 1 inverse
//
/* AutoRxRestartOn = 1, page 24:
   after the controller has emptied the FIFO the receiver will re-enter the WAIT mode described
   above, after a delay of InterPacketRxDelay, allowing for the distant transmitter to ramp down, hence avoiding a false
   RSSI detection. In both cases (AutoRxRestartOn=0 or AutoRxRestartOn=1), the receiver can also re-enter the WAIT
   mode by setting RestartRx bit to 1.

   Works but somehow, with AFC active the receiver drifts off into the wilderness
   and doesn't receive unless forced to transmit. 
 */
///////////////////////////////////////////////////////////////////////////////
/// Configure some values in EEPROM for easy config of the RF12 later on.
// 2009-05-06 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php
// this version adds flash memory support, 2009-11-19
// Adding frequency features. 2013-09-05
// Added postbox semaphore feature 2013-10-24
// Added message storage feature 2014-03-04
// Add acknowledgement to all node groups 2014-05-20
// Increase support to 100 nodes mixed between all groups 2014-05-24
// Add 1284p supporting over 1000 nodes 2014-08-20
// Based on RF12Demo from RF12Demo branch 2014-11-24
// Add RegRssiThresh to eeprom config 2015-01-08
// Introduce shifted commands:
// 'S' to interact with Salus FSK devices 2015-08-28
// Avoid adding shifted commands 'A' through 'F'
// as these are used by the hexadecimal input code
// Added +/- to match RF frequency between different hardware
// Added T & R commands to set hardware specific transmit & receive parameters
// Added rolling percentage packet quality, displayed before any RSSI value with each packet
// Added min/max for above
// Basic verification that a Posting has been received. It should appear as rf12_data[0] in following packets
// Added min/max FEI levels per node 2016-05-13
// Added verbosity flag, command 15v displays all extra information 2016-06-10
//  1v displays packet reception details, fei, lna etc
//  2v displays a basic packet decode
//  4v displays receiver restarts
//	8v displays RX Threshold adjustments use with 10,200R
//  add any values above to combine display settings.
// Changed RX interrupt trigger to be RSSI rather than SyncMatch 2016-06-20
// Mask two stray interrupts, sleep before reconfiguring radio 2016-06-23
// Added dynamic control of RX Threshold based on restart rate 2016-07-12
// Variable time constant for rate calculation stoed in eeprom
// Added counter for ACK's aborted due to busy airwaves 2016-09-01
// Added inter packet time gaps, also min/max 2016-12-12
// Tweaks around <cr> handling to better fit use of folie as a terminal emulator 2017-01-5
// Add 1Hz Timer1 for rate calculation and longer elapsed duration 2017-02-28

#if defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny44__)
#define TINY 1
#endif

#if TINY
#define OOK          0   // Define this to include OOK code f, k - Adds ?? bytes to Tiny image
#define JNuMOSFET    1   // Define to power up RFM12B on JNu2/3 - Adds 4 bytes to Tiny image
#else
#define configSTRING 1   // Define to include "A i1 g210 @ 868 MHz q1" - Adds ?? bytes to Tiny image
#define HELP         1   // Define to include the help text
#define MESSAGING    1   // Define to include message posting code m, p - Will not fit into any Tiny image
#define STATISTICS   1   // Define to include stats gathering - Adds ?? bytes to Tiny image
#define NODE31ALLOC  1   // Define to include offering of spare node numbers if node 31 requests ack
#define DEBUG        0   //
#endif

#define REG_BITRATEMSB 0x03  // RFM69 only, 0x02, // BitRateMsb, data rate = 49,261 khz
#define REG_BITRATELSB 0x04  // RFM69 only, 0x8A, // BitRateLsb divider = 32 MHz / 650 == 49,230 khz
#define REG_BITFDEVMSB 0x05  // RFM69 only, 0x02, // FdevMsb = 45 KHz
#define REG_BITFDEVLSB 0x06  // RFM69 only, 0xE1, // FdevLsb = 45 KHz
#define REG_RSSIVALUE  0x24
#define REG_SYNCCONFIG 0x2E  // RFM69 only, register containing sync length
#define oneByteSync    0x87  // RFM69 only, value to get only one byte sync with max bit errors.
#define REG_SYNCGROUP  0x32  // RFM69 only, register containing group number
#define REG_SYNCVALUE7 0x35  // RFM69 only
#define REG_SYNCVALUE8 0x36  // RFM69 only

#include <JeeLib.h>
#include <util/crc16.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <util/parity.h>
#include "release.h"    // Version tracking by dzach

#define MAJOR_VERSION RF12_EEPROM_VERSION // bump when EEPROM layout changes
#define MINOR_VERSION 0                   // bump on other non-trivial changes
#define VERSION "\n[RFxConsole.4]\n"    // keep in sync with the above
#if !configSTRING
#define rf12_configDump()                 // Omit A i1 g210 @ 868 MHz q1
#endif
#if TINY
#define SERIAL_BAUD    38400   // can only be 9600 or 38400
#define DATAFLASH      0       // do not change
#undef  LED_PIN                // do not change
#define messageStore  16
#else
#define TINY        0
#define SERIAL_BAUD 115200  // adjust as needed
#define DATAFLASH   0       // set to 0 for non-JeeLinks, else 4/8/16 (Mbit)
#endif

/// Save a few bytes of flash by declaring const if used more than once.
const char INITFAIL[] PROGMEM = "\nInit failed\n";
const char RFM12x[] PROGMEM = "RFM12x ";
const char RFM69x[] PROGMEM = "RFM69x ";
const char BLOC[] PROGMEM = "BLOCK ";
const char UNSUPPORTED[] PROGMEM = "RX Unsupported ";
const char DONE[] PROGMEM = "Done\n";
const char ABORTED[] PROGMEM = " Aborted ";


#define SALUSFREQUENCY 1660       // Default value
byte salusMode = false;
unsigned int SalusFrequency = SALUSFREQUENCY;


unsigned int NodeMap;
unsigned int newNodeMap;
byte stickyGroup;
byte eepromWrite;
byte qMin = ~0;
byte qMax = 0;
byte lastrssiThreshold;
unsigned long lastRSSIrestart;
unsigned long lastThresholdRSSIrestart;
unsigned long rxLast;
unsigned long minGap = ~0;
unsigned long maxGap = 0; 

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
// http://jeelabs.net/boards/7/topics/3229?r=3268#message-3268

#if SERIAL_BAUD == 9600
#define BITDELAY 54          // 9k6 @ 8MHz, 19k2 @16MHz
#endif
#if SERIAL_BAUD == 38400
#define BITDELAY 12          // 28/5/14 from value 11 // 38k4 @ 8MHz, 76k8 @16MHz
#endif

#define MAX_NODES 0
#define _receivePin 8
static char _receive_buffer;
static byte _receive_buffer_index;

static void showString (PGM_P s); // forward declaration

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
#elif defined(__AVR_ATmega1284P__) // Moteino MEGA
// http://lowpowerlab.com/moteino/#whatisitMEGA
#define LED_PIN     15       // activity LED, comment out to disable on/off operation is reversed to a normal Jeenode
#define LED_ON       1
#define LED_OFF      0
#define messageStore  255    // Contrained by byte variables
#define MAX_NODES 1004       // Constrained by eeprom

#else
#if BLOCK
#define LED_PIN     8        // activity LED, comment out to disable
#else
#define LED_PIN     9        // activity LED, comment out to disable
#endif
#define messageStore  128
#define MAX_NODES 31        // Contrained by RAM (9 bytes RAM per node)
#endif

ISR(WDT_vect) { Sleepy::watchdogEvent(); }

volatile unsigned long elapsedSeconds;
volatile unsigned long previousRestarts;
volatile unsigned int restartRate;
volatile unsigned int maxRestartRate;
ISR(TIMER1_COMPA_vect){
	elapsedSeconds++;

    // Update restart rate
    restartRate = (rfapi.RSSIrestart - previousRestarts);
    previousRestarts = rfapi.RSSIrestart;
              	
    if (restartRate > maxRestartRate) { 
    	maxRestartRate = restartRate;
    }                     
}


static unsigned long now () {
    // FIXME 49-day overflow
    return millis() / 1000;
} 

byte ledStatus = 0;
static void activityLed (byte on) {
#ifdef LED_PIN
    pinMode(LED_PIN, OUTPUT);
#endif
#if INVERT_LED
    ledStatus = on;            // Reported by the software
#ifdef LED_PIN
    digitalWrite(LED_PIN, on);  // Inverted by the hardware
#endif
#else
    ledStatus = on;
#ifdef LED_PIN
    digitalWrite(LED_PIN, !on);
#endif
#endif
}

static void printOneChar (char c) {
    Serial.print(c);
}

static void showString (PGM_P s) {
    for (;;) {
        char c = pgm_read_byte(s++);
        if (c == 0)
            break;
        if (c == '\n')
            printOneChar('\r');
        printOneChar(c);
    }
}

static void displayVersion () {
    showString(PSTR(VERSION));
#if TINY
    showString(PSTR(" Tiny"));
#endif
}

/// @details
/// For the EEPROM layout, see http://jeelabs.net/projects/jeelib/wiki/RF12demo
/// Useful url: http://blog.strobotics.com.au/2009/07/27/rfm12-tutorial-part-3a/

// RF12 configuration area
typedef struct {

    byte nodeId;            // used by rf12_config, offset 0
    byte group;             // used by rf12_config, offset 1
    byte format;            // used by rf12_config, offset 2
    byte output       :2;   // 0 = dec, 1 = hex, 2 = dec+ascii, 3 = hex+ascii
    byte collect_mode :1;   // 0 = ack, 1 = don't send acks
    byte quiet_mode   :1;   // 0 = show all, 1 = show only valid packets
    byte spare_flags  :3;   // offset 3
    byte defaulted    :1;   // 0 = config set via UI
    word frequency_offset;  // used by rf12_config, offset 4 & 5
    byte RegPaLvl;          // See datasheet RFM69x Register 0x11, offset 6
    byte RegRssiThresh;     // See datasheet RFM69x Register 0x29, offset 7
    signed int matchingRF :8;// Frequency matching for this hardware, offset 8
    byte ackDelay         :4;// Delay in ms added on turnaround RX to TX, RFM69 offset 9
    byte verbosity        :4;// Controls output format offset 9
    byte clearAir;          // Transmit permit threshold, offset 10
    byte rateInterval;      // Seconds between rate updates offset 11
    byte pad[RF12_EEPROM_SIZE - 14];
    word crc;

} RF12Config;
static RF12Config config;

static char cmd;
static unsigned int value;
static byte stack[RF12_MAXDATA+4], top, sendLen, dest;
static byte testCounter;
static word messageCount = 0;

//#if !TINY
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
//#endif

byte ones = 0;
byte other = 0;
byte watchNode = 0;
byte lastTest;
byte busyCount;
byte missedTests;
byte sendRetry = 0;
unsigned int packetAborts;
unsigned int testTX;
unsigned int testRX;

#if MESSAGING
static byte semaphores[MAX_NODES];
#endif
static unsigned long goodCRC;
#if RF69_COMPAT && STATISTICS
static signed int minFEI[MAX_NODES];
static signed int maxFEI[MAX_NODES];
static byte minRSSI[MAX_NODES];
static byte lastRSSI[MAX_NODES];
static byte maxRSSI[MAX_NODES];
static byte minLNA[MAX_NODES];
static byte lastLNA[MAX_NODES];
static byte maxLNA[MAX_NODES];
#endif
#if RF69_COMPAT && !TINY
static byte CRCbadMinRSSI = 255;
static byte CRCbadMaxRSSI = 0;
static byte minTxRSSI = 255;
static byte maxTxRSSI = 0;

static signed int previousAFC;
static signed int previousFEI;
static unsigned int changedAFC;
static unsigned int changedFEI;
#endif
#if STATISTICS
static unsigned int CRCbadCount = 0;
static unsigned int pktCount[MAX_NODES];
static unsigned int nonBroadcastCount = 0;
static byte postingsIn, postingsOut;
#endif

unsigned int loopCount, idleTime = 0, offTime = 0;

#if !TINY
const char messagesF[] PROGMEM = { 
    0x05, 'T', 'e', 's', 't', '1', 
    0x05, 'T', 'e', 's', 't', '2', 

    0 }; // Mandatory delimiter

#define MessagesStart 129

byte messagesR[messageStore];

byte *sourceR;
byte topMessage;    // Used to store highest message number
#endif

static void showNibble (byte nibble) {
    char c = '0' + (nibble & 0x0F);
    if (c > '9')
        c += 7;
    Serial.print(c);
}

static void showByte (byte value) {
    if (config.output & 0x1) {
        showNibble(value >> 4);
        showNibble(value);
    } else
        Serial.print((word) value, DEC);
}
static void showWord (unsigned int value) {
    if (config.output & 0x1) {
        showByte (value >> 8);
        showByte (value);
    } else
        Serial.print((word) value);    
}

static word calcCrc (const void* ptr, byte len) {
    word crc = ~0;
    for (byte i = 0; i < len; ++i)
        crc = _crc16_update(crc, ((const byte*) ptr)[i]);
    return crc;
}

static void loadConfig () {
    // eeprom_read_block(&config, RF12_EEPROM_ADDR, sizeof config);
    // this uses 166 bytes less flash than eeprom_read_block(), no idea why
    for (byte i = 0; i < sizeof config; ++i)
        ((byte*) &config)[i] = eeprom_read_byte(RF12_EEPROM_ADDR + i);
    lastrssiThreshold = config.RegRssiThresh;
    rfapi.rateInterval = (uint32_t)config.rateInterval << 10;    
    config.defaulted = false;   // Value if UI saves config
}

static void saveConfig () {
    activityLed(1);
    config.format = MAJOR_VERSION;
    config.crc = calcCrc(&config, sizeof config - 2);
    // eeprom_write_block(&config, RF12_EEPROM_ADDR, sizeof config);
    // this uses 170 bytes less flash than eeprom_write_block(), no idea why

    for (byte i = 0; i < sizeof config; ++i) {
        byte* p = &config.nodeId;
        if (eeprom_read_byte(RF12_EEPROM_ADDR + i) != p[i]) {
            eeprom_write_byte(RF12_EEPROM_ADDR + i, p[i]);
            eepromWrite++;
        }
    }

    salusMode = false;
#if STATISTICS    
    messageCount = nonBroadcastCount = CRCbadCount = 0; // Clear stats counters
#endif
    rf12_sleep(RF12_SLEEP);                                             // Sleep while we tweak things
    if (!rf12_configSilent()) showString(INITFAIL);
    activityLed(0); 
    showString(DONE);
} // saveConfig

static byte bandToFreq (byte band) {
    return band == 4 ? RF12_433MHZ : band == 8 ? RF12_868MHZ : band == 9 ? RF12_915MHZ : 0;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// OOK transmit code

#if RF69_COMPAT // not implemented in RF69 compatibility mode
static void fs20cmd(word house, byte addr, byte cmd) {}
static void kakuSend(char addr, byte device, byte on) {}
#else

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
#include "dataflash.h"
#else // DATAFLASH

#define df_present() 0
#define df_initialize()
#define df_dump()
#define df_replay(x,y)
#define df_erase(x)
#define df_wipe()
#define df_append(x,y)

#endif

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

const char helpText1[] PROGMEM =
"\n"
"Available commands:\n"
" <nn>i      - set node ID (standard node ids are 1..30)\n"
" <n>b       - set MHz band (4 = 433, 8 = 868, 9 = 915)\n"
" <nnnn>o    - change frequency offset within the band (default 1600)\n"
"               96..3903 is the range supported by the RFM12B\n"
" <nnn>g     - set network group (RFM12 only allows 212, 0 = any)\n"
" <n>c       - set collect mode (advanced, normally 0)\n"
" t          - broadcast max-size test packet, request ack\n"
" ...,<nn>a  - send data packet to node <nn>, request ack\n"
"              if using group 0 then sticky group number is used\n"
" ...,<nn>s  - send data packet to node <nn>, no ack\n"
" ... <nn>   - space character is a valid delimiter\n"
" 128,<i>,n  - release group/node index number <i> entry in eeprom\n"
" <g>n       - set group <g> as sticky. Group 0 only, see p command\n"
" <n>l       - turn activity LED on PB1 on or off\n"
"  ...,m     - add message string to ram, see p command\n"
" <g>,<i>,<s>p post semaphore <s> for group <g> node <i>, to be\n"
"              sent with its next ack. Group number becomes sticky\n"
" <n>q       - set quiet mode (1 = don't report bad packets)\n"
" <n>x       - set reporting format (0: decimal, 2: decimal+ascii\n"
"            -  1: hex, 3: hex+ascii)\n"
" v          - return firmware version and current settings\n"
" <n>+ or -  - RF hardware matching\n" 
#if !TINY
" 123z       - total power down, needs a reset to start up again\n"
#endif
#if OOK
"Remote control commands:\n"
" <hchi>,<hclo>,<addr>,<cmd> f      - FS20 command (868 MHz)\n"
" <addr>,<dev>,<on> k               - KAKU command (433 MHz)\n"
#endif
;

const char helpText2[] PROGMEM =
"Flash storage (JeeLink only):\n"
"   d                                  - dump all log markers\n"
"   <sh>,<sl>,<t3>,<t2>,<t1>,<t0> r    - replay from specified marker\n"
"   123,<bhi>,<blo> e                  - erase 4K block\n"
"   12,34 w                            - wipe entire flash memory\n"
;

static void showHelp () {
#if TINY
    showString(PSTR("?\n"));
#elif HELP
    showString(helpText1);
    if (df_present())
        showString(helpText2);
#endif
#if !TINY && configSTRING
    Serial.print((__DATE__));
    Serial.print(" ");
    Serial.print((__TIME__));
    Serial.print(" ");
    Serial.println(VERSION_SIG);
#if RF69_COMPAT
    showString(RFM69x);
#else
    showString(RFM12x);
#endif
    showStatus();
#endif

}

static void showStatus() {
#if RF69_COMPAT
    showString(RFM69x);
#else
	showString(RFM12x);
#endif
	unsigned long s = elapsedSeconds;
    showString(PSTR("Elapsed "));
    Serial.print(s / 86400UL);
	printOneChar('d');
    Serial.print((s%86400UL) / 3600UL);
	printOneChar('h');
    Serial.print((s%3600UL) / 60UL);
	printOneChar('m');
    Serial.print(s%60UL);
	printOneChar('s');
	printOneChar('=');
    Serial.print(elapsedSeconds);
	printOneChar('s');
    showString(PSTR(", Led is ")); if (ledStatus) showString(PSTR("on")); else showString(PSTR("off"));
    showString(PSTR(", Free Ram(b) "));
    Serial.print(freeRam());     
#if RF69_COMPAT
    showString(PSTR(", RX Restarts "));
    Serial.print(rfapi.RSSIrestart);
    showString(PSTR(", Rate "));
    Serial.print(restartRate);
    printOneChar('m');
    Serial.print(maxRestartRate);
    showString(PSTR("/s, Sync Match "));
    Serial.print(rfapi.syncMatch);

    showString(PSTR(", Good CRC "));
    Serial.print(goodCRC);
    showString(PSTR(", Noise Floor "));
    Serial.print(rfapi.noiseFloorMin);
    printOneChar('/');
    Serial.print(rfapi.rssiThreshold);
    printOneChar('/');
    Serial.print(rfapi.noiseFloorMax);
    showString(PSTR(", Ack Aborts "));
    Serial.print(packetAborts);
    showString(PSTR(", Floor "));
    Serial.print(minTxRSSI);
    printOneChar('/');
    Serial.print(maxTxRSSI);
    showString(PSTR(", InterSync(ms) "));
    Serial.print(minGap);
    printOneChar('/');
    Serial.print((millis() - rfapi.interpacketTS));
    printOneChar('/');
    Serial.print(maxGap);
    showString(PSTR(", Mode Errors "));
    Serial.print(rfapi.modeError);

#endif
    showString(PSTR(", Eeprom"));
    rf12_configDump();
#if RF69_COMPAT
    if (!RF69::present) {
        showString(PSTR("RFM69x Problem "));        
        Serial.print((RF69::control(REG_SYNCVALUE7,0)), HEX);
        Serial.println((RF69::control(REG_SYNCVALUE8,0)), HEX);
        unsigned int mask = 0xAA;
        for (unsigned int i = 0; i < 8; i++) {
            RF69::control(REG_SYNCVALUE7 | 0x80, mask);
            Serial.print(mask, BIN);
            printOneChar('?');
            Serial.println((RF69::control(REG_SYNCVALUE7, 0)), BIN);
            mask = mask >> 1;
        }
    }

    byte* b = RF69::SPI_pins();  // {OPTIMIZE_SPI, PINCHG_IRQ, RF69_COMPAT, RFM_IRQ, SPI_SS, SPI_MOSI, SPI_MISO, SPI_SCK }

#if defined(__AVR_ATmega1284P__) // Moteino MEGA    
static byte n[] = {1,0,1,4,5,6,7,2,2};     // ATMEega1284 with RFM69 settings
#else
static byte n[] = {1,0,1,2,3,4,5,2,0};     // Default ATMega328 with RFM69 settings
#endif

byte mismatch = false;
for (byte i = 0; i < 9; i++) {
    if (b[i] != n[i]) {
        mismatch = true;
        showByte(i);
        printOneChar(':');
        showByte(b[i]);
        printOneChar(' ');
    }
}    

if (mismatch) showString(PSTR("Mismatch"));
//#if DEBUG
printOneChar('[');
Serial.print(RF69::unexpected);
printOneChar(',');
Serial.print(RF69::unexpectedFSM);
printOneChar(',');
Serial.print(RF69::unexpectedIRQFLAGS2);
printOneChar(',');
Serial.print(RF69::unexpectedMode);
printOneChar(']');
printOneChar(',');
Serial.print(RF69::IRQFLAGS2);
printOneChar(',');
Serial.print(RF69::DIOMAPPING1);
//#endif
Serial.println();
#endif    
Serial.flush();
}

bool cr = false;
static void handleInput (char c) {
    //      Variable value is now 16 bits to permit offset command, stack only stores 8 bits
    //      not a problem for offset command but beware.
    if ('0' <= c && c <= '9') {
        if (config.output & 0x1) value = 16 * value + c - '0';
        else value = 10 * value + c - '0';
        return;
    }

    if (('A' <= c && c <= 'F') && (config.output & 0x1)) {
        value = 16 * value + (c - 'A' + 0xA);
        return;
    }

    if (c == ',' || c == ' ') {   // Permit comma or space as delimiters
        if (top < sizeof stack)
            stack[top++] = value; // truncated to 8 bits
        value = 0;
        return;
    }
    
    if ((cr) && (c == 13)) {
    	value = 0;	// Loose <cr> to work with folie
    	cr = false;	// Allow the next <cr>
    	return;
    }

    if (32 > c || c > 'z') {    // Trap unknown characters bar null
		if (c != 13) {
	        for (byte i = 0; i < top; ++i) {
    	        showByte(stack[i]);
        	    printOneChar(',');
        	}
        	showWord(value);
        	showString(PSTR(",Key="));
        	showByte(c);          // Highlight Tiny serial framing errors.  
        	printOneChar(',');
        }
        
        showStatus();

        value = top = 0;	// Clear up
        ones = 0;
        other = 0;
    } else cr = true;		// Loose next <cr> to work with folie

    // keeping this out of the switch reduces code size (smaller branch table)
    // TODO Using the '>' command with incorrect values hangs the hardware
    if (c == '>') {
        // special case, send to specific band and group, and don't echo cmd
        // input: band,group,node,header,data...
        stack[top++] = value;
        // TODO: frequency offset is taken from global config, is that ok?
        // I suspect not OK, could add a new number on command line,
        // the last value before '>' as the offset is the only place a 16 bit value will available.
        rf12_initialize(stack[2], bandToFreq(stack[0]), stack[1],
                config.frequency_offset);
        rf12_sendNow(stack[3], stack + 4, top - 4);
        rf12_sendWait(2);
        rf12_configSilent();

    } else if (c > ' ') {

        // TODO Do we need the "else if" above    

        switch (c) {

            case 'i': // set node id
                if ((value > 0) && (value <= 31)) {
                    config.nodeId = (config.nodeId & 0xE0) + (value & 0x1F);
                    saveConfig();
                }
                break;

            case 'b': // set band: 4 = 433, 8 = 868, 9 = 915
                value = bandToFreq(value);
                if (value) {
                    config.nodeId = (value << 6) + (config.nodeId & 0x3F);
                    config.frequency_offset = 1600;
                    saveConfig();
                }
                break;

            case 'g': // set network group
                config.group = value;
                saveConfig();
                //            stickyGroup = value;
                break;

            case 'o':{ // Offset frequency within band
                         // Stay within your country's ISM spectrum management guidelines, i.e.
                         // allowable frequencies and their use when selecting operating frequencies.
                         if (value) {
                             if (((value + config.matchingRF + config.matchingRF) > 95) 
                                     && ((value + config.matchingRF + config.matchingRF) < 3904)) { // supported by RFM12B
                                 Serial.println(value + config.matchingRF);
                                 config.frequency_offset = value;
                                 saveConfig();
                             } else {
                                 showString(UNSUPPORTED);
                                 break;
                             } 
                         } else value = config.frequency_offset;
#if RF69_COMPAT
                         if (stack[0]) RF69::microOffset = (stack[0] & 63);
#endif
#if !TINY
                         // this code adds about 400 bytes to flash memory use
                         // display the exact frequency associated with this setting
                         byte freq = 0, band = config.nodeId >> 6;
                         switch (band) {
                             case RF12_433MHZ: freq = 43; break;
                             case RF12_868MHZ: freq = 86; break;
                             case RF12_915MHZ: freq = 90; break;
                         }
                         uint32_t f1 = (freq * 100000L + band * 25L * config.frequency_offset) + RF69::microOffset;
                         Serial.print((word) (f1 / 10000));
                         printOneChar('.');
                         word f2 = f1 % 10000;
                         // tedious, but this avoids introducing floating point
                         printOneChar('0' + f2 / 1000);
                         printOneChar('0' + (f2 / 100) % 10);
                         printOneChar('0' + (f2 / 10) % 10);
                         printOneChar('0' + f2 % 10);
                         showString(PSTR(" MHz"));

#if RF69_COMPAT
                         showString(PSTR(" Noise Floor"));
                         for (byte i = 0; i < 10; i++) {
                             //          Serial.print(RF69::rssiConfig);
                             showString(PSTR(" @"));
                             // display current RSSI value in this channel
                             byte r = RF69::currentRSSI();
                             if (config.output & 0x1)                  // Hex output?
                                 showByte(r);
                             else {
                                 Serial.print(r >> 1);
                                 if (r & 0x01) showString(PSTR(".5"));
                                 showString(PSTR("dB "));
                             }
                         }
#endif
                         Serial.println();
#endif
                         break;
                     }

            case '+': // Increment hardware dependant RF offset
                     if (value) {
                         if (((config.frequency_offset + config.matchingRF + value) > 95) 
                                 && (config.frequency_offset + config.matchingRF + value) < 3904) { // supported by RFM12B              
                             config.matchingRF = config.matchingRF + value;
                             Serial.println(config.matchingRF);
                             saveConfig();
                         } else {
                             showString(UNSUPPORTED);
                         } 
                     } else {
                         if (config.matchingRF > (-1)) printOneChar('+');
                         Serial.println(config.matchingRF);
                         c = ' ';
                     }
                     break;

            case '-': // Increment hardware dependant RF offset
                     if (value) {
                         if (((config.frequency_offset + config.matchingRF - value) > 95) 
                                 && (config.frequency_offset + config.matchingRF - value) < 3904) { // supported by RFM12B              
                             config.matchingRF = config.matchingRF - value;
                             Serial.println(config.matchingRF);
                             saveConfig();
                         } else {
                             showString(UNSUPPORTED);
                         } 
                     } else {
                         if (config.matchingRF > (-1)) printOneChar('+');
                         Serial.println(config.matchingRF);
                         c = ' ';
                     }
                     break;

            case 'c': // set collect mode (off = 0, on = 1)
                     config.collect_mode = value;
                     if (top == 1) {
                         Serial.print(config.ackDelay); printOneChar(' ');
                         config.ackDelay = stack[0]; // Ack turnaround additional delay for RFM69
                     }
                     saveConfig();
                     break;

            case 't': // broadcast a maximum size test packet, request an ack
                     // Various test packets may be requested:
                     //   50,0,t will transmit byte 0x00 repeated 50 times
                     // 64,170,t will transmit 64 bytes of 0xAA, repeated bits alternating
                     // 66,255,t will transmit 66 bytes of 0xFF
                     //       0t will transmit 66 bytes incrementing from 0x00, changing but biased 0
                     //     190t will transmit 66 bytes incrementing from 0xBF, changing but biased 1
                     //   20,48t will transmit 20 bytes incrementing from 0x30
                     //      0,t will transmit a zero length packet
                     cmd = 'a';
                     if (top >= 1 && stack[0] <= RF12_MAXDATA)
                         sendLen = stack[0];
                     else sendLen = RF12_MAXDATA;
                     dest = 0;
                     if (value != 0) testCounter = value;  // Seed test pattern?
                     for (byte i = 0; i < RF12_MAXDATA; ++i) {
                         if (top == 2) 
                             stack[i] = stack[1];       // fixed byte pattern
                         else stack[i] = i + testCounter;
                     }

                     showString(PSTR("test "));
                     if (sendLen) showByte(stack[0]); // first byte in test buffer
                     ++testCounter;
                     testTX++;
                     break;

            case 'a': // send packet to node ID N, request an ack
                     // TODO Group number used is "stickyGroup" unless we do something here.
            case 's': // send packet to node ID N, no ack
                     cmd = c;
                     sendLen = top;
                     dest = (byte)value;
                     break;

            case 'T': 
                     // Set hardware specific TX power in eeprom
                     if(value) config.RegPaLvl = value;
                     // Transmit permit threshold
                     if (top == 1 && (stack[0])) config.clearAir = stack[0];
                     saveConfig();
                     break;

            case 'R': // Set hardware specific RX threshold in eeprom
                     //        	Serial.println(value);

#if RF69_COMPAT
                     RF69::control((0x80) | (0x29), value);	// Set radio
                     //	    	Serial.println(RF69::control(0x29, 160));

#endif
                     config.RegRssiThresh = value;
                     //        	Serial.println(config.RegRssiThresh);
                     rfapi.rssiThreshold = value;
                     //        	Serial.println(rfapi.rssiThreshold);
                     if (top == 1) {
                         config.rateInterval = stack[0];
                         rfapi.rateInterval = (uint32_t)(config.rateInterval) << 10;
                     }
                     saveConfig();
                     break;

#if !TINY
            case 'S': // send FSK packet to Salus devices
                     if (!top) {
                         if (value) SalusFrequency = value;
                         else value = SalusFrequency;
                     }

                     rf12_sleep(RF12_SLEEP);                                             // Sleep while we tweak things
                     rf12_initialize (config.nodeId, RF12_868MHZ, 212, SalusFrequency);  // 868.30 MHz
                     rf12_sleep(RF12_SLEEP);                                             // Sleep while we tweak things
#if RF69_COMPAT
                     RF69::control(REG_BITRATEMSB | 0x80, 0x34);                         // 2.4kbps
                     RF69::control(REG_BITRATELSB | 0x80, 0x15);
                     RF69::control(REG_BITFDEVMSB | 0x80, 0x04);                         // 75kHz freq shift
                     RF69::control(REG_BITFDEVLSB | 0x80, 0xCE);
                     rfapi.RssiToSync = SALUSPACKET16;
#else
                     rf12_control(RF12_DATA_RATE_2);                                     // 2.4kbps
                     rf12_control(0x9830);                                               // 75khz freq shift
#endif
                     salusMode = true;
                     rf12_skip_hdr(2);                   // Ommit Jeelib header 2 bytes on transmission & validating reception
                     rf12_fix_len(15);                   // Maximum fixed length packet size.
                     rf12_sleep(RF12_WAKEUP);            // All set, wake up radio

                     if (top >= 1) {
                         cmd = c;
                         // New Command format 165,8,0S == ON
                         // 165,8,17S
                         //                stack[2] = value;
                         //                stack[1] = OK;
                         //                stack[0] = OK;  
                         sendLen = 5; //4?
                         /*
                         // Command format 16,1S
                         // 16 is the ID
                         // 1 = ON
                         // 2 = OFF
                         stack[3] = 90;
                         stack[2] = value | stack[0];
                         stack[1] = value;
                         sendLen = 4; */
                     }
                     break;
#endif
            case 'W': // Watch specific packet type
                     watchNode = value;
                     break;
#if OOK
            case 'f': // send FS20 command: <hchi>,<hclo>,<addr>,<cmd>f
                     rf12_initialize(0, RF12_868MHZ, 0);
                     activityLed(1);
                     fs20cmd(256 * stack[0] + stack[1], stack[2], value);
                     activityLed(0);
                     rf12_configSilent();
                     break;

            case 'k': // send KAKU command: <addr>,<dev>,<on>k
                     rf12_initialize(0, RF12_433MHZ, 0);
                     activityLed(1);
                     kakuSend(stack[0], stack[1], value);
                     activityLed(0);
                     rf12_configSilent();
                     break;
#endif
            case 'q': // turn quiet mode on or off (don't report bad packets)
                     config.quiet_mode = value;
                     saveConfig();

#if RF69_COMPAT
                     // The 4 byte sync used by the RFM69 reduces detected noise dramatically.
                     // The command below sets the sync length to 1 to test radio reception.
                     if (top == 1) RF69::control(REG_SYNCCONFIG | 0x80, oneByteSync); // Allow noise
                     // Appropriate sync length will be reset by the driver after the next transmission.
                     // The 's' command is an good choice to reset the sync length. 
                     // Packets will not be recognised until until sync length is reset.
#endif
                     break;

            case 'x': // set reporting mode to decimal (0), hex (1), hex+ascii (2)
                     config.output = value;
#if RF69_COMPAT
                     config.RegPaLvl = RF69::control(0x11, 0x9F);   // Pull the current RegPaLvl from the radio
                     // An obscure method because one can blow the hardware
#endif                                                     
                     saveConfig();
                     break;

            case 'v': // display the interpreter version
                     config.verbosity = value;        
                     displayVersion();
                     saveConfig();
#if configSTRING
                     Serial.println();
#endif
                     break;

#if MESSAGING         
            case 'm': 
                     // Message storage handliing
                     // Remove a message string from RAM:
                     // messages can not be removed if queued or
                     // when any higher numbered messages are queued.
                     // TODO Bug removing entries with 131m
                     byte *fromR;
                     getMessage(255);                // Find highest message number
                     Serial.println(topMessage);
                     if ((value >= MessagesStart) && (value <= topMessage)) {
                         byte len = getMessage(value);
                         fromR = sourceR;                // Points to next message length byte, if RAM
                         if ((sourceR) && (len)) {       // Is message in RAM?
                             byte valid = true;
                             for (unsigned int i = 0; i < MAX_NODES; i++) {                    // Scan for message in use
                                 if ((semaphores[i] >= value) && (semaphores[i] <= topMessage)) {   // If so, or higher then can't
                                     showString(PSTR("In use i"));
                                     Serial.println((word) i);
                                     valid = false;
                                 }
                             }
                             displayString(&stack[sizeof stack - (len + 1)], len + 1);
                             Serial.println();
                             if ((valid) && (value <= topMessage)) {
                                 Serial.print("Copying memory len=");
                                 Serial.println(((sourceR - fromR) + 1));
                                 memcpy((fromR - (len + 1)), fromR, ((sourceR - fromR) + 1));  
                             }                      
                         } 
                     }

                     if (top) {
                         // Store a message string in RAM, to be used by the 'p' command
                         getMessage(255);    // Get pointer to end of messages
                         if ((((sourceR + 1) - &messagesR[0]) + top + 1 ) <= sizeof messagesR) {
                             *sourceR = top; // Start message, overwrite null length byte
                             memcpy((sourceR + 1), &stack, top);
                             sourceR = (sourceR + 1) + top;
                             *sourceR = 0;   // create message string terminator
                             value = ~0;
                         }
                     }             
                     if (value == 0) {
                         for (byte i = MessagesStart; i <= 254; i++) {
                             byte len = getMessage(i); 
                             if (!len) break;
                             printOneChar('m'); 
                             stack[sizeof stack - (len + 1)] = i;  // Store message number  
                             displayString(&stack[sizeof stack - (len + 1)], len + 1);
                             Serial.println();
                             if (config.output & 2) {
                                 printOneChar(' '); 
                                 displayASCII(&stack[sizeof stack - (len + 1)], len + 1);
                                 Serial.println();
                             }
                         }

                         showByte(((sourceR) - &messagesR[0]) + 1);
                         printOneChar('/');                    
                         showByte(sizeof messagesR);
                         Serial.println();
                     }

                     break;
#endif

            case 'p':
                     // Post a semaphore for a remote node, to be collected along with
                     // the next ACK. Format is 212,20,127p where 20 is the node and 212 
                     // is the group number 127 is the desired value to be posted. 
                     // The byte stack[0] contains the target group and stack[1] contains the 
                     // node number. The message string to be posted is in value
#if MESSAGING
                     if (top == 2) {
                         if (getIndex(stack[0], stack[1])) { 
                             semaphores[NodeMap] = value;
                             postingsIn++;
                             stickyGroup = stack[0];
                         }
                         Serial.println();
                     } else {
                         // Accepts a group number and prints matching entries from the eeprom
                         nodeShow(value);
                     }
#endif
                     break;

            case 'n':
                     dumpAPI();
                     dumpRegs();

                     if ((top == 0) && (config.group == 0)) {
                         showByte(stickyGroup);
                         stickyGroup = (int)value;
                         printOneChar('>');
                         showByte(stickyGroup);
                     } else if (top == 1) {
                         for (byte i = 0; i < 4; ++i) {
                             // Display eeprom byte                  
                             byte b = eeprom_read_byte((RF12_EEPROM_NODEMAP) + (value * 4) + i);
                             showByte(b);
                             if (!(config.output & 0x1)) printOneChar(' ');

                             if ((stack[0] >= 0x80) & (i == 0)) {
                                 // Set the removed flag 0x80
                                 eeprom_write_byte((RF12_EEPROM_NODEMAP) + (value * 4) + i, (b | stack[0]));
                             }
                         }
                     }


                     // Show and set RFMxx registers
                     if ((top == 2) & ((stack[0] == 128) || (stack[0] == 0))) {
                         showByte(stack[1]);
#if RF69_COMPAT
                         printOneChar('=');
                         /* Example usage: 1x        // Switch into hex input mode (optional, adjust values below accordingly)
                         //                80,29,E4n  // 0x80 (write bit) + 0x29 (RSSI Threshold) == 0xA9; E4 (default RSSI threshold); n = node command
                         //                80,11,80n  // 0x80 (write bit) + 0x11 (Output power) == 0x91; 80 (PA0 transmit power minimum); n = node command
                         //                x         // Save certain registers in eeprom and revert to decimal mode
                          */
                         showByte(RF69::control((stack[1] | stack[0]), value)); // Prints out Register value before any change requested.
                         if (stack[0] == 128) {
                             printOneChar('>');
                             showByte(value);
                         }
#else

                         Serial.print((word)(rf12_control(value)));
#endif
                     }
                     Serial.println();

                     break;

                     // the following commands all get optimised away when TINY is set 

            case 'l': // turn activity LED on or off
                     activityLed(value);
                     break;

            case 'd': // dump all log markers
                     if (df_present())
                         df_dump();
                     break;

            case 'r': // replay from specified seqnum/time marker
                     if (df_present()) {
                         word seqnum = (stack[0] << 8) | stack[1];
                         long asof = (stack[2] << 8) | stack[3];
                         asof = (asof << 16) | ((stack[4] << 8) | value);
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
                         showString(PSTR("erased\n"));
                     }
                     break;

            case 'z': // put the ATmega in ultra-low power mode (reset needed)
                     if (value == 123) {
                         clrConfig();
                         showString(PSTR(" Zzz...\n"));
                         Serial.flush();
                         rf12_sleep(RF12_SLEEP);
                         cli();
                         Sleepy::powerDown();
                     }
                     if (value == 255) {

                         //                clrNodeStore();

                         showString(PSTR("Watchdog enabled, restarting\n"));
                         WDTCSR |= _BV(WDE);
                     }
                     break;

            default:
                     showHelp();
        } // End case group

    }

    if ('a' <= c && c <= 'z' || 'R' <= c && c <= 'W' || '+' <= c && c <= '-') {
        showString(PSTR("> "));
        for (byte i = 0; i < top; ++i) {
            showByte(stack[i]);
            printOneChar(',');
        }
        Serial.print(value);
        Serial.println(c);
    }
    value = top = 0;
    if (eepromWrite) {
        showString(PSTR("Eeprom written:"));
        Serial.println(eepromWrite);
        eepromWrite = 0;
        showStatus();
    }    
} // handleInput

#if MESSAGING
static byte getMessage (byte rec) {
    if (rec < MessagesStart) return 0;
    byte len, pos;                          // Scan flash string
    sourceR = 0;                            // Not RAM!
    PGM_P sourceF = &messagesF[0];          // Start of Flash messages
    for  (pos = MessagesStart; pos < 254; pos++) {
        len = pgm_read_byte(sourceF++);
        if (!len) break;
        if (pos == rec) break; 
        sourceF = sourceF + len;
    }
    if (len) {
        // String is copied to top end of stack
        for (byte b = 0; b < len ; b++) {
            stack[(sizeof stack - (len) + b)] = pgm_read_byte(sourceF++);
        }
        //      *sourceF points to next length byte
        return len;
    } else {      // Scan RAM string
        sourceR = &messagesR[0];            // Start of RAM messages
        for  (; pos < 254; pos++) {
            len = *sourceR; 
            if (!len) {
                topMessage = pos - 1;    
                return 0;             // Not found
            }
            if (pos == rec) break; 
            sourceR = sourceR + (len + 1);  // Step past len + message
        }
    }
    for (byte b = 0; b < len ; b++) {
        // String is copied to top end of stack
        stack[(sizeof stack - (len) + b)] = *(++sourceR);
    }
    // *sourceR is pointing to the length byte of the next message
    sourceR++;  // *sourceR is now pointing to the length byte of the next message
    return len;
} // getMessage
#endif

static void dumpAPI() {
    byte* p = &rfapi.len;
    for (byte i = 0; i < sizeof rfapi; ++i) {
        Serial.print(p[i]);
        printOneChar(' ');
    }
    Serial.println();
}

static void displayString (const byte* data, byte count) {
    for (byte i = 0; i < count; ++i) {
        char c = (char) data[i];
        showByte(data[i]);
        if (!(config.output & 0x1)) printOneChar(' ');
    }
}

static void printPos (byte c) {
    if (config.output & 0x1) { // Hex output?
        printOneChar(' ');
    } else {
        if (c > 99) printOneChar(' ');
        if (c > 9) printOneChar(' ');
    }
}

static void printASCII (byte c) {
    char d = (char) c;
    printPos((byte) c);
    printOneChar(d < ' ' || d > '~' ? '.' : d);
    if (!(config.output & 0x1)) printOneChar(' ');
}       

static void displayASCII (const byte* data, byte count) {
    for (byte i = 0; i < count; ++i) {
        //        if (config.output & 0x1) printOneChar(' ');
        byte c = data[i]; 
        printASCII(c);        
    }
}

static int freeRam () {
    extern int __heap_start, *__brkval; 
    int v; 
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

#if !TINY
uint8_t resetFlags __attribute__ ((section(".noinit")));
void resetFlagsInit(void) __attribute__ ((naked)) __attribute__ ((section (".init0")));
void resetFlagsInit(void)
{
    // save the reset flags passed from the bootloader
    __asm__ __volatile__ ("mov %0, r2\n" : "=r" (resetFlags) :);
}
#endif

void setup () {

    delay(10000);

    //  clrConfig();

#if TINY
    PCMSK0 |= (1<<PCINT2);  // tell pin change mask to listen to PA2
    GIMSK |= (1<<PCIE0);    // enable PCINT interrupt in general interrupt mask
    whackDelay(BITDELAY*2); // if we were low this establishes the end
    pinMode(_receivePin, INPUT);        // PA2 - doesn't work if before the PCMSK0 line
    digitalWrite(_receivePin, HIGH);    // pullup!

#endif
    Serial.begin(SERIAL_BAUD);
    displayVersion();
#if LED_PIN == 8
    showString(BLOC);
#endif
#if DEBUG
    //    dumpRegs();
    //    dumpEEprom();
#endif

#ifndef PRR
#define PRR PRR0
#endif   
    ACSR &= (1<<ACIE);      // Disable Analog Comparator Interrupt
    ACSR |= (1<<ACD);       // Disable Analog Comparator
    ADCSRA &= ~ bit(ADEN);  // disable the ADC
    // Switch off some unused hardware
//	PRR |= (1 << PRTIM1) | (1 << PRADC);
    PRR |= (1 << PRADC);
#if defined PRTIM2
    PRR |= (1 << PRTIM2);
#endif
#if defined PRR2
    PRR1 |= (1 << PRTIM3);  // 1284P
#endif

// Set up timer1 interrupt at 1Hz
  	TCCR1A = 0;								// Set TCCR1A to 0
  	TCCR1B = 0;								// Same for TCCR1B
  	TCNT1  = 0;								// Counter value to 0
  	// 1hz increments
  	OCR1A = 15624;							// = (16*10^6) / (1*1024) - 1 (must be <65536)
  
  	TCCR1B |= (1 << WGM12);					// Activate CTC mode
  
  	TCCR1B |= (1 << CS12) | (1 << CS10);	// Set 1024 prescaler  
  
  	TIMSK1 |= (1 << OCIE1A);				// Timer1 compare interrupt

    // Consider adding the following equivalents for RFM12x
    /*
#if !TINY
showNibble(resetFlags >> 4);
showNibble(resetFlags);
    // TODO the above doesn't do what we need, results vary with Bootloader etc
#endif
     */

#if MESSAGING && !TINY
    // messagesR = 0x05, 'T', 'e', 's', 't', '3'; // TODO    // Can be removed from RAM with "129m"
    messagesR[messageStore] = 0;                             // Save flash, init here: Mandatory delimiter
#endif

#if RF69_COMPAT && STATISTICS
    // Initialise min/max/count arrays
    for (int i = 0; i < MAX_NODES; i++) {
        minFEI[i] = 32767;
        maxFEI[i] = -32767;
    }
    //memset(minFEI,32767,sizeof(minFEI));
    //
    //memset(maxFEI,-32767,sizeof(maxFEI));
    memset(minRSSI,255,sizeof(minRSSI));
    memset(maxRSSI,0,sizeof(maxRSSI));
    memset(minLNA,255,sizeof(minLNA));
    memset(maxLNA,0,sizeof(maxLNA));
#endif
#if STATISTICS
    memset(pktCount,0,sizeof(pktCount));
#endif

#if JNuMOSFET     // Power up the wireless hardware
    bitSet(DDRB, 0);
    bitClear(PORTB, 0);
    delay(1000);
#endif
    if (rf12_configSilent()) {
        loadConfig();
    } else {
        dumpEEprom();
        showString(INITFAIL);
        memset(&config, 0, sizeof config);
        config.nodeId = 0x9F;       // 868 MHz, node 31
        config.group = 0x00;        // Default group 0
        config.frequency_offset = 1600;
        config.collect_mode = true; // Default to no-ACK
        config.quiet_mode = true;   // Default flags, quiet on
        config.defaulted = true;    // Default config initialized
        config.ackDelay = 0;
#if RF69_COMPAT == 0
        config.RegRssiThresh = 2;
        config.clearAir = 160;      // 80dB
#endif
        saveConfig();
        WDTCSR |= _BV(WDE);			// Trigger watchdog restart
        //        if (!(rf12_configSilent()))
        //          showString(INITFAIL);

    }

    stickyGroup = config.group;
    if (!(stickyGroup)) stickyGroup = 212;

    df_initialize();

#if !TINY
    //    showHelp();
    showStatus();
    unsigned int a = ((SPCR | (SPSR << 2)) & 7);    
    if (a != 4) {    // Table 18.5 Relationship Between SCK and the Oscillator Frequency
        showString(PSTR(" SPI="));
        Serial.println(a); 
    }
#endif
    Serial.flush();
} // setup

static void clrConfig() {
    // Clear Config eeprom
    showString(PSTR("Clearing Config\n"));
    for (byte i = 0; i < sizeof config; i++) {
        eeprom_write_byte((RF12_EEPROM_ADDR) + i, 0xFF);
    }
}

static void clrNodeStore() {
    // Clear Node Store eeprom
    showString(PSTR("Clearing NodeStore\n"));
    for (unsigned int n = 0; n < 2016; n++) {
        eeprom_write_byte((RF12_EEPROM_NODEMAP) + n, 0xFF);
    }
}

/// Display eeprom configuration space
static void dumpEEprom() {
    Serial.println("\n\rConfig eeProm:");
    uint16_t crc = ~0;
    for (byte i = 0; i < (RF12_EEPROM_SIZE); ++i) {
        byte d = eeprom_read_byte(RF12_EEPROM_ADDR + i);
        showNibble(d >> 4); showNibble(d);
        crc = _crc16_update(crc, d);
    }
    if (crc) {
        Serial.print(" BAD CRC ");
        Serial.println(crc, HEX);
    }
    else Serial.print(" GOOD CRC ");
}

//#if DEBUG && RF69_COMPAT
/// Display the RFM69x registers
static void dumpRegs() {
    Serial.println("\nRFM69x Registers:");
    for (byte r = 1; r < 0x80; ++r) {
        showByte(RF69::control(r, 0)); // Prints out Radio Registers.
        printOneChar(',');
        delay(2);
    }
    Serial.println();
    delay(10);
}
//#endif
/// Display stored nodes and show the post queued for each node
/// the post queue is not preserved through a restart of RF12Demo
static void nodeShow(byte group) {
    unsigned int index;
    for (index = 0; index < MAX_NODES; index++) {
        byte n = eeprom_read_byte((RF12_EEPROM_NODEMAP) + (index * 4));     // Node number
http://forum.arduino.cc/index.php/topic,140376.msg1054626.html
        if (n & 0x80) {                                                     // Erased or empty entry?
            if (n == 0xFF) break;                                           // Empty, assume end of table
            if (!group) {
                Serial.println(index); 
            }         
        } else {
            byte g = eeprom_read_byte((RF12_EEPROM_NODEMAP) + (index * 4) + 1);  // Group number
            if (!group || group == g) {          
                Serial.print(index);
                showString(PSTR(" g"));      
                showByte(g);
                showString(PSTR(" i"));      
                showByte(n & RF12_HDR_MASK);
#if STATISTICS      
                showString(PSTR(" rx:"));
                Serial.print(pktCount[index]);
#endif
#if MESSAGING 
                showString(PSTR(" post:"));      
                showByte(semaphores[index]);
#endif
#if RF69_COMPAT && STATISTICS            
                if (pktCount[index]) {
                    showString(PSTR(" fei("));
                    Serial.print(minFEI[index]);
                    printOneChar('/');
                    Serial.print(maxFEI[index]);
                    printOneChar('/');
                    Serial.print(-((minFEI[index] + maxFEI[index]) / 2));
                    showString(PSTR(") rssi("));
                    showByte(minRSSI[index]);
                    printOneChar('/');
                    showByte(eeprom_read_byte((RF12_EEPROM_NODEMAP) + (index * 4) + 2)); // Show original RSSI value
                    printOneChar('/');
                    showByte(lastRSSI[index]);
                    printOneChar('/');
                    showByte(maxRSSI[index]);
                    showString(PSTR(") lna("));
                    Serial.print(minLNA[index]);
                    printOneChar('/');
                    showByte(eeprom_read_byte((RF12_EEPROM_NODEMAP) + (index * 4) + 3)); // Show original LNA value
                    printOneChar('/');
                    Serial.print(lastLNA[index]);
                    printOneChar('/');
                    Serial.print(maxLNA[index]);
                    printOneChar(')');
                }
#endif
                Serial.println();
            }
        }
    }
#if MESSAGING    
    showString(PSTR("Postings "));      
    Serial.print((word) postingsIn);
    printOneChar(',');
    Serial.println((word) postingsOut);
#endif
#if RF69_COMPAT && STATISTICS
    showString(PSTR("Stability "));
    if (changedAFC) {
        Serial.print(word((messageCount + CRCbadCount) / changedAFC));    
        printOneChar(',');
    }
    if (changedFEI) {
        Serial.print(word((messageCount + CRCbadCount) / changedFEI));  
        printOneChar(' ');
    }
    Serial.print(word(changedAFC));    
    printOneChar(',');
    Serial.println(word(changedFEI));
    Serial.println(RF69::control(REG_SYNCCONFIG, 0));   
#endif  
#if STATISTICS
    Serial.print(messageCount);
    printOneChar('(');
    Serial.print(CRCbadCount);
    printOneChar(')');
    Serial.print(nonBroadcastCount);
    printOneChar('@');
    Serial.print((millis() >> 10));  // An approximation to seconds
    printOneChar(':');
    Serial.print(qMin); printOneChar('~'); Serial.print(qMax); printOneChar('%');
    printOneChar(' ');
#endif
#if RF69_COMPAT && STATISTICS
    if (CRCbadMaxRSSI) {
        printOneChar('>');
        Serial.print(CRCbadMinRSSI);    
        printOneChar('<');
        Serial.print(CRCbadMaxRSSI);
        printOneChar(' ');
    }
    Serial.print(RF69::interruptCount);
    printOneChar('(');
    Serial.print(RF69::rxP);
    printOneChar(',');
    Serial.print(RF69::txP);
    printOneChar(',');
    Serial.print(RF69::discards);
    printOneChar(',');
    Serial.print(RF69::byteCount);         // Length of previous packet
    printOneChar(',');
    Serial.print((RF69::payloadLen));      // Length of previous payload
    printOneChar(',');
    printOneChar('{');
    Serial.print(RF69::badLen);            // Invalid payload lengths detected 
    printOneChar('}');
    printOneChar(',');
    Serial.print((RF69::packetShort));     // Packet ended short
    printOneChar(',');
    printOneChar('[');
    Serial.print(RF69::unexpected);
    printOneChar(',');
    Serial.print(RF69::unexpectedFSM);
    printOneChar(',');
    Serial.print(RF69::unexpectedIRQFLAGS2);
    printOneChar(',');
    Serial.print(RF69::unexpectedMode);
    printOneChar(']');
    printOneChar(',');
    //    Serial.print(RF69::nestedInterrupts);
    printOneChar(',');
    Serial.print(RF69::IRQFLAGS2);
    printOneChar(',');
    Serial.print(RF69::DIOMAPPING1);
    printOneChar(',');
    Serial.print(RF69::underrun);
    printOneChar(')');
#endif
    Serial.println();
    Serial.println(freeRam());
    Serial.print(testTX);
    printOneChar('-');
    Serial.print(busyCount);
    printOneChar('=');
    Serial.println(testTX - busyCount);
    Serial.print(testRX);
    printOneChar('+');
    Serial.print(missedTests);
    printOneChar('=');
    Serial.println(testRX + missedTests);
    busyCount = missedTests = testTX = testRX = testCounter = lastTest = 0;
    idleTime = loopCount = 0;
} // nodeShow
static unsigned int getIndex (byte group, byte node) {
    newNodeMap = NodeMap = 0xFFFF;
    // Search eeprom RF12_EEPROM_NODEMAP for node/group match
    for (unsigned int index = 0; index < MAX_NODES; index++) {
        byte n = eeprom_read_byte((RF12_EEPROM_NODEMAP) + (index * 4));
        //              http://forum.arduino.cc/index.php/topic,140376.msg1054626.html
        if (n & 0x80) {                                     // Erased (0xFF) or empty (0x80) entry?
            if (newNodeMap == 0xFFFF) newNodeMap = index;   // Save pointer to a first free entry
            if (n == 0xFF) return(false);                   // Erased, assume end of table!
        } else {
            if ((n & RF12_HDR_MASK) == (node & RF12_HDR_MASK)) {  // Node match?
                byte g = eeprom_read_byte((RF12_EEPROM_NODEMAP) + (index * 4) + 1);
                if (g == group) {                                 // Group match?
                    // found a match;
                    NodeMap = index;
                    return (true);
                }
            }
        }
    } 
    return(false);
}
void loop () {
#if TINY
    if (_receive_buffer_index) {
        handleInput(inChar());
    }
#else

    if (Serial.available())
        handleInput(Serial.read());
#endif
    if (rf12_recvDone()) {

#if RF69_COMPAT && !TINY                // At this point the radio is in Standby

        rf12_recvDone();                // Attempt to buffer next RF packet
        // At this point the receiver is active
        
 		unsigned long rxGap = rf12_interpacketTS - rxLast;
 		rxLast = rf12_interpacketTS;
 		if (rxGap < minGap) minGap = rxGap;
 		if (rxGap > maxGap) maxGap = rxGap;
 		
        observedRX.afc = rf12_afc;
        observedRX.fei = rf12_fei;
        observedRX.rssi2 = rf12_rssi;
        observedRX.lna = rf12_lna >> 3;

        if ((observedRX.afc) && (observedRX.afc != previousAFC)) { // Track volatility of AFC
            changedAFC++;    
            previousAFC = observedRX.afc;
        }
        if (observedRX.fei != previousFEI) {            // Track volatility of FEI
            changedFEI++;
            previousFEI = observedRX.fei;
        }
#endif  
        byte n = rf12_len;
        byte crc = false;


        if (rf12_crc == 0) {
#if STATISTICS && !TINY
            messageCount++;                             // Count a broadcast packet
#endif
            goodCRC++;
            if ((watchNode) && ((rf12_hdr & RF12_HDR_MASK) != watchNode)) return;
            showString(PSTR("OK"));
            crc = true;
        } else {
            activityLed(1);
#if STATISTICS && !TINY
            CRCbadCount++;
#endif
#if RF69_COMPAT && STATISTICS && !TINY
            if (observedRX.rssi2 < (CRCbadMinRSSI))
                CRCbadMinRSSI = observedRX.rssi2;   
            if (observedRX.rssi2 > (CRCbadMaxRSSI))
                CRCbadMaxRSSI = observedRX.rssi2;   
#endif            
            activityLed(0);

#if !TINY
            if(rf12_buf[0] == 212 && (rf12_buf[1] | rf12_buf[2]) == rf12_buf[3] && (salusMode)){
                Serial.print((word) SalusFrequency, DEC);  
                showString(PSTR(" Salus I Channel "));
                showByte(rf12_buf[1]);
                printOneChar(':');
                showByte(rf12_buf[2]);
                Serial.println();
                //                return;
                n = RF69::byteCount - 3;
            }            
            if(rf12_buf[0] == 212 && rf12_buf[1] >= 160 && (salusMode)) {
                Serial.print((word) SalusFrequency, DEC);  
                showString(PSTR(" Salus II Device:"));
                Serial.print(rf12_buf[1]);

                showString(PSTR(" Addr:"));
                unsigned int addr = (rf12_buf[3] << 8) | rf12_buf[2];   // Guessing at a 16 bit address
                Serial.print(addr);
                showString(PSTR(" Type:"));
                Serial.print(rf12_buf[4]);
                switch (rf12_buf[1]) {
                    case 165: // Thermostat
                        printOneChar(' ');
                        Serial.print((rf12_buf[6] << 8) | rf12_buf[5]);
                        printOneChar(':');
                        Serial.print((rf12_buf[8] << 8) | rf12_buf[7]);
                        printOneChar(':');
                        Serial.print((rf12_buf[10] << 8) | rf12_buf[9]);
                        printOneChar(':');
                        Serial.print(((rf12_buf[12] << 8) | rf12_buf[11]), HEX);
                        break;
                    case 166:   // OTO One Touch Override
                        printOneChar(' ');
                        Serial.print(rf12_buf[5]);
                        break;
                    default:
                        showString(PSTR(" Unknown"));
                        break;
                }

                Serial.println();
                //                return;
            }            
#endif
#if RF69_COMPAT
            if ((rf12_hdr &  ~RF12_HDR_MASK) == (RF12_HDR_DST | RF12_HDR_ACK) && 
                    ((rf12_hdr &  RF12_HDR_MASK) > 23) && (config.group != 0)) {
                showString(UNSUPPORTED);
                showString(RFM69x);
                // RFM69 radio problem
                //        		Serial.println();
            }  
#endif

            if (config.quiet_mode) return;

            crc = false;
            showString(PSTR("   ?"));
            n = n + 2;	// Include potential CRC
            if (n > 16) n = 16;
        }

        if (config.output & 0x1)
            printOneChar('X');
        // Compatibility with HouseMon v0.7.0     else printOneChar(' ');
        if (config.group == 0) {
            showString(PSTR(" G"));
            showByte(rf12_grp);
        } else if (!crc) {
            showByte(rf12_grp);
        }
        printOneChar(' ');
        showByte(rf12_hdr);

        if (!crc) {
            if (!(config.output & 1))
                printOneChar(' ');
            showByte(rf12_len);
        }

        byte testPacket = false;
        if (n == 66) { // Is it a test packet
            testPacket = true;
            for (byte b = 0; b < 65; b++) {
                // TODO if ((((rf12_data[b]) + 1) & 255) != rf12_data[b + 1]) 
                if ((byte) (rf12_data[b] + 1) != rf12_data[b + 1]) {
                    testPacket = false;
                }
            }
        }
        if (testPacket) {        
            testRX++;
            showString(PSTR(" t")); // Abbreviate Test string
            showByte(rf12_data[0]);
            byte n = rf12_data[0] - (lastTest + 1);
            if (n) {
                printOneChar('-');
                showByte(n);
                missedTests =+ n;
            }
            lastTest = rf12_data[0];
        } else {
            for (byte i = 0; i < n; ++i) {
                if (!(config.output & 1)) // Decimal output?
                    printOneChar(' ');
                showByte(rf12_data[i]);
            }
        }
#if RF69_COMPAT && !TINY
        if ((config.verbosity & 1) || (!crc)) {
            showString(PSTR(" a="));
            Serial.print(observedRX.afc);                      // TODO What units has this number?
            showString(PSTR(" f="));
            Serial.print(observedRX.fei);                      // TODO What units has this number?
            /*
               LNA gain setting:
               000 gain set by the internal AGC loop
               001 G1 = highest gain
               010 G2 = highest gain – 6 dB
               011 G3 = highest gain – 12 dB
               100 G4 = highest gain – 24 dB
               101 G5 = highest gain – 36 dB
               110 G6 = highest gain – 48 dB
             */            
            showString(PSTR(" l="));
            Serial.print(observedRX.lna);

            showString(PSTR(" t="));
            Serial.print((RF69::readTemperature(-10)));        

            if ((CRCbadCount + 1) && (messageCount + 1) && (nonBroadcastCount + 1)) {
                showString(PSTR(" q="));
                unsigned long v = (messageCount + nonBroadcastCount);
                byte q = ((v * 100) / (CRCbadCount + v));
                Serial.print(q);
                if ((messageCount + nonBroadcastCount + CRCbadCount) > 100) {
                    if (q < qMin) qMin = q;
                    if (q > qMax) qMax = q;
                }
                printOneChar('%');
            } else {    // If we overflow then clear them all.
                showString(PSTR(" Reset "));
                CRCbadCount = messageCount = nonBroadcastCount = 0;
            }

            if (rf12_sri) {
                showString(PSTR(" Rs="));
                Serial.print(rf12_drx);
                printOneChar('/');                
                // display RSSI at the start of RX phase value
                if (config.output & 0x1)                  // Hex output?
                    showByte(rf12_sri);
                else {
                    Serial.print(rf12_sri >> 1);
                    if (rf12_sri & 0x01) showString(PSTR(".5"));
                    showString(PSTR("dB"));
                }
            }

            showString(PSTR(" d=("));
            Serial.print(rf12_tfr);
            printOneChar(')');
            Serial.print(rf12_rtp);

            showString(PSTR(" r="));
            Serial.print(rf12_rst);
            showString(PSTR(" i="));
            Serial.print(rxGap);
            /*
               showString(PSTR(" M="));
               Serial.print(RF69::REGIRQFLAGS1, HEX);
             */            
            /*
               showString(PSTR(" Ra="));
            // display RSSI at the end of TX phase value
            if (config.output & 0x1)                  // Hex output?
            showByte(rssiEndRX2);
            else {
            Serial.print(rssiEndRX2 >> 1);
            if (rssiEndRX2 & 0x01) showString(PSTR(".5"));
            showString(PSTR("dB"));
            }

            showString(PSTR(" L="));
            Serial.print(RF69::byteCount);  // Length of packet
            RF69::byteCount = 0;  // DEBUG    */                
        }

        // display RSSI value after packet data
        showString(PSTR(" ("));
        if (config.output & 0x1)                  // Hex output?
            showByte(observedRX.rssi2);
        else {
            Serial.print(observedRX.rssi2 >> 1);
            if (observedRX.rssi2 & 0x01) showString(PSTR(".5"));
            showString(PSTR("dB"));
        }
        printOneChar(')');
        //        Serial.print((RF69::control(9,0)), HEX);  // LSB of frequency
#if DEBUG
        showString(PSTR(" mCR1="));
        Serial.print(modeChange1);
        showString(PSTR(" mCT2="));
        Serial.print(modeChange2);
        showString(PSTR(" mCs3="));
        Serial.print(modeChange3);
#endif
#endif        
        if (config.verbosity & 2) {
            if(!(crc)) showString(PSTR(" Bad"));
            if (!(rf12_hdr & 0xA0)) showString(PSTR(" Packet "));
            else showString(PSTR(" Ack "));
            if (rf12_hdr & 0x20) showString(PSTR("Requested "));
            if (rf12_hdr & 0x80) showString(PSTR("Reply "));
            if (rf12_hdr & 0x40) showString(PSTR("to i"));
            else showString(PSTR("from i"));
            Serial.print(rf12_hdr & RF12_HDR_MASK);
            showString(PSTR(" len "));
            Serial.print(rf12_len);
#if RF69_COMPAT                        
            if(rf12_len != rfapi.lastLen) {
                printOneChar('(');
                Serial.print(rfapi.lastLen);	// Show actual length received
                printOneChar(')');
            }
#endif
            /*            printOneChar(' ');
                          Serial.print(RF69::rssiChanged);  printOneChar('/'); Serial.print(RF69::lastState);
                          printOneChar('/'); Serial.print(RF69::countRSSI);
                          printOneChar('/'); Serial.print(RF69::interruptRSSI);
             */
        }
        /*
#else
Serial.print(" (");
Serial.print(rf12_getRSSI());
Serial.print(")");
         */
        Serial.println();

#if !TINY
        if (config.output & 0x2) { // also print a line as ascii
            showString(PSTR("ASC"));                         // 'OK'
            if (crc) {
                //                printOneChar(' ');                           // ' '
                if (config.group == 0) {
                    printOneChar(' ');                       // 'G'
                    printASCII(rf12_grp);                    // grp
                    if (config.output & 1) printOneChar(' ');
                }
                printOneChar(rf12_hdr & RF12_HDR_DST ? '>' : '<');
                if ((rf12_hdr > 99) && (!(config.output & 1))) printPos(' ');
                printOneChar('@' + (rf12_hdr & RF12_HDR_MASK));
                if (!(config.output & 1)) printOneChar(' ');
            } else {
                printOneChar('?');                       // '?'
                if (config.output & 1) {
                    printOneChar('X');                   // 'X'
                } else {
                    printOneChar(' ');                    // ''
                }  
                if (config.group == 0) {
                    printOneChar('G');                   // 'G'
                }
                printASCII(rf12_grp);                    // grp
                if (config.output & 1) {
                    printOneChar(' ');                  // ' '
                }
                if (config.group == 0) {
                    printOneChar(' ');                   // ' '
                }
                printASCII(rf12_hdr);      // hdr
                printASCII(rf12_len);      // len
            }
            if (testPacket) {
                showString(PSTR("t")); // Abbreviate Test string
                showByte(rf12_data[1]);
                displayASCII((const byte*) rf12_data, 1);
            } else {
                displayASCII((const byte*) rf12_data, n);
            }
            Serial.println();
        }
#endif
        if (rf12_crc == 0) {
            byte crlf = false;
            activityLed(1);

            if (df_present())
                df_append((const char*) rf12_data - 2, rf12_len + 2);

            if (!(rf12_hdr & RF12_HDR_DST)) {
                // This code only sees broadcast packets *from* another nodes.
                // Packets addressed to nodes do not identify the source node!          
                // Search RF12_EEPROM_NODEMAP for node/group match
                // Node 31 will also be added even though a Node Allocation will
                // be offered, to track everyone who was out there.
#if !TINY           
                if (!getIndex(rf12_grp, (rf12_hdr & RF12_HDR_MASK)) && (!(testPacket))) {
                    if (newNodeMap != 0xFFFF) { // Storage space available?
                        // Node 31 will also be added even though a Node Allocation will
                        // also be offered, to track everyone who is/was out there.
                        showString(PSTR("New Node g"));
                        showByte(rf12_grp);
                        showString(PSTR(" i"));
                        showByte(rf12_hdr & RF12_HDR_MASK);
                        showString(PSTR(" Index "));
                        Serial.println(newNodeMap);
                        eeprom_write_byte((RF12_EEPROM_NODEMAP) + (newNodeMap * 4), (rf12_hdr & RF12_HDR_MASK));  // Store Node and
                        eeprom_write_byte(((RF12_EEPROM_NODEMAP) + (newNodeMap * 4) + 1), rf12_grp);              // and Group number
#if RF69_COMPAT
                        eeprom_write_byte(((RF12_EEPROM_NODEMAP) + (newNodeMap * 4) + 2), observedRX.rssi2);      //  First RSSI value
                        eeprom_write_byte(((RF12_EEPROM_NODEMAP) + (newNodeMap * 4) + 3), observedRX.lna);        //  First LNA value
#endif
                        NodeMap = newNodeMap;
                        newNodeMap = 0xFFFF;
                    } else {
                        showString(PSTR("Node table full g"));
                        showByte(rf12_grp);
                        showString(PSTR(" i"));
                        showByte(rf12_hdr & RF12_HDR_MASK);
                        showString(PSTR(" not saved"));
                        Serial.println(); 
                    }
                }
#endif // !TINY

#if RF69_COMPAT && STATISTICS
                // Check/update to min/max/count
                if (observedRX.lna < (minLNA[NodeMap]))       
                    minLNA[NodeMap] = observedRX.lna;
                lastLNA[NodeMap] = observedRX.lna;   
                if (observedRX.lna > (maxLNA[NodeMap]))
                    maxLNA[NodeMap] = observedRX.lna;   

                if (rf12_fei < (minFEI[NodeMap]))       
                    minFEI[NodeMap] = rf12_fei;
                if (rf12_fei > (maxFEI[NodeMap]))
                    maxFEI[NodeMap] = rf12_fei;   

                if (observedRX.rssi2 < (minRSSI[NodeMap]))
                    minRSSI[NodeMap] = observedRX.rssi2;
                lastRSSI[NodeMap] = observedRX.rssi2;   
                if (observedRX.rssi2 > (maxRSSI[NodeMap]))
                    maxRSSI[NodeMap] = observedRX.rssi2;   
#endif
#if STATISTICS            
                pktCount[NodeMap]++;
            } else {
                nonBroadcastCount++;
#endif
            }

            // Where requested, acknowledge broadcast packets - not directed packets
            // unless directed to this nodeId
            if ((RF12_WANTS_ACK && (config.collect_mode) == 0) && (!(rf12_hdr & RF12_HDR_DST))             
                    || (rf12_hdr & (RF12_HDR_MASK | RF12_HDR_ACK | RF12_HDR_DST)) 
                    == ((config.nodeId & 0x1F) | RF12_HDR_ACK | RF12_HDR_DST)) {

                byte ackLen = 0;
				byte special = false;
                // This code is used when an incoming packet requesting an ACK is also from Node 31
                // The purpose is to find a "spare" Node number within the incoming group and offer 
                // it with the returning ACK.
                // If there are no spare Node numbers nothing is offered
                // TODO perhaps we should increment the Group number and find a spare node number there?
                if (((rf12_hdr & RF12_HDR_MASK) == 31) && (!(rf12_hdr & RF12_HDR_DST)) && (!(testPacket))) {
                	special = true;
                    // Special Node 31 source node
                    // Make sure this nodes node/group is already in the eeprom
                    if (((getIndex(config.group, config.nodeId))) && (newNodeMap != 0xFFFF)) {   
                        // node/group not found but there is space to save
                        eeprom_write_byte((RF12_EEPROM_NODEMAP) + (newNodeMap * 4), (config.nodeId & RF12_HDR_MASK));
                        eeprom_write_byte(((RF12_EEPROM_NODEMAP) + (newNodeMap * 4) + 1), config.group);
                        eeprom_write_byte(((RF12_EEPROM_NODEMAP) + (newNodeMap * 4) + 2), 255);
                    }
#if NODE31ALLOC                    
                    for (byte i = 1; i < 31; i++) {
                        // Find a spare node number within received group number
                        if (!(getIndex(rf12_grp, i ))) {         // Node/Group pair not found?
                            observedRX.offset_TX = config.frequency_offset;
#if RF69_COMPAT                       
                            observedRX.RegPaLvl_TX = RF69::control(0x11, 0x9F);    // Pull the current RegPaLvl from the radio
                            observedRX.RegTestLna_TX = RF69::control(0x58, 0x1B);  // Pull the current RegTestLna from the radio
                            observedRX.RegTestPa1_TX = RF69::control(0x5A, 0x55);  // Pull the current RegTestPa1 from the radio
                            observedRX.RegTestPa2_TX = RF69::control(0x5C, 0x70);  // Pull the current RegTestPa2 from the radio
#endif

                            ackLen = (sizeof observedRX) + 1;
                            stack[sizeof stack - ackLen] = i + 0xE0;  // 0xE0 is an arbitary value
                            // Change Node number request - matched in RF12Tune
                            byte* d = &stack[sizeof stack];
                            memcpy(d - (ackLen - 1), &observedRX, (ackLen - 1));
                            showString(PSTR("Node allocation "));
                            //                            crlf = true;
                            showByte(rf12_grp);
                            printOneChar('g');
                            printOneChar(' ');
                            showByte(i);        
                            printOneChar('i');
                            //                            printOneChar(' ');
                            break;
                        }                            
                    }
                    if (!ackLen) {
                        showString(PSTR("No free node numbers in "));
                        //                        crlf = true;
                        showByte(rf12_grp);
                        printOneChar('g');
                    }
                    Serial.println();                    
                }
#endif                    
                crlf = true;
                delay(config.ackDelay);          // changing into TX mode is quicker than changing into RX mode for RF69.     
                showString(PSTR("TX "));
                byte r = rf12_canSend(config.clearAir);
                if (r) {
#if RF69_COMPAT && !TINY
                    Serial.print(rfapi.sendRSSI);
                    if (rfapi.sendRSSI < minTxRSSI) minTxRSSI = rfapi.sendRSSI;
                    if (rfapi.sendRSSI > maxTxRSSI) maxTxRSSI = rfapi.sendRSSI;
#endif            
                    showString(PSTR(" -> ack "));
                    if (testPacket) {  // Return test packet number being ACK'ed
                        stack[(sizeof stack - 2)] = 0x80;
                        stack[(sizeof stack - 1)] = rf12_data[0];
                        ackLen = 2;
                    }
#if RF69_COMPAT && !TINY
                    if (config.group == 0) {
                        showString(PSTR("g"));
                        showByte(rf12_grp);
                        RF69::control(REG_SYNCGROUP | 0x80, rf12_grp); // Reply to incoming group number
                        printOneChar(' ');
                    }
#endif
                    printOneChar('i');
                    showByte(rf12_hdr & RF12_HDR_MASK);                    
#if MESSAGING
	                // This code is used when an incoming packet is requesting an ACK, it determines if a semaphore is posted for this Node/Group.
    	            // If a semaphore exists it is stored in the buffer. If the semaphore has a message addition associated with it then
        	        // the additional data from the message store is appended to the buffer and the whole buffer transmitted to the 
            	    // originating node with the ACK.
                
                	if (semaphores[NodeMap] && (!(special))) {				// Something to post?
                    	stack[sizeof stack - 1] = semaphores[NodeMap];		// Pick up message pointer
	                    ackLen = getMessage(stack[sizeof stack - 1]);		// Check for a message to be appended
    	                if (ackLen){
        	                stack[(sizeof stack - (ackLen + 1))] = semaphores[NodeMap];
            	        }
                	    ackLen++;                                                    // If 0 or message length then +1 for length byte
                    	showString(PSTR(" Posted "));
	                    if (rf12_data[0] == semaphores[NodeMap]) {  // Check if previous Post value is the first byte of this payload 
    	                    semaphores[NodeMap] = 0;                // Indicating it was received
        	                showString(PSTR("and cleared "));
            	        }
                    	crlf = true;
                    	displayString(&stack[sizeof stack - ackLen], ackLen);        // 1 more than Message length!                      
                    	postingsOut++;
                	}
#endif                                        
                    rf12_sendStart(RF12_ACK_REPLY, &stack[sizeof stack - ackLen], ackLen);
                    rf12_sendWait(1);
                    
                } else {
                    packetAborts++;   
            		Serial.print(rfapi.sendRSSI);
                    showString(ABORTED);		// Airwaves busy, drop ACK and look for a retransmission.
                    showByte(packetAborts);
                    printOneChar(' ');
#if RF69_COMPAT && !TINY
                    Serial.print(minTxRSSI);
                    printOneChar(' ');
                    Serial.print(maxTxRSSI);
                    printOneChar(' ');
#endif
                    Serial.print(rfapi.rxfill);
                    printOneChar(' ');
                    Serial.print(rfapi.rxdone);
                }
            }
            if (crlf) Serial.println();

            activityLed(0);
        }
    } // rf12_recvDone
#if RF69_COMPAT && !TINY    
    else { 

        if (rfapi.RSSIrestart != lastRSSIrestart) {

            if (ledStatus) activityLed(0);
            else activityLed(1);

            if (config.verbosity & 4) {
                // RX restart 10 2 202 0 0 -70 1 200
                // RX restart 5334 0 217 1 0 0 1 208
                //			 r	 r rss d
                showString(PSTR("RX restart "));
                Serial.print(rfapi.RSSIrestart);
                printOneChar(' ');
                Serial.print(rfapi.rssiThreshold);
                printOneChar(' ');
                Serial.print(restartRate);
                printOneChar(' ');
                Serial.print(rf12_drx);
//                printOneChar(' ');
                showString(PSTR(" afc="));
                Serial.print(RF69::afc);
//                printOneChar(' ');
                showString(PSTR(" fei="));
                Serial.print(RF69::fei);
//                printOneChar(' ');
                showString(PSTR(" lna="));
                Serial.print((RF69::lna >> 3));
//                printOneChar(' ');
                showString(PSTR(" rssi=="));
                Serial.print(RF69::rssi);
                printOneChar(' ');
                unsigned long m = millis();
                Serial.print(m - rfapi.interpacketTS);
                printOneChar(' ');
				Serial.println(m);
//				Serial.println();                  
            }
        }
        if (rfapi.rssiThreshold != lastrssiThreshold) {
            if (config.verbosity & 8) {
                showString(PSTR("RX threshold change "));
                Serial.print(lastrssiThreshold);
                printOneChar(' ');
                Serial.print(rfapi.rssiThreshold);
                printOneChar(' ');
                Serial.print(rfapi.RSSIrestart);
                printOneChar(' ');
                Serial.print(rfapi.syncMatch);
                printOneChar(' ');
                Serial.print(goodCRC);
                printOneChar(' ');

                //                  Serial.print(rfapi.RSSIrestart  - lastThresholdRSSIrestart);
                //                  lastThresholdRSSIrestart = lastRSSIrestart;
                //                  printOneChar(' ');
                Serial.print(restartRate);
                printOneChar(' ');
                Serial.print(maxRestartRate);
                printOneChar(' ');
                Serial.print(millis());
                /*
                   printOneChar('*');
                   Serial.print(rfapi.setmode);
                   printOneChar('*');
                   Serial.print(rfapi.irqflags1);
                   printOneChar('*');
                   Serial.print(rfapi.mode);
                 */
                Serial.println();
            }
            lastrssiThreshold = rfapi.rssiThreshold;     
        }
        lastRSSIrestart = rfapi.RSSIrestart;
    } // rf12_recvDone
#endif    
    byte r;
    sendRetry = 0;
    if (cmd) {
        r = rf12_canSend(config.clearAir);
        if (r) {
            activityLed(1);
            showString(PSTR("TX "));
            Serial.print(r);
            showString(PSTR(" -> "));
            showByte(sendLen);
            showString(PSTR(" b\n"));
            byte header = (cmd == 'a' ? RF12_HDR_ACK : 0);
            if (dest)
                header |= RF12_HDR_DST | dest;
#if RF69_COMPAT && !TINY
            if (config.group == 0) {
                //                RF69::control(REG_SYNCGROUP | 0x80, stickyGroup);  // Set a group number to use for transmission
            }
#endif
            rf12_sendStart(header, stack, sendLen);
#if DEBUG            
            for (byte i = 0; i < rf12_len + 3; i++) {;
                showByte(rf12_data[i]);
                printOneChar(' ');
            }
            Serial.print((rf12_crc & 0x00FF), HEX);
            Serial.println((rf12_crc >> 8), HEX);
#endif
            rf12_sendWait(1);  // Wait for transmission complete
            cmd = 0;
            activityLed(0);
        } else { // rf12_canSend
            uint16_t s = rf12_status();            
            showString(PSTR("TX "));					// Not ready to send
#if RF69_COMPAT && !TINY
            Serial.print(rfapi.sendRSSI);
            printOneChar(' ');
#endif            
            showString(PSTR("Busy 0x"));				// Not ready to send            
            Serial.print(s, HEX);
            busyCount++;
            if ((++sendRetry & 3) == 0) {
                showString(PSTR(" Command"));
                showString(ABORTED);					// Drop the command
                cmd = 0;								// Request dropped
            } else {
                delay(sendRetry << 2);					// or try a little later
            }
            Serial.println();   
        }
    } // cmd
} // loop
