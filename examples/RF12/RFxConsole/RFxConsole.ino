/// @dir RFxConsole
///////////////////////////////////////////////////////////////////////////////
#define RF69_COMPAT     1	 // define this to use the RF69 driver i.s.o. RF12 
///                          // The above flag must be set similarly in RF12.cpp
///                          // and RF69_avr.h
#define SX1276			1
#define BLOCK  			0	// Alternate LED pin?
#define INVERT_LED      1	// 0 is Jeenode usual and 1 inverse

#define hubID			31
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
//  1v displays packet reception details, CRC, fei, lna etc
//  2v displays a basic packet decode
//  4v displays receiver restarts
//	8v displays RX statistics once per minute 2018-06-24
//  add any values above to combine display settings.
// Changed RX interrupt trigger to be RSSI rather than SyncMatch 2016-06-20
// Mask two stray interrupts, sleep before reconfiguring radio 2016-06-23
// Added dynamic control of RX Threshold based on restart rate 2016-07-12
// Variable time constant for rate calculation stored in eeprom
// Added counter for ACK's aborted due to busy airwaves 2016-09-01
// Added inter packet time gaps, also min/max 2016-12-12
// Tweaks around <cr> handling to better fit use of folie as a terminal emulator 2017-01-5
// Add 1Hz Timer1 for rate calculation and longer elapsed duration 2017-02-28
// Remove Noise Floor check before entering RX mode 2017-04-18
// Added the 'N' command, setting an interval in seconds for a Noise Floor check 2017-04-19
// Added the 'U' command, 0 suppresses most help/config output. This hopefully stops
//	erroneous configuration changes when UI connected to a program for text capture
//	instead of a human. 1M restores output without changing eeprom 2017-06-07
// Added support for displaying the CRC, received & transmitted 2018-02-13
// Added support for a semaphore queue to store and forward postings to nodes in ACK's 2018-02-27
// Sum the RSSI & FEI of packets that trigger a restart without a sync, reported to serial using 8v
// Support fine radio frequency control using microOffset 32,1600o 2018-06-30
// Add rfapi.configFlags to control afc off/on using "128,8b" 2018-07-4
// Watchdog timer enabled 2018-10-17
// Added an elapsed timer before "OK", use "1U" to activate
// Allow semaphores to be updated using node,group,oldvalue,newvalue 2019-02-04
// Removed the 'm' command, replaced by an extension of the semaphore approach 2019-02-07
// Extended 'U' command to allow locking of configuration 2020-04-04
// Use 3 bits of eeprom per node to store a multiplier of 15ms additional delay to ACK 2020-04-16
//  use 17,212,7n to set the upper bits in eeprom node number to the multiplier 7
// Extend basic ACK to also return the RSSI value of the packet being ACKed 2020-04-19

#if defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny44__)
	#define TINY 1
#endif

#if TINY
  #define OOK          0   // Define this to include OOK code f, k - Adds ?? bytes to Tiny image
  #define JNuMOSFET    1   // Define to power up RFM12B on JNu2/3 - Adds 4 bytes to Tiny image
#else
  #define configSTRING 1   // Define to include "A i1 g210 @ 868 MHz q1" - Adds ?? bytes to Tiny image
  #define HELP         0   // Define to include the help text
  #define MESSAGING    1   // Define to include message posting code m, p - Will not fit into any Tiny image
  #define STATISTICS   1   // Define to include stats gathering - Adds ?? bytes to Tiny image
  #define NODE31ALLOC  0   // Define to include offering of spare node numbers if node 31 requests ack
  #define DEBUG        0   //
#endif
/*
#define REG_BITRATEMSB 0x03  // RFM69 only, 0x02, // BitRateMsb, data rate = 49,261 khz
#define REG_BITRATELSB 0x04  // RFM69 only, 0x8A, // BitRateLsb divider = 32 MHz / 650 == 49,230 khz
#define REG_BITFDEVMSB 0x05  // RFM69 only, 0x02, // FdevMsb = 45 KHz
#define REG_BITFDEVLSB 0x06  // RFM69 only, 0xE1, // FdevLsb = 45 KHz
#define REG_RSSIVALUE  0x24
#define REG_SYNCCONFIG 0x2E  // RFM69 only, register containing sync length
#define REG_SYNCGROUP  0x32  // RFM69 only, register containing group number
#define REG_SYNCVALUE7 0x35  // RFM69 only
#define REG_SYNCVALUE8 0x36  // RFM69 only
*/
#include <JeeLib.h>
#include <util/crc16.h>
//#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <util/parity.h>
#include <avr/wdt.h>
#include "release.h"    // Version tracking by dzach
//#include <Statistic.h>  // without trailing s

#define MAJOR_VERSION RF12_EEPROM_VERSION // bump when EEPROM layout changes
#define MINOR_VERSION 0                   // bump on other non-trivial changes
//#define VERSION "\n[RFxConsole.4]\n"      // keep in sync with the above
#if !configSTRING
  #define rf12_configDump()                 // Omit A i1 g210 @ 868 MHz q1
#endif
#if TINY
  #define SERIAL_BAUD    38400   // can only be 9600 or 38400
  #define DATAFLASH      0       // do not change
  #undef  LED_PIN                // do not change
#else
  #define TINY        0
  #define SERIAL_BAUD 115200  // adjust as needed
  #define DATAFLASH   0       // set to 0 for non-JeeLinks, else 4/8/16 (Mbit)
#endif

/// Save a few bytes of flash by declaring const if used more than once.
const char INITFAIL[] PROGMEM = "\nInit failed\n";
const char RFM12x[] PROGMEM = "RFM12x ";
const char RFM69x[] PROGMEM = "RFM69x ";
const char SX1276x[] PROGMEM = "SX1276 ";
const char BLOC[] PROGMEM = "BLOCK ";
const char UNSUPPORTED[] PROGMEM = "RX Unsupported ";
const char DONE[] PROGMEM = "Done\n";
const char ABORTED[] PROGMEM = " Aborted ";
const char UNKNOWN[] PROGMEM = " Unknown";
const char TX[] PROGMEM = "TX ";
const char SEMAPHOREFULL[] PROGMEM = "Semaphore table full";
#define SALUSFREQUENCY 1660       // Default value
byte salusMode = false;
unsigned int SalusFrequency = SALUSFREQUENCY;

unsigned int NodeMap;
unsigned int newNodeMap;
unsigned long lastRSSIrestart;
unsigned long lastThresholdRSSIrestart;
unsigned long rxCrcLast;
unsigned long minCrcGap = ~0; 
unsigned long maxCrcGap = 0; 
byte minHdr, OldHdr, OldBadHdr, minOldHdr, minOldBadHdr;
byte stickyGroup = 212;
byte eepromWrite;
byte qMin = ~0;
byte qMax = 0;
byte lastrssiThreshold;

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
  #define MAX_NODES 100			// Constrained by eeprom

#elif defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
  #define LED_PIN     13		// activity LED, comment out to disable on/off operation is reversed to a normal Jeenode
  #define LED_ON       1
  #define LED_OFF      0
  #define MAX_NODES 100			// Constrained by eeprom

#else
  #if BLOCK
    #define LED_PIN     8		// activity LED, comment out to disable
  #else
//    #define LED_PIN     13		// activity LED, comment out to disable
  #endif
  #define MAX_NODES 10			// Contrained by RAM (22 bytes RAM per node)
#endif

byte ledStatus = 0;
static void activityLed (byte on) {
#ifdef LED_PIN
    pinMode(LED_PIN, OUTPUT);
#endif
#if INVERT_LED
    ledStatus = on;				// Reported by the software
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

/*00*/byte nodeId;				// used by rf12_config, offset 0
/*01*/byte group;				// used by rf12_config, offset 1
/*02*/byte format;				// used by rf12_config, offset 2
/*03*/byte output       :2;		// 0 = dec, 1 = hex, 2 = dec+ascii, 3 = hex+ascii
/*03*/byte collect_mode :1;		// 0 = ack, 1 = don't send acks
/*03*/byte quiet_mode   :1;		// 0 = show all, 1 = show only valid packets
/*03*/byte helpMenu	  :1;		// 0 = Suppress help menu
/*03*/byte spare_flags  :2;		// offset 3
/*03*/byte defaulted    :1;		// 0 = config set via UI, offset 3
/*04*/word frequency_offset;	// used by rf12_config, offset 4 & 5
/*06*/byte RegPaLvl;			// See datasheet RFM69x Register 0x11, offset 6
/*07*/byte RegRssiThresh;		// See datasheet RFM69x Register 0x29, offset 7
/*08*/signed int matchingRF :8;	// Frequency matching for this hardware, offset 8
/*09*/byte ackDelay         :4;	// Delay in ms added on turnaround RX to TX, RFM69 offset 9
/*09*/byte verbosity        :4;	// Controls output format offset 9
/*10*/byte clearAir;			// Transmit permit threshold, offset 10
/*11*/byte rateInterval;		// Seconds between rate updates, offset 11
/*12*/byte chkNoise;			// Seconds between noise floor checks, offset 12
/*13*/byte spare_bits	:2;		// Available
/*13*/byte microOffset	:6;		// Low order 6 bits of radio frequency
/*14  byte pad[RF12_EEPROM_SIZE - 15];//Fully used! */
/*14*/word crc;					// Integrity CRC

} RF12Config;
static RF12Config config;

static char cmd;
static bool nullValue = true;
static unsigned int value;
static word messageCount = 0;
static byte stack[RF12_MAXDATA+4], top, sendLen, dest;
static byte testCounter;

typedef struct {
    signed int afc;
    signed int fei;
    byte lna;
    byte rssi2;
    unsigned int offset_TX;
    byte PaLvl_TX;
    byte TestLna_TX;
    byte TestPa1_TX;
    byte TestPa2_TX;
} observed;
static observed observedRX;

byte ones = 0;
byte other = 0;
byte watchNode = 0;
byte lastTest;
byte busyCount;
byte missedTests;
byte sendRetry = 0;
static byte highestAck[MAX_NODES];
unsigned int packetAborts;
unsigned int testTX;
unsigned int testRX;

ISR(WDT_vect) { Sleepy::watchdogEvent(); }

volatile unsigned long elapsedSeconds;
static unsigned long currentRestarts;
volatile unsigned long previousRestarts;
volatile unsigned long chkNoise;
volatile unsigned int restartRate;
volatile unsigned int maxRestartRate;
volatile byte ping = false;
volatile byte minuteTick = false;
volatile byte statsInterval = 60;

ISR(TIMER1_COMPA_vect){
	elapsedSeconds++;

#if RF69_COMPAT
    // Update restart rate
    if ((elapsedSeconds % (uint32_t)statsInterval) == 0UL) {
    	minuteTick = true;
    	restartRate = (currentRestarts - previousRestarts);
    	previousRestarts = currentRestarts;
              	
    	if (restartRate > maxRestartRate) { 
    		maxRestartRate = restartRate;
    	}
    }
/*
    if (config.chkNoise) {
    	if (elapsedSeconds > chkNoise) {
    		ping = true;
    		chkNoise = elapsedSeconds + (unsigned long)config.chkNoise;
    	}
    }
*/
#endif
}

unsigned int loopCount, idleTime = 0, offTime = 0;
#if MESSAGING
#define ackEntry 8
#define ackQueue 16
static byte semaphoreStack[ (ackQueue * ackEntry) + 1];	// FIFO per node group /* integer aligned */
#endif
//static unsigned long goodCRC;
#if RF69_COMPAT && STATISTICS
static int32_t CumNodeFEI[MAX_NODES];
static uint32_t CumNodeTfr[MAX_NODES];
static uint32_t rxTimeStamp[MAX_NODES];
static uint16_t CumNodeRtp[MAX_NODES];
static signed int minFEI[MAX_NODES];
static signed int lastFEI[MAX_NODES];
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
static byte nextKey;
#if STATISTICS
static unsigned int CRCbadCount = 0;
static unsigned int pktCount[MAX_NODES];
static unsigned int nonBroadcastCount = 0;
static unsigned int postingsIn, postingsClr, postingsOut, postingsLost;
#endif

static void showNibble (byte nibble) {
    char c = '0' + (nibble & 0x0F);
    if (c > '9')
        c += 7;
    Serial.print(c);
}

static void showByte (byte value, byte format = 0) {
    if ( (config.output & 0x1) || (format) ) {
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
    lastrssiThreshold = rfapi.rssiThreshold = config.RegRssiThresh;
    rfapi.rateInterval = (uint32_t)config.rateInterval << 10;    
    chkNoise = elapsedSeconds + (unsigned long)config.chkNoise;
    config.defaulted = false;   // Value if UI saves config
}

static void saveConfig (byte force = false) {
	if ( (!(config.helpMenu)) && (!(force)) ) {
		showString(BLOC);
	} else {
	    activityLed(1);
    	config.format = MAJOR_VERSION;
    	config.crc = calcCrc(&config, sizeof config - 2);
    	// eeprom_write_block(&config, RF12_EEPROM_ADDR, sizeof config);
    	// this uses 170 bytes less flash than eeprom_write_block(), no idea why

    	for (byte i = 0; i < sizeof config; ++i) {
        	byte* p = &config.nodeId;
        	if (eeprom_read_byte(RF12_EEPROM_ADDR + i) != p[i]) {
				wdt_reset();		// Hold off Watchdog: Eeprom writing is slow...
            	eeprom_write_byte(RF12_EEPROM_ADDR + i, p[i]);
            	delay(4);
            	eepromWrite++;
			}
		}
 
		loadConfig();
	
    	salusMode = false;
#if STATISTICS    
    	messageCount = nonBroadcastCount = CRCbadCount = 0; // Clear stats counters
#endif
    	rf12_sleep(RF12_SLEEP);                             // Sleep while we tweak things
    	if (!rf12_configSilent()) showString(INITFAIL);
    	activityLed(0); 
//    showString(DONE);
	}
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
" <i>,<g>,<s>p post semaphore <s> for group <g> node <i>, to be\n"
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
	if (config.helpMenu) {
#if TINY
    	showString(PSTR("?\n"));
#elif HELP
    	showString(helpText1);
    	if (df_present()) showString(helpText2);
#endif
#if !TINY && configSTRING
    	Serial.print((__DATE__));
    	Serial.print(" ");
    	Serial.print((__TIME__));
    	Serial.print(" ");
    	Serial.println(VERSION_SIG);

    	if (config.helpMenu) showStatus();
    }
#endif
}

static void showStatus() {
#if SX1276
    showString(SX1276x);
#elif RF69_COMPAT
    showString(RFM69x);
#else
	showString(RFM12x);
#endif
	unsigned long s = elapsedSeconds;
    showString(PSTR("Elapsed "));
	elapsed(s);
	
	printOneChar('=');
    Serial.print(s);
	printOneChar('s');
    showString(PSTR(", Led is ")); if (ledStatus) showString(PSTR("on")); else showString(PSTR("off"));
    showString(PSTR(", Free Ram(B) "));
    Serial.print(freeRam());     
#if RF69_COMPAT
	showString(PSTR(", Temperature "));
	Serial.print(RF69::readTemperature(0));
	showString(PSTR("ºC"));
    showString(PSTR(", Restarts "));
    Serial.print(currentRestarts);
    showString(PSTR(", Rate "));
    Serial.print(restartRate);
    printOneChar('^');
    Serial.print(maxRestartRate);
    showString(PSTR("/min,\nSync Match "));
    Serial.print(rfapi.syncMatch);

    showString(PSTR(", Good CRC "));
    Serial.print(rfapi.goodCRC);
    if (rfapi.discards) {
		showString(PSTR(", Discards "));
   		Serial.print(rfapi.discards);
    }
    if (rfapi.rtpMin) {
	    showString(PSTR(", Bounds "));
    	Serial.print(rf12_rtp);
    	printOneChar(';');
    	Serial.print(rfapi.rtpMin);
    	printOneChar('^');
    	Serial.print(rfapi.rtpMax);
    }
//    printOneChar('\n');
    showString(PSTR(", RSSI Rx "));
    Serial.print(rf12_rssi);
    printOneChar(';');
    Serial.print(rfapi.noiseFloorMin);
    printOneChar('^');
    Serial.print(rfapi.noiseFloorMax);
    showString(PSTR(", Tx "));
    Serial.print(rfapi.sendRSSI);
    printOneChar(';');
    Serial.print(minTxRSSI);
    printOneChar('^');    
    Serial.print(maxTxRSSI);
    showString(PSTR(",\nAck Aborts "));
    Serial.print(packetAborts);
    showString(PSTR(", Busy Count "));
    Serial.print(busyCount);
    showString(PSTR(", InterSync(ms) "));
    uint32_t ms = millis();
    Serial.print(ms - rfapi.rxLast);
    printOneChar(';');
    Serial.print(rfapi.minGap);
    printOneChar('^');
    Serial.print(rfapi.maxGap);
    showString(PSTR(", InterCRC(ms) "));
	Serial.print(minOldBadHdr);
    printOneChar('&');
	Serial.print(minOldHdr);
    printOneChar('&');
	Serial.print(minHdr);
    printOneChar(';');
    Serial.print(ms - rxCrcLast);
    printOneChar(';');
    Serial.print(minCrcGap);
    printOneChar('^');
    Serial.print(maxCrcGap);
#endif
//    printOneChar('\n');
    showString(PSTR("\nEeprom U"));
    Serial.print(config.helpMenu);
    rf12_configDump();
#if RF69_COMPAT
    if (!RF69::present) {
        showString(PSTR("RFM69x Problem "));        
        Serial.print((RF69::radioIndex(0,0)), HEX);
        Serial.println((RF69::radioIndex(1,0)), HEX);
        unsigned int mask = 0xAA;
        for (unsigned int i = 0; i < 8; i++) {
            RF69::radioIndex(0 | 0x80, mask);
            Serial.print(mask, BIN);
            printOneChar('?');
            Serial.println((RF69::radioIndex(0, 0)), BIN);
            mask = mask >> 1;
        }
    }

    byte* b = RF69::SPI_pins();  // {OPTIMIZE_SPI, PINCHG_IRQ, RF69_COMPAT, RFM_IRQ, SPI_SS, SPI_MOSI, SPI_MISO, SPI_SCK 
  #if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__) // ATMega1280/2560 with SX1276    
static byte n[] = {1,0,1,4,2,3,1,3,1};     // ATMega1280/2560 with SX1276 settings
  #elif defined(__AVR_ATmega1284P__) 	   // Moteino MEGA    
static byte n[] = {1,0,1,4,5,6,7,2,2};     // ATMega1284 with RFM69 settings
  #else
static byte n[] = {1,0,1,2,3,4,5,3,1};     // Default ATMega328 with RFM69 settings
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

if (mismatch) showString(PSTR("Mismatch\n"));
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
printOneChar(',');
Serial.println(rfapi.rssiZero);
//Serial.print("Micros="); Serial.println((uint32_t)micros());        
#endif    
Serial.flush();
}

// Null handling could be made to store null true/false for each stack entry
bool cr = false;
bool outputTime;
bool hash = false;
static void handleInput (char c) {
	if (hash) {	// Bash style comments    
    	if (c < 32 || top > sizeof stack - 2) {
			stack[top++] = 0;
			printOneChar('#');
			for (byte i = 0; i < top; i++) printOneChar((char)stack[i]);
			Serial.println();
    		hash = false;
    		value = top = 0;
        	nullValue = true;
		} else {
			stack[top++] = c;
		}
    	return;    			
	}    
    if ((c == '.') || (c == 10)) {	// Full stop or <LF>
    	value = top = 0;
        nullValue = true;
    	return;
	}
    	
    //      Variable value is now 16 bits to permit offset command, stack only stores 8 bits
    //      not a problem for offset command but beware.
    if ('0' <= c && c <= '9') {
        nullValue = false;
        if (config.output & 0x1) value = 16 * value + c - '0';
        else value = 10 * value + c - '0';
        return;
    }

    if (('A' <= c && c <= 'F') && (config.output & 0x1)) {
        nullValue = false;
        value = 16 * value + (c - 'A' + 0xA);
        return;
    }

    if (c == ',' || c == ' ') {   // Permit comma or space as delimiters
        if (top < sizeof stack) {
        	if (value < 256)
            	stack[top++] = value; // truncated to 8 bits
            else {
            	Serial.print(value); showString(PSTR(" is GT 255!\n"));
            }
        }
        value = 0;
        nullValue = true;
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
//        	printOneChar(',');
			Serial.println();
        }
        
        if (config.helpMenu) showStatus();

        value = top = 0;	// Clear up
        nullValue = true;
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
        // the last value before '>' as the offset is the only place a 16 bit value will be available.
        rf12_initialize(stack[2], bandToFreq(stack[0]), stack[1],
                config.frequency_offset);
        rf12_sendNow(stack[3], stack + 4, top - 4);
        rf12_sendWait(2);
        rf12_configSilent();

    } else if (c > ' ') {

        // TODO Do we need the "else if" above    

        switch (c) {

            case 'i': // set node id
                if ((value > 0) && (value <= hubID)) {
                    config.nodeId = (config.nodeId & 0xE0) + (value & 0x1F);
                    saveConfig();
                }
                break;

            case 'b': // set band: 4 = 433, 8 = 868, 9 = 915
#if RF69_COMPAT
				if (top == 1) {
					rfapi.ConfigFlags = rfapi.ConfigFlags = stack[0];
				} else {
					stack[0] = rfapi.ConfigFlags;
					top = 1;
				}
				
#endif
                stack[1] = bandToFreq(value);
                if (stack[1]) {
                    config.nodeId = (stack[1] << 6) + (config.nodeId & 0x3F);
                    config.frequency_offset = 1600;
                    saveConfig();
                }
                break;

            case 'g': // set network group
                config.group = value;
                saveConfig();
                //            stickyGroup = value;
                break;

            case 'o':{	 // Offset frequency within band
                         // Stay within your country's ISM spectrum management guidelines, i.e.
                         // allowable frequencies and their use when selecting operating frequencies.
#if RF69_COMPAT
						if (top == 1) {
                         	config.microOffset = (stack[0] & 63);
                            saveConfig();
						} else {
                         	stack[0] = config.microOffset;
							top = 1;
						}
#endif
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
#if !TINY
                         // this code adds about 400 bytes to flash memory use
                         // display the exact frequency associated with this setting
                         byte freq = 0, band = config.nodeId >> 6;
                         switch (band) {
                             case RF12_433MHZ: freq = 43; break;
                             case RF12_868MHZ: freq = 86; break;
                             case RF12_915MHZ: freq = 90; break;
                         }
                         uint32_t f1 = (freq * 100000L + band * 25L * config.frequency_offset) + config.microOffset;
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
                     config.RegPaLvl = value;
                     // Transmit permit threshold
                     if (top == 1 && (stack[0])) config.clearAir = stack[0];
                     saveConfig();
                     break;

            case 'R': // Set hardware specific RX threshold in eeprom
//                	Serial.println(RSSITHRESHOLD);
					if (value) {
#if RF69_COMPAT
/*
						RF69::radioIndex(0x80 | RSSITHRESHOLD, value);
						Serial.print("Readback:");
                     	Serial.println(RF69::control(0x10, 160), HEX);
*/
#endif
                     	//        	Serial.println(config.RegRssiThresh);
                     	//        	Serial.println(rfapi.rssiThreshold);
                     	config.RegRssiThresh = rfapi.rssiThreshold = value;
                     } else config.RegRssiThresh = rfapi.rssiThreshold = 210;
                     
                     if (top == 1) {
                         config.rateInterval = stack[0];
                         rfapi.rateInterval = (uint32_t)(config.rateInterval) << 10;
                     }
                     saveConfig();
                     currentRestarts = previousRestarts = maxRestartRate = 0;
                     previousRestarts = currentRestarts;
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
                     RF69::radioIndex(BITRATEMSB | 0x80, 0x34);                         // 2.4kbps
                     RF69::radioIndex(BITRATELSB | 0x80, 0x15);
                     RF69::radioIndex(BITFDEVMSB | 0x80, 0x04);                         // 75kHz freq shift
                     RF69::radioIndex(BITFDEVLSB | 0x80, 0xCE);
                     rfapi.RssiToSyncLimit = SALUSPACKET16;
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
 //                    if (top == 1) RF69::radioIndex(SYNCCONFIG | 0x80, oneByteSync); // Allow noise
                     // Appropriate sync length will be reset by the driver after the next transmission.
                     // The 's' command is an good choice to reset the sync length. 
                     // Packets will not be recognised until until sync length is reset.
#endif
                     break;

            case 'x': // set reporting mode to decimal (0), hex (1), hex+ascii (2)
                     config.output = value;
                     saveConfig();
                     break;

            case 'v': // display the interpreter version
            		 if (top == 1) {
            			if (stack[0] == 0) statsInterval = 60;
            			else statsInterval = stack[0];
            		 } else statsInterval = 60;
                     config.verbosity = value;        
                     displayVersion();
                     saveConfig();
#if configSTRING
//                     Serial.println();
#endif
                     break;

            case 'p':
                     // Post a semaphore for a remote node, to be collected along with
                     // the next ACK. Format is 20,212,127p where 20 is the node and 212 
                     // is the group number 127 is the desired value to be posted. 
                     // The byte stack[1] contains the target group and stack[0] contains the 
                     // node number. The message string to be posted is in value
#if MESSAGING
					if (nullValue && top == 2) {
						while ( (semaphoreDrop (stack[0], stack[1] )));	// Drop all node, group semaphores
						showPost();	
						break;
					} else 
					if (nullValue) stack[5] = 1<<5; else stack[5] = 3<<5;
					if (top == 1) stack[1] = stickyGroup;
					else if (top == 2) {
						top = 1;
					} else if (top == 3) {
						stack[3] = stack[2]; stack[2] = stack[1]; stack[1] = stickyGroup;
						top = 4;
					}
					if (top) {
						if (!(getIndex(stack[1], stack[0]))) {// Validate Group & Node 
							showByte(stack[0]);
							printOneChar(',');
							showByte(stack[1]);
							showString(UNKNOWN);
							break;
						}
					}

					if (top == 5) {			// Node    Group     Old Key   New Key   flag      Post
						if (semaphoreUpdate((stack[0] | stack[5]), stack[1], stack[2], stack[3], stack[4], value)) {
							showPost();
					 		c = 0;	// loose command printout
							break;
						} else showString(UNKNOWN);
					}
					if (top == 4) {		// Node       Length     Group     Key       Flag      Post
						if (semaphoreSave((stack[0] | stack[5]), stack[1], stack[2], stack[3], value)) {							
							showPost();
					 		c = 0;	// loose command printout
							break;
					 	} else {
                    		showString(SEMAPHOREFULL);
                    		++postingsLost;
				 		}
					}
					if (top  == 1) {
						if (nullValue) {
							showPost();
							if (!(semaphoreDrop(stack[0], stack[1]))) {
                         		showString(UNKNOWN);
								break;
							}
							showPost();							
					 		c = 0;	// loose command printout
							break;
						}
						stack[2] = (uint8_t) value;
						// nextKey is used to try and prevent keys being duplicated
						// such that an Ack may released before actually being posted.
						if (semaphoreSave((stack[0] | 1<<5), stack[1], nextKey, stack[2], 0)) {
							nextKey++;
							nextKey = nextKey%16;						
							postingsIn++;
							showPost();
				 			c = 0;	// loose command printout
					 	} else {
                    		showString(SEMAPHOREFULL);
                    		++postingsLost;
				 		}
                     } else nodeShow(value);
#endif
                     break;
            
            case 'U':
//					if (value == 2) outputTime = true;
//					else
            		if (value == 123) {
//            			outputTime = false;
            			config.helpMenu = 0;	// Lock out eeprom write
            			showStatus();
            		}
            		else
            		if (value == 1953) {
            			config.helpMenu = (stack[0] & 1);
            			saveConfig(true);	// Force eeprom write	
            		}
//            		else config.helpMenu = value & 1;
            		break;	

            case 'm':
            		for (uint8_t i = 0; i < MAX_NODES; i++) {
            			Serial.print(highestAck[i]);
        				printOneChar(' ');
            		}
            		Serial.println();
            		
            		for (byte i = 0; i < (ackQueue * ackEntry); i++) {
            			Serial.print(semaphoreStack[i]);
        				printOneChar(' ');
            		}
            		Serial.println();
            		
 					showPost();
            		break;	

            case 'n':
#if DEBUG
                     dumpAPI();
                     dumpRegs();
#endif
                     if ((top == 0) && (config.group == 0)) {
                         showByte(stickyGroup);
                         stickyGroup = (int)value;
                         showString(UNKNOWN);
                         showByte(stickyGroup);
                     } else if (top == 1) {
                         for (byte i = 0; i < 4; ++i) {
                             // Display eeprom byte                  
                             byte b = eeprom_read_byte((RF12_EEPROM_NODEMAP) + (value * 4) + i);
                             showByte(b);
                             if (!(config.output & 0x1)) printOneChar(' ');

                             if ((stack[0] >= 128) & (i == 0)) {
                                 // Set the removed flag 0x00
                                 eeprom_write_byte((RF12_EEPROM_NODEMAP) + (value * 4), 0);
                                 delay(4);
                             }
                         }
                     } else if (top == 2) {
						if ( !(getIndex( stack[1], (stack[0] & RF12_HDR_MASK))) ) {
							showByte( (stack[0] & RF12_HDR_MASK) );
							printOneChar(',');
							showByte(stack[1]);
							showString(UNKNOWN);
                     	} else {
//		Serial.print(NodeMap);
//        printOneChar(';');
        stack[0] = eeprom_read_byte((RF12_EEPROM_NODEMAP) + (NodeMap * 4));
//        showByte( (stack[0] >> 5) );
//        printOneChar(';');
//        showByte( (stack[0] & RF12_HDR_MASK) ); // Node
//        printOneChar(';');
//        showByte(eeprom_read_byte((RF12_EEPROM_NODEMAP) + (NodeMap * 4) + 1)); // Group
//		Serial.println();

		if ( !(nullValue)) {
			eeprom_write_byte( RF12_EEPROM_NODEMAP + (NodeMap * 4) + 0, ((stack[0] & RF12_HDR_MASK) | (byte)value << 5) ); 
     		delay(4);
     	}
		                    	
                     	}                     	
                     }

                     // Show and set RFMxx registers
                     if ((top == 2) & ((stack[0] == 128) || (stack[0] == 0))) {
                         showByte(stack[1], HEX);
#if RF69_COMPAT
                         printOneChar(':');
                         /* Example usage: 1x        // Switch into hex input mode (optional, adjust values below accordingly)
                         //                80,29,E4n  // 0x80 (write bit) + 0x29 (RSSI Threshold) == 0xA9; E4 (default RSSI threshold); n = node command
                         //                80,11,80n  // 0x80 (write bit) + 0x11 (Output power) == 0x91; 80 (PA0 transmit power minimum); n = node command
                         //                x         // Save certain registers in eeprom and revert to decimal mode
                          */
                         showByte(RF69::control((stack[1] | stack[0]), value), 0xF); // Prints out Register value before any change requested.
                         if (stack[0] == 128) {
                             printOneChar('>');
                             if (!(nullValue)) showByte(value, HEX);
                         }
#else

                         Serial.print((word)(rf12_control(value)));
#endif
                     	Serial.println();
                     }

                     break;

            case 'N': // Set Noise check interval in seconds
					config.chkNoise = value;
					saveConfig();
					break;

			// the following commands all get optimised away when TINY is set 

            case 'l': // turn activity LED on or off
                     activityLed(value);
                     break;

            case 'd': // dump all log markers
            	/*
            		 for (byte i = 0; i < 67; i++) {
            			Serial.print(RF69::radioIndex(0, 0), HEX);
            		 	if(i == 31 || i == 63 || i == 95 || i == 127) Serial.println();
            		 	else Serial.print(".");
            		 }
            		 Serial.println();
            	*/
            		 dumpRegs();

            		 showString(PSTR("InterruptCounts="));
            		 Serial.print(rfapi.interruptCountTX);
                     printOneChar(',');
					 Serial.println(rfapi.interruptCountRX);
            		 
            		 showString(PSTR("Debug="));
            		 Serial.println(rfapi.debug);
            		 rfapi.debug = 0;
            		 
            		 showString(PSTR("intRXFIFO="));
            		 Serial.println(rfapi.intRXFIFO);
            		 
            		 showString(PSTR("LastLen="));
            		 Serial.println(rfapi.lastLen);
                     if (df_present())
                         df_dump();

					 showString(PSTR("PCMSK0:"));
					 Serial.println(PCMSK0, BIN);
					 showString(PSTR("EIMSK:"));
					 Serial.println(EIMSK, BIN);
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

            case 'z':
					if (value < MAX_NODES) {
						oneShow(value);
#if RF69_COMPAT
						pktCount[value] = lastFEI[value] = minFEI[value] = maxFEI[value]
						= lastRSSI[value] = minRSSI[value] = maxRSSI[value] = CumNodeFEI[value] = CumNodeTfr[value]
						= CumNodeRtp[value] = lastLNA[value] = minLNA[value] = maxLNA[value] = 0;
#endif
            		 }
#if RF69_COMPAT
					if (value == 101) {
            		 	rfapi.minGap = minCrcGap = ~0;
            		 	rfapi.maxGap = maxCrcGap = maxRestartRate = 0;
					} else 
#endif
/*            		 if (value == 102) {
            		 	for (int c = 0; c < ackQueue; ++c)  ????
            		 		semaphoreStack[c * 3] = 0;
					} else */
					if (value == 123) {
						clrConfig();
						showString(PSTR(" Zzz...\n"));
                        Serial.flush();
                        rf12_sleep(RF12_SLEEP);
                        cli();
                        Sleepy::powerDown();
					} else 
						if (value == 250) {
                        	clrNodeStore();
                    } else
                    	 if (value == 254) {
                    	 	asm volatile ("  jmp 0");                      
					} else if (value == 255) {
						showString(PSTR("Delay, watchdog enabled\n"));
						delay(10000);
// Done in setup		WDTCSR |= _BV(WDE);
					}
                     break;
            case '*':
            case '#':
					hash = true;
					break;
            default:
                    showHelp();
        } // End case group

    }	// else if (c > ' ')
	 
    if ( ('a' <= c && c <= 'z') || ('R' <= c && c <= 'W') || ('+' <= c && c <= '-') ) {
        showString(PSTR("> "));
        for (byte i = 0; i < top; ++i) {
            showByte(stack[i]);
 //           stack[i] = 0;
            printOneChar(',');
        }
        if (!(nullValue)) Serial.print(value);
        Serial.println(c);
    }

    value = top = 0;
    nullValue = true;
    if (eepromWrite) {
        showString(PSTR("Eeprom written:"));
        Serial.println(eepromWrite);
        eepromWrite = 0;
        if (config.helpMenu) showStatus();
    }    
} // handleInput

static void dumpAPI() {
    byte* p = &rfapi.len;
    for (byte i = 0; i < sizeof rfapi; ++i) {
        Serial.print(p[i]);
        printOneChar(' ');
    }
    Serial.println();
}
/*
static void displayString (const byte* data, byte count) {
    for (byte i = 0; i < count; ++i) {
        char c = (char) data[i];
        showByte(data[i]);
        if (!(config.output & 0x1)) printOneChar(' ');
    }
}
*/
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

// Function Pototype
void wdt_init(void) __attribute__ ((naked, used, section(".init3")));

// Function Implementation
void wdt_init(void)
{
    MCUSR = 0;
    wdt_disable();
    return;
}

#endif

void elapsed (uint32_t s) {
	uint32_t m = s / 604800UL;
	bool p = false;
	if (m) {
		Serial.print(m);
		printOneChar('w');
		p = true;
	}
	m = (s%604800UL) / 86400UL;	
	if (m || p) {
		Serial.print(m);
		printOneChar('d');
		p = true;
	} 
	m = (s%86400UL) / 3600UL;	
	if (m || p) {
		Serial.print(m);
		printOneChar('h');
		p = true;
	}
	m = (s%3600UL) / 60UL;
	if (m || p) {
		Serial.print(m);
		printOneChar('m');
		p = true;
	}
	Serial.print(s%60UL);
	printOneChar('.');
	Serial.print(millis()%1000UL);	
	printOneChar('s');
}

void setup () {
// Disable global interrupts
	cli();
// Setup WatchDog
	wdt_reset();   			// First thing, turn it off
	MCUSR = 0;
	wdt_disable();
	wdt_enable(WDTO_8S);   // enable watchdogtimer
// Enable global interrupts
	sei();

//    delay(380);

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
//	Serial.print("Micros="); Serial.println((uint32_t)micros()); Serial.flush();       
#if LED_PIN == 8
    showString(BLOC);
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
    
#if !TINY
showString(PSTR("ReInit "));showNibble(resetFlags >> 4);
showNibble(resetFlags);
printOneChar(' ');
Serial.println(MCUSR, HEX);
    // TODO the above doesn't do what we need, results vary with Bootloader etc
#endif

#if RF69_COMPAT && STATISTICS
    // Initialise min/max/count arrays
    for (int i = 0; i < MAX_NODES; i++) {
        minFEI[i] = ~0;
        maxFEI[i] = ~0;

    	rfapi.minGap = ~0;	//TODO Find a better place, in the driver to init structure

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
        config.frequency_offset = 1600;
        config.collect_mode = true; // Default to no-ACK
        config.quiet_mode = true;   // Default flags, quiet on
        config.defaulted = true;    // Default config initialized
        config.ackDelay = 0;
        config.helpMenu = true;
#if RF69_COMPAT == 0
        config.group = 212;			// Default group 212
        config.RssiThresh = 2;
        config.clearAir = 160;      // 80dB
#else
        config.group = 0x00;        // Default group 0
        config.RegRssiThresh = 180;	// -90dB
        config.clearAir = 160;      // -80dB
	#if SX1276
        config.RegPaLvl = 223;		// Maximum power TX for SX1276!
	#else        
        config.RegPaLvl = 159;		// Maximum power TX for RFM69CW!
    #endif
#endif
        saveConfig();
        WDTCSR |= _BV(WDE);			// Trigger watchdog restart
        //        if (!(rf12_configSilent()))
        //          showString(INITFAIL);

    }

//    stickyGroup = config.group;
//    if (!(stickyGroup)) stickyGroup = 212;

    df_initialize();

#if !TINY
    //    showHelp();
    if (config.helpMenu) showStatus();
    unsigned int a = ((SPCR | (SPSR << 2)) & 7);    
    if (a != 4) {    // Table 18.5 Relationship Between SCK and the Oscillator Frequency
        showString(PSTR(" SPI="));
        Serial.println(a); 
//        Serial.println(SPCR,HEX);
//        Serial.println(SPSR,HEX);
    }
#endif

    Serial.flush();
    maxRestartRate = 0;
    previousRestarts = rfapi.RSSIrestart;

} // setup

static void clrConfig() {
    // Clear Config eeprom
    showString(PSTR("Clearing Config\n"));
    for (byte i = 0; i < sizeof config; i++) {
        eeprom_write_byte((RF12_EEPROM_ADDR) + i, 0xFF);
    }
    delay(4);
}

static void clrNodeStore() {
    // Clear Node Store eeprom
    showString(PSTR("Clearing NodeStore\n"));
    for (unsigned int n = 0; n < 0x3D0; n++) {
    	Serial.print( eeprom_read_byte(RF12_EEPROM_ADDR + n) );
    	printOneChar(' ');
        eeprom_write_byte((RF12_EEPROM_NODEMAP) + n, 0xFF);
    }
    delay(4);
#if RF69_COMPAT
	for (byte i = 0; i < MAX_NODES; i++) {
		pktCount[i] = lastFEI[i] = minFEI[i] = maxFEI[i]
		= lastRSSI[i] = minRSSI[i] = maxRSSI[i] = CumNodeFEI[i] = CumNodeTfr[i]
		= CumNodeRtp[i] = lastLNA[i] = minLNA[i] = maxLNA[i] = 0;
	}
#endif   
}

/// Display eeprom configuration space
static void dumpEEprom() {
    showString(PSTR("\n\rConfig eeProm:\n"));
    uint16_t crc = ~0;
    for (byte i = 0; i < (RF12_EEPROM_SIZE); ++i) {
        byte d = eeprom_read_byte(RF12_EEPROM_ADDR + i);
        showNibble(d >> 4); showNibble(d);
        crc = _crc16_update(crc, d);
    }
    if (crc) {
        showString(PSTR(" BAD CRC "));
        Serial.println(crc, HEX);
    }
    else showString(PSTR(" GOOD CRC "));
}

/// Display the RFM69x registers
static void dumpRegs() {
	showString(PSTR("\nRadio Registers:\n"));      
	showString(PSTR("    00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\n"));      
    for (byte i = 0; i < 0x80; i+=16) {
    	showNibble(i >> 4); showNibble(i); printOneChar(':');
        for (byte j=0; j<16; j++)
            if (i==0 && j==0) showString(PSTR(" --")); 
            else {
    			printOneChar(' ');
	            byte r = RF69::control((i + j), 0);
    			showNibble(r >> 4); showNibble(r);
    		}
    		Serial.println();
    }
}
//#endif
static void showPost() {
#if MESSAGING
	if (semaphoreStack[0] == 0) {
		showString( PSTR("No post pending") ); 
		Serial.println();
		return;
	} 
    int c = 0;
    while (semaphoreStack[c * ackEntry + 0] != 0) {
        printOneChar('e');										// Envelope
    	Serial.print(c); printOneChar(' ');
        printOneChar('c');
		Serial.print(semaphoreStack[(c * ackEntry) + 6]);		// TX Count
        printOneChar(' ');
        printOneChar('i');
    	Serial.print(semaphoreStack[(c * ackEntry) + 0] & 31);	// Node
        printOneChar(' ');
        printOneChar('g');
	   	Serial.print(semaphoreStack[(c * ackEntry) + 1]);		// Group
        printOneChar(' ');
        printOneChar('k');
    	Serial.print(semaphoreStack[(c * ackEntry) + 2]);		// Key
	    printOneChar(' ');
	    printOneChar('f');
    	Serial.print(semaphoreStack[(c * ackEntry) + 3]);		// Flag
    	byte l = (semaphoreStack[ c * ackEntry + 0 ] >> 5);
	    if (l > 1) {
	        printOneChar(' ');
	        printOneChar('p');
			showWord((semaphoreStack[(c * ackEntry) + 5]) << 8 | semaphoreStack[(c * ackEntry) + 4]);
		}
		Serial.println();										// Integer post
   		++c;   
    }    
return;
}
#endif


/// Display stored nodes and show the next post queued for each node
/// the post queue is not preserved through a restart of RFxConsole
static void nodeShow(byte group) {
    unsigned int index;
    for (index = 0; index < MAX_NODES; index++) {
        byte n = eeprom_read_byte((RF12_EEPROM_NODEMAP) + (index * 4));     // Node number
// 		http://forum.arduino.cc/index.php/topic,140376.msg1054626.html
        if (n == 0xFF) break;                                           // Empty, assume end of table
        if ( (n != 0) ) {                                               // Skip erased entry?
        	oneShow(index);
        }
    }
#if MESSAGING    
    showString(PSTR("Postings "));      
    Serial.print((word) postingsIn);
    printOneChar(',');
    Serial.print((word) postingsClr);
    printOneChar(',');
    Serial.print((word) postingsOut);
    printOneChar(',');
    Serial.println((word) postingsLost);
    
	showPost();
	
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
    Serial.println(RF69::radioIndex(SYNCCONFIG, 0));   
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
    Serial.print(rfapi.interruptCountRX);
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
    Serial.print(RF69::unexpectedMode);			//	0=Sleep, 1=Standby, 2=FS, 3=TX, 4=RX
    printOneChar(',');
    Serial.print(RF69::unexpectedFSM);			// enum TXCRC1, TXCRC2, TXDONE, TXIDLE, TXRECV, RXFIFO
    printOneChar(',');
    Serial.print(RF69::unexpectedIRQFLAGS2);	// Reg 0x28
    printOneChar(',');
    Serial.print(RF69::unexpected);				// Count
    printOneChar(']');
    printOneChar(',');
    Serial.print(rfapi.intRXFIFO);
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

static void oneShow(byte index) {
	if (index > MAX_NODES) return;
    byte n = eeprom_read_byte((RF12_EEPROM_NODEMAP) + (index * 4) + 0);	// Node number
    if (n == 0xFF) return;
    byte g = eeprom_read_byte((RF12_EEPROM_NODEMAP) + (index * 4) + 1);	// Group number
//	printOneChar('#');         
    showByte(index);
    showString(PSTR(" d"));      
    showByte(n >> 5);				// Ack additional delay indicator
    showString(PSTR(" i"));      
    showByte(n & RF12_HDR_MASK);	
    showString(PSTR(" g"));      
    showByte(g);
#if STATISTICS 
	printOneChar(' ');
	elapsed(elapsedSeconds - rxTimeStamp[index]);	
	unsigned int c = pktCount[index];
	if (c) {   
	    showString(PSTR(" rx:"));
	    Serial.print(c);
	}
	if (highestAck[index]) {
		showString(PSTR(" h-ack:"));		
		Serial.print(highestAck[index]);	
	}
#endif

#if MESSAGING 
	byte * v = semaphoreGet(n, g);    
    if (v) {
    	showString(PSTR(" post:")); 
    	showByte((*(v + 3)));	// More informative than the (+ 2) value
    }
#endif

#if RF69_COMPAT && STATISTICS            
    if (c) {
        showString(PSTR(" FEI(Cum:"));
		Serial.print(CumNodeFEI[index]);
//        printOneChar(' ');

		int16_t delta = abs(minFEI[index] - maxFEI[index]);
        showString(PSTR(" Last:"));
     	Serial.print(lastFEI[index]);
        showString(PSTR(" Mid:"));
		Serial.print(minFEI[index] + (delta / 2));     	     	
        showString(PSTR(" Avg:"));
        Serial.print((CumNodeFEI[index]) / (int32_t)pktCount[index]);
        printOneChar(';');
        Serial.print(minFEI[index]);
        printOneChar('-');
        Serial.print(maxFEI[index]);
        printOneChar('=');
//        Serial.print(abs(minFEI[index] - maxFEI[index]));
        Serial.print(delta);
        showString(PSTR(") RSSI("));
        showByte(lastRSSI[index]);
        printOneChar(';');
        showByte(eeprom_read_byte((RF12_EEPROM_NODEMAP) + (index * 4) + 2)); // Show original RSSI value
        printOneChar('/');
        showByte(minRSSI[index]);
        printOneChar('^');
        showByte(maxRSSI[index]);
        showString(PSTR(") LNA("));
        Serial.print(lastLNA[index]);
        printOneChar(';');
        showByte(eeprom_read_byte((RF12_EEPROM_NODEMAP) + (index * 4) + 3)); // Show original LNA value
        printOneChar('/');
        Serial.print(minLNA[index]);
        printOneChar('^');
        Serial.print(maxLNA[index]);
        showString(PSTR(") TFR "));
        Serial.print(((CumNodeTfr[index]) / (uint32_t)pktCount[index]) + 1000);
        printOneChar(' ');
        Serial.print(CumNodeRtp[index]);
  }
#endif
    Serial.println();
}

byte specificNodeDelay = 0;
static byte getIndex (byte group, byte node) {
    newNodeMap = NodeMap = 0xFFFF;
    // Search eeprom RF12_EEPROM_NODEMAP for node/group match
    for (unsigned int index = 0; index < MAX_NODES; index++) {
        byte n = eeprom_read_byte((RF12_EEPROM_NODEMAP) + (index * 4));
        //              http://forum.arduino.cc/index.php/topic,140376.msg1054626.html
        if ( (n == 0) || (n == 0xFF) ) {					// Erased (0xFF) or empty (0x00) entry?
            if (newNodeMap == 0xFFFF) newNodeMap = index;   // Save pointer to a first free entry
            if (n == 0xFF) return(false);                   // Erased, assume end of table!
        } else {
            if ((n & RF12_HDR_MASK) == (node & RF12_HDR_MASK)) {  // Node match?
                byte g = eeprom_read_byte((RF12_EEPROM_NODEMAP) + (index * 4) + 1);
                if (g == group) {                                 // Group match?
                    // found a match;
                    NodeMap = index;
                    specificNodeDelay = (n >> 5);
                    return (true);
                }
            }
        }
    } 
    return(false);
}

static bool semaphoreSave (byte node, byte group, byte key, byte flag, unsigned int value) {
	for (int c = 0; c < ackQueue; ++c) {
		if (semaphoreStack[(c * ackEntry) + 0] == 0) {
			semaphoreStack[(c * ackEntry) + 0] = node;	
			semaphoreStack[(c * ackEntry) + 1] = group;	
			semaphoreStack[(c * ackEntry) + 2] = key;
			semaphoreStack[(c * ackEntry) + 3] = flag;
			semaphoreStack[(c * ackEntry) + 4] = value;
			semaphoreStack[(c * ackEntry) + 5] = value >> 8;
			semaphoreStack[(c * ackEntry) + 6] = 0;	// TX Count
			semaphoreStack[(c * ackEntry) + 7] = 0;	// Spare		
			return true;	
		}
	}
	return false;
}

static bool semaphoreUpdate (byte node, byte group, byte key, byte newKey, byte flag, uint16_t value) {
	for (int c = 0; c < ackQueue; ++c) {
		if ( ( semaphoreStack[ (c * ackEntry) + 0] & 31) == (node & 31)	
		&& semaphoreStack[ (c * ackEntry) + 1] == group				
		&&	semaphoreStack[ (c * ackEntry) + 2] == key) {
				semaphoreStack[(c * ackEntry) + 0] = node;	// Possibly updates ackLen
				semaphoreStack[(c * ackEntry) + 2] = newKey;
				semaphoreStack[(c * ackEntry) + 3] = flag;
				semaphoreStack[(c * ackEntry) + 4] = value;
				semaphoreStack[(c * ackEntry) + 5] = value >> 8;
				semaphoreStack[(c * ackEntry) + 6] = 0;	// Clear TX count
				return true;	
		} else
			if (semaphoreSave(node, group, newKey, flag, value)) return true;
	}
	return false;
}
static bool semaphoreDrop (byte node, byte group) {
	for (int c = 0; c < ackQueue; c++) {
		if ( ( semaphoreStack[ (c * ackEntry) + 0] & 31) == (node & 31)	
		&& semaphoreStack[ (c * ackEntry) + 1] == group) {
			while (c < ackQueue) {
				// Overwrite by shifting down entries above
				semaphoreStack[ (c * ackEntry) + 0] = semaphoreStack[ (c * ackEntry) + ackEntry];
				if (semaphoreStack[ (c * ackEntry) + ackEntry] == 0) {	// Reached highest used stack entry
					break;
				}
				semaphoreStack[ (c * ackEntry) + 1] = semaphoreStack[ (c * ackEntry) + (ackEntry + 1)];
				semaphoreStack[ (c * ackEntry) + 2] = semaphoreStack[ (c * ackEntry) + (ackEntry + 2)];
				semaphoreStack[ (c * ackEntry) + 3] = semaphoreStack[ (c * ackEntry) + (ackEntry + 3)];
				semaphoreStack[ (c * ackEntry) + 4] = semaphoreStack[ (c * ackEntry) + (ackEntry + 4)];
				semaphoreStack[ (c * ackEntry) + 5] = semaphoreStack[ (c * ackEntry) + (ackEntry + 5)];
				semaphoreStack[ (c * ackEntry) + 6] = semaphoreStack[ (c * ackEntry) + (ackEntry + 6)];
				semaphoreStack[ (c * ackEntry) + 7] = semaphoreStack[ (c * ackEntry) + (ackEntry + 7)];
				++c;
			}
		return true;
		}
	}
	return false;
}
static byte * semaphoreGet (byte node, byte group) {
	for (int c = 0; c < ackQueue; ++c) {
		if ( ( semaphoreStack[ (c * ackEntry) + 0] & 31) == (node & 31)	
		&& (semaphoreStack[(c * ackEntry) + 1] == group)) {
			return &(semaphoreStack[c * ackEntry]);
		}
	}
	return 0;	// Not found
}

void loop () {
	wdt_reset();
#if TINY
    if ( _receive_buffer_index ) handleInput( inChar() );
#else
    if ( Serial.available() ) handleInput( Serial.read() );
#endif
	if ( rf12_recvDone() ) {
    	currentRestarts = rfapi.RSSIrestart;

#if RF69_COMPAT && !TINY	// At this point the radio is in standby

        if (rf12_crc == 0) {
			unsigned long rxCrcGap;
        
 			if (RF12_WANTS_ACK && (config.collect_mode) == 0) {
				RF69::control(1, 2);	// radio to mode FS, ACK will be needed
 			} else {          	
				// ACK not required for current packet 				
        		rf12_recvDone();		// Attempt to buffer next RF packet
        		// At this point the receiver is active but previous buffer intact        		     					
 			}
 
//        	rf12_recvDone();		// Attempt to buffer next RF packet
//        	// At this point the receiver is active but previous buffer intact        		     					
 			
         	rxCrcGap = rf12_interpacketTS - rxCrcLast;
 			rxCrcLast = rf12_interpacketTS;
 			if (rxCrcGap < minCrcGap) {
 				minCrcGap = rxCrcGap;
 				minOldHdr = OldHdr;
 				minOldBadHdr = OldBadHdr;
 				minHdr = rf12_hdr;
 			}
 			if (rxCrcGap > maxCrcGap) maxCrcGap = rxCrcGap;
 			
		}
       
        observedRX.afc = rf12_afc;
        observedRX.fei = rf12_fei;
        observedRX.rssi2 = rf12_rssi;
        observedRX.lna = rf12_lna;

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
//            goodCRC++;
            if ((watchNode) && ((rf12_hdr & RF12_HDR_MASK) != watchNode)) return;
            
            if (outputTime) {
            	elapsed(elapsedSeconds);
                printOneChar(' ');
			}            
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
                Serial.print((word) elapsedSeconds, DEC);  
                showString(PSTR("s Salus I Channel "));
                showByte(rf12_buf[1]);
                printOneChar(':');
                showByte(rf12_buf[2]);
                Serial.println();
                //                return;
                n = RF69::byteCount - 3;
            }            
            if(rf12_buf[0] == 212 && rf12_buf[1] >= 160 && (salusMode)) {
                Serial.print((word) elapsedSeconds, DEC);  
                showString(PSTR("s Salus II Device:"));
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
                        showString(UNKNOWN);
                        break;
                }

                Serial.println();
            }            
#endif
#if RF69_COMPAT
            if ((rf12_hdr &  ~RF12_HDR_MASK) == (RF12_HDR_DST | RF12_HDR_ACK) && 
                    ((rf12_hdr &  RF12_HDR_MASK) > 23) && (config.group != 0)) {
                showString(UNSUPPORTED);
                showString(RFM69x);
                // RFM69 radio problem
                // Serial.println();
            }  
#endif
            if (config.quiet_mode) {
            	OldBadHdr = rf12_hdr;	// Save node number in case next packet triggers an inquest.
				return;
			}
			
            crc = false;
            showString(PSTR("RX ? "));
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
            byte m = rf12_data[0] - (lastTest + 1);
            if (m) {
                printOneChar('-');
                showByte(m);
                missedTests =+ m;
            }
            lastTest = rf12_data[0];
        } else {
            for (byte i = 0; i < n; ++i) {
                if (!(config.output & 1)) // Decimal output?
                    printOneChar(' ');
                showByte(rf12_data[i]);
            }
        }

        if (config.verbosity & 1) {
        	printOneChar(' ');
// Print the CRC
        	showNibble(rf12_data[(n + 1)] >> 4);
			showNibble(rf12_data[(n + 1)]);
        	showNibble(rf12_data[(n)] >> 4);
			showNibble(rf12_data[(n)]);
        	printOneChar('h');
        }
        
#if RF69_COMPAT && !TINY
        if ((config.verbosity & 1) || (!crc)) {
			if (observedRX.afc) {        
            	showString(PSTR(" a="));
            	Serial.print(observedRX.afc);		// TODO What units has this number?
			}
            showString(PSTR(" f="));
            Serial.print(observedRX.fei);			// TODO What units has this number?
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
/*
            showString(PSTR(" t="));
            Serial.print((RF69::readTemperature(0)));        

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
*/
            showString(PSTR(" d="));
            Serial.print(rf12_tfr);
            if (rf12_rtp) {
	            printOneChar('~');
				Serial.print(rf12_rtp);
			}
			/*
            showString(PSTR(" r="));
            Serial.print(rf12_rst);
//            showString(PSTR(" i="));
//            Serial.print(rxGap);

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
     //   showString(PSTR(" ("));
        printOneChar(' ');                
        if (config.output & 0x1)                  // Hex output?
            showByte(observedRX.rssi2);
        else {
            Serial.print(observedRX.rssi2 >> 1);
            if (observedRX.rssi2 & 0x01) showString(PSTR(".5"));
            showString(PSTR("dB"));
        }
		bool gotIndex = getIndex(rf12_grp, (rf12_hdr & RF12_HDR_MASK));
		if (gotIndex) {
	        printOneChar(' ');
    	    elapsed(elapsedSeconds - rxTimeStamp[NodeMap]);
//        	printOneChar(' ');
//        	Serial.print(NodeMap);
	        rxTimeStamp[NodeMap] = elapsedSeconds;
        }
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
            if(rf12_len != rf12_advisedLen) {
                printOneChar('(');
                Serial.print(rf12_advisedLen);	// Show actual length received
                printOneChar(')');
            }
#endif
         }
		
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

			if ((rf12_hdr & (RF12_HDR_CTL | RF12_HDR_DST)) == (RF12_HDR_CTL | RF12_HDR_DST)) 
			  rf12_hdr = (hubID | RF12_HDR_CTL | RF12_HDR_ACK);	
				         
            if ( !(rf12_hdr & RF12_HDR_DST) && (rf12_hdr & RF12_HDR_MASK) != hubID ) {
                // This code only sees broadcast packets *from* other nodes.
                // Packets addressed to nodes do not identify the source node!          
                // Search RF12_EEPROM_NODEMAP for node/group match
#if !TINY
                if ( !(gotIndex) && !(testPacket) ) {
                    if (newNodeMap != 0xFFFF) { // Storage space available?
                        showString(PSTR("New Node g"));
                        showByte(rf12_grp);
                        showString(PSTR(" i"));
                        showByte(rf12_hdr & RF12_HDR_MASK);
                        showString(PSTR(" Index "));
                        Serial.println(newNodeMap);
                        eeprom_write_byte( (RF12_EEPROM_NODEMAP) + (newNodeMap * 4), (rf12_hdr & RF12_HDR_MASK) );  // Store Node and
                        eeprom_write_byte(((RF12_EEPROM_NODEMAP) + (newNodeMap * 4) + 1), rf12_grp);              // and Group number
  #if RF69_COMPAT
                        eeprom_write_byte(((RF12_EEPROM_NODEMAP) + (newNodeMap * 4) + 2), observedRX.rssi2);      //  First RSSI value
                        eeprom_write_byte(((RF12_EEPROM_NODEMAP) + (newNodeMap * 4) + 3), observedRX.lna);        //  First LNA value
  #endif
                        delay(4);
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

				lastFEI[NodeMap] = rf12_fei;
				CumNodeFEI[NodeMap] = CumNodeFEI[NodeMap] + rf12_fei;
				CumNodeTfr[NodeMap] = CumNodeTfr[NodeMap] + (rf12_tfr - 1000UL);	// Save capacity
				CumNodeRtp[NodeMap] = CumNodeRtp[NodeMap] + rf12_rtp;
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
                if ((pktCount[NodeMap] % 100) == 0) oneShow(NodeMap);
            } else {
                nonBroadcastCount++;
#endif
            }

            // Where requested, acknowledge broadcast packets - not directed packets
            // unless directed to this nodeId
            if ( ((RF12_WANTS_ACK && (config.collect_mode) == 0) && (!(rf12_hdr & RF12_HDR_DST)) )             
                    || (rf12_hdr & (RF12_HDR_MASK | RF12_HDR_ACK | RF12_HDR_DST)) 
                    == ((config.nodeId & 0x1F) | RF12_HDR_ACK | RF12_HDR_DST)) {

                byte ackLen = 0;
				byte special = false;
#if NODE31ALLOC                    
                // This code is used when an incoming packet requesting an ACK is also from Node 31
                // The purpose is to find a "spare" Node number within the incoming group and offer 
                // it with the returning ACK.
                // If there are no spare Node numbers nothing is offered
                // TODO perhaps we should increment the Group number and find a spare node number there?
                if (((rf12_hdr & RF12_HDR_MASK) == hubID) && (!(rf12_hdr & RF12_HDR_DST)) && (!(testPacket))) {
                	special = true;
                    // Special Node 31 source node
/*
                    // Make sure this nodes node/group is already in the eeprom
                    if (((getIndex(config.group, config.nodeId))) && (newNodeMap != 0xFFFF)) {   
                        // node/group not found but there is space to save
                        eeprom_write_byte((RF12_EEPROM_NODEMAP) + (newNodeMap * 4), (config.nodeId & RF12_HDR_MASK));
                        eeprom_write_byte(((RF12_EEPROM_NODEMAP) + (newNodeMap * 4) + 1), config.group);
                        eeprom_write_byte(((RF12_EEPROM_NODEMAP) + (newNodeMap * 4) + 2), 255);
                    }
                    delay(4);
*/
                    for (byte i = 1; i < hubID; i++) {
                        // Find a spare node number within received group number
                        if (!(getIndex(rf12_grp, i ))) {         // Node/Group pair not found?
                            observedRX.offset_TX = config.frequency_offset;
/*
  #if RF69_COMPAT  			// Below may need rework as a result of double buffering the radio                     
                            observedRX.PaLvl_TX = RF69::radioIndex(RegPaLvl, 0x9F);    // Pull the current RegPaLvl from the radio
                            observedRX.TestLna_TX = RF69::radioIndex(RegTestLna, 0x1B);  // Pull the current RegTestLna from the radio
                            observedRX.TestPa1_TX = RF69::radioIndex(RegTestPa1, 0x55);  // Pull the current RegTestPa1 from the radio
                            observedRX.TestPa2_TX = RF69::radioIndex(RegTestPa2, 0x70);  // Pull the current RegTestPa2 from the radio
  #endif
*/
  // TODO					The above doesn't realise that radio packet buffering may be happening

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
                crlf = true;									// A static delay for all ACK's, more later
/*
				if (rf12_data[0] != 85) {
               		showString(PSTR("Alert (k")); 
    				showByte( (rf12_data[0] ) );
					showString(PSTR(") i")); 
					showByte(rf12_hdr & RF12_HDR_MASK);
					showString(PSTR(" g")); 
             		Serial.println(rf12_grp);
             	}
*/                                       
                if (config.ackDelay) delayMicroseconds( 800 + (config.ackDelay * 50) );	// changing into TX mode is quicker than changing into RX mode for RF69.     

                byte i = getIndex( rf12_grp, (rf12_hdr & RF12_HDR_MASK) );
                
                if (specificNodeDelay) {
            		delay( (specificNodeDelay * 5) );	// Multiplier of 5ms
                }

            	showString(TX);
                byte r = rf12_canSend(config.clearAir);
                if (r) {
#if RF69_COMPAT && !TINY
                    Serial.print(rfapi.sendRSSI);	//delay(10);
                    if (rfapi.sendRSSI < minTxRSSI) minTxRSSI = rfapi.sendRSSI;
                    if (rfapi.sendRSSI > maxTxRSSI) maxTxRSSI = rfapi.sendRSSI;
#endif            
                    showString(PSTR(" -> ack "));
                    if (testPacket) {  // Return test packet number being ACK'ed
                        stack[(sizeof stack - 2)] = 0x80;
                        stack[(sizeof stack - 1)] = rf12_data[0];
                        ackLen = 2;
                    }
                    printOneChar('i');
                    showByte(rf12_hdr & RF12_HDR_MASK);                    

#if RF69_COMPAT && !TINY
                    if (config.group == 0) {
                        printOneChar(' ');
                        showString(PSTR("g"));
                        showByte(rf12_grp);
                        RF69::radioIndex(SYNCGROUP | 0x80, rf12_grp); // Reply to incoming group number
                    }
#endif
 #if MESSAGING
	                // This code is used when an incoming packet is requesting an ACK, it determines if a semaphore is posted for this Node/Group.
    	            // If a semaphore exists it is stored in the buffer. If the semaphore has a message addition associated with it then
        	        // the additional data from the message store is appended to the buffer and the whole buffer transmitted to the 
            	    // originating node with the ACK.
            	    
            	    byte * v;    
                    v = semaphoreGet((rf12_hdr & RF12_HDR_MASK), rf12_grp);
                	if ( (v) && (!(special)) ) {	// Post pending?
                		bool dropNow = false;                			
            	        ackLen = (*(v + 0) >> 5) + 1;	// ACK length in high bits of node
	                    if (rf12_data[0] == (*(v + 2)) && ( *(v + 6)) ) { // Matched and transmitted at least once
	                    // Check if previous Post value is the first byte of this payload 
        	                showString(PSTR(" Released "));
                    		postingsClr++;
                    		dropNow = true;
                    	} else	
                		if (rf12_data[0] == 170) {
                    		dropNow = true;
        	                showString(PSTR(" Rejected ")); 
           	        	} else {
                    		showString(PSTR(" Posted "));
                    		(byte)++(*(v + 6));
                    		postingsOut++;
                    	}	// 09/05/2020 17:37:08 TX 132 -> ack i19 Posted c1 (k1) k212 f203 v300 l4
                    	printOneChar('c');
                    	showByte((*(v + 6)));			// Count
                    	showString(PSTR(" (k"));
            	    	showByte( (rf12_data[0]) );		// Incoming Key
        	            showString(PSTR(") "));

                    	printOneChar('k');
						showByte( (*(v + 2) ) );		// Outgoing Key
						showString(PSTR(" f"));
						Serial.print( (*(v + 3) ) );	// Outgoing Function

						if (ackLen > 2) {
							showString(PSTR(" v"));		// Outgoing Value
							Serial.print( (uint16_t) ( (*(v + 5)) << 8 | (*(v + 4) ) ) );
						}
						showString(PSTR(" l"));
						crlf = true;
                     	Serial.print(ackLen);			// Length
                     	                    	
                    	if (i) {
                    		if ( (*(v + 6) > highestAck[NodeMap]) ) 
                    	  	highestAck[NodeMap] = (*(v + 6));		// Save hi point
                    	}
                    	
                    	v+=2;		// Adjust pointer past semaphore header bytes
                    	
                     	if (dropNow) {
	                    	if ( !(semaphoreDrop((rf12_hdr & RF12_HDR_MASK), rf12_grp) ) )
	                			showString(PSTR(" NOT FOUND "));
	                		// Deliver a standard, one byte Ack
        	      			v = (byte *)&rf12_rssi;	// Use as the TX buffer pointer
		        	      	printOneChar(' ');
		        			showByte(rf12_rssi);       	      		
		                	ackLen = 1;		// Convert to a basic ACK	                		
                    	}
        	        } else {
        	      		v = (byte *)&rf12_rssi;	// Point to RSSI as the TX buffer
        	      		printOneChar(' ');
        	      		showByte(rf12_rssi);       	      		
        	        	ackLen = 1;		// Supply received RSSI value in all basic ACKs
        	        }	// if ( (v) && (!(special)) )

 					if (rf12_data[0] != 85) {
               			showString(PSTR(" Alert (k")); 
    					showByte( (rf12_data[0] ) );
                    	printOneChar(')');
    	        	}

#endif                                        
////////////////////////////////////////////////////////////////////      	                	                
// Temporary Code until i21 is upgraded to understand new Acks//////
//////////////////////////////////////////////////////////////////// 
					if ( (rf12_hdr & RF12_HDR_MASK) == 21) {
						ackLen = 0;
						showString(PSTR("Above Ack set zero length")); 
					}
					
////////////////////////////////////////////////////////////////////      	                	                
////////////////////////////////////////////////////////////////////      	                	                
    	                	                
					rf12_sendStart(RF12_ACK_REPLY, (v), ackLen);
                	rf12_sendWait(0);
    				chkNoise = elapsedSeconds + (unsigned long)config.chkNoise;// Delay check
    				ping = false;		// Cancel any pending Noise Floor checks
                    
            	} else { // if (r)
            		packetAborts++;   
            		Serial.print(rfapi.sendRSSI);
                	showString(ABORTED);		// Airwaves busy, drop ACK and await a retransmission.
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
                	for (byte c = 0; c < 10; c++ ) {
                		printOneChar(' ');
						Serial.print(c);
            			printOneChar('=');
            			Serial.print(RF69::currentRSSI());
            		}
				} //if (r)
			} // if ( ((RF12_WANTS_ACK
			
			if (crlf) Serial.println();
        	activityLed(0);
        	OldHdr = rf12_hdr;	// Save node number in case next packet triggers an inquest.
		} // if (rf12_crc
		
	}// if ( rf12_recvDone()
    
#if RF69_COMPAT && !TINY			// Weird conditional when Tiny84    
    else if (currentRestarts != lastRSSIrestart) {
    		lastRSSIrestart = currentRestarts;

            if (ledStatus) activityLed(0);
            else activityLed(1);

            if (config.verbosity & 4) {
                showString(PSTR("RX restart "));
                Serial.print(currentRestarts);
                printOneChar(' ');
                Serial.print(rfapi.rssiThreshold);
                printOneChar(' ');
                Serial.print(restartRate);
                printOneChar(' ');
                Serial.print(rf12_drx);
                showString(PSTR(" afc="));
                Serial.print(RF69::afc);
                showString(PSTR(" fei="));
                Serial.print(RF69::fei);
                showString(PSTR(" lna="));
                Serial.print((RF69::lna));
                showString(PSTR(" rssi="));
                Serial.print(RF69::rssi);
                printOneChar(' ');
                unsigned long m = millis();
                Serial.print(m - rfapi.interpacketTS);
                printOneChar(' ');
				Serial.println(m);
            }
        }

        if ((config.verbosity & 8) && (minuteTick)) {
            minuteTick = false;            	
            if (rfapi.changed) {
            	rfapi.changed = false;
	            showString(PSTR("RX Stats "));
    	        Serial.print(rfapi.rssiThreshold);
        	    printOneChar(' ');
        		Serial.print(currentRestarts);
	        	printOneChar(' ');
    	        Serial.print(rfapi.syncMatch);
        		printOneChar(' ');
            	Serial.print(rfapi.goodCRC);
	        	printOneChar(' ');
		        Serial.print(restartRate);
        	    printOneChar(' ');
            	Serial.print(maxRestartRate);
 
				printOneChar(' ');
    			Serial.print(rfapi.cumRSSI[1] +  rfapi.cumRSSI[2] + rfapi.cumRSSI[3]
    			 + rfapi.cumRSSI[4] + rfapi.cumRSSI[5] + rfapi.cumRSSI[6] + rfapi.cumRSSI[7]);
    			printOneChar(' ');
   		 
	    		Serial.print(rfapi.cumFEI[1] +  rfapi.cumFEI[2] + rfapi.cumFEI[3]
    			 + rfapi.cumFEI[4] + rfapi.cumFEI[5] + rfapi.cumFEI[6] + rfapi.cumFEI[7]);
   				printOneChar(' ');
    		 
//	    		Serial.print(rfapi.cumAFC[1] +  rfapi.cumAFC[2] + rfapi.cumAFC[3]
//    			 + rfapi.cumAFC[4] + rfapi.cumAFC[5] + rfapi.cumAFC[6] + rfapi.cumAFC[7]);
//   				printOneChar(' ');

    			Serial.print(rfapi.cumZeros[1] +  rfapi.cumZeros[2] + rfapi.cumZeros[3]
    		 	+ rfapi.cumZeros[4] + rfapi.cumZeros[5] + rfapi.cumZeros[6] + rfapi.cumZeros[7]);
   				printOneChar(' ');
    		 
    			Serial.print(rfapi.cumCount[1] +  rfapi.cumCount[2] + rfapi.cumCount[3]
    		 	+ rfapi.cumCount[4] + rfapi.cumCount[5] + rfapi.cumCount[6] + rfapi.cumCount[7]);
            
				for (byte i = 0; i < 8; i++) {
            		if (rfapi.cumCount[i]) {
            			showString(PSTR(" [ "));
						Serial.print(i);
	        			printOneChar(' ');
        				Serial.print(rfapi.cumRSSI[i] / rfapi.cumCount[i]);
        				printOneChar(' ');
        				Serial.print(rfapi.cumFEI[i] / rfapi.cumCount[i]);
//    	    			printOneChar(' ');
//        				Serial.print(rfapi.cumAFC[i] / rfapi.cumCount[i]);
//	  	    			printOneChar(' ');
//	    	        	Serial.print(rfapi.cumLNA[i] / rfapi.cumCount[i]);
    	    			printOneChar(' ');
	    	        	Serial.print(rfapi.cumZeros[i]);
    	    			printOneChar(' ');
	    	        	Serial.print(rfapi.cumCount[i]);
            			showString(PSTR(" ]"));
	        			
		        		rfapi.cumRSSI[i] = rfapi.cumFEI[i] /*= rfapi.cumLNA[i]*/ = 
		        	 	/*rfapi.cumAFC[i] =*/ rfapi.cumZeros[i] = rfapi.cumCount[i] = 0;
		        	 }
	        	}
            Serial.println();	        	
	        }
        }
#endif

#if TINY						// Very weird, needed to make Tiny code compile
//    } // !rf12_recvDone
#endif

	wdt_reset();
	    
    if ((cmd) || (ping)) {
        byte r = rf12_canSend(config.clearAir);
        if (r) {
			sendRetry = 0;
#if RF69_COMPAT        
            if (rfapi.sendRSSI < minTxRSSI) minTxRSSI = rfapi.sendRSSI;
            if (rfapi.sendRSSI > maxTxRSSI) maxTxRSSI = rfapi.sendRSSI;
#endif        
            activityLed(1);
            showString(TX);
			Serial.print(r); //delay(10);
            if (cmd) {
            	showString(PSTR(" -> "));
            	showByte(sendLen);
            	showString(PSTR("b "));
            	byte header = (cmd == 'a' ? RF12_HDR_ACK : 0);
            	if (dest)
                	header |= RF12_HDR_DST | dest;

           		rf12_sendStart(header, stack, sendLen);
            	rf12_sendWait(1);  // Wait for transmission complete

            	if (config.verbosity & 1) {
					// Display CRC transmitted           	
            		showNibble(rf12_crc >> 12);
            		showNibble(rf12_crc >> 8);
            		showNibble(rf12_crc >> 4);
            		showNibble(rf12_crc);
            		printOneChar('h');
            	}
            	Serial.println();
            	cmd = 0;
            	activityLed(0);
    			chkNoise = elapsedSeconds + (unsigned long)config.chkNoise;// Delay check
            	ping = false;	// Clear any pending radio pings            
            } else { // cmd
            	showString(PSTR(" Clear\n"));	// Airwaves are available
            	ping = false;	// Ping completed
            }
        } else { // (r)
        
            uint16_t s = rf12_status();            
            showString(TX);
#if RF69_COMPAT && !TINY
            Serial.print(rfapi.sendRSSI);
            printOneChar(' ');
#endif            
            Serial.print(busyCount);
            printOneChar(',');
            Serial.print(sendRetry);
            showString(PSTR(" Busy 0x"));				// Not ready to send            
            Serial.print(s, HEX);
            busyCount++;
            if ((++sendRetry) > 3) {
            	sendRetry = 0;
                showString(ABORTED);					// Drop the command
                cmd = 0;								// Request dropped
                ping = false;							// Drop Noise level check
            }
            Serial.println();   
        } // (r)
    } // (cmd || ping)
} // loop
