/// @dir RF12demo
/// Configure some values in EEPROM for easy config of the RF12 later on.
// 2009-05-06 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

// this version adds flash memory support, 2009-11-19
// Adding frequency features. 2013-09-05
// Added postbox semaphore feature 2013-10-24
// For the ATTiny84 node number 15 is recommended since
// node numbers > 14 cannot be stored in eeprom
// Node numbers 16-31 can only be used if MAX_NODES and thereby
// the size of the nodes array is adjusted accordingly
//
#define RF69_COMPAT 1 // define this to use the RF69 driver i.s.o. RF12
#include <JeeLib.h>
#include <util/crc16.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <util/parity.h>

#define MAJOR_VERSION RF12_EEPROM_VERSION // bump when EEPROM layout changes
#define MINOR_VERSION 2                   // bump on other non-trivial changes
#define VERSION "\n[RF12demo.12]"         // keep in sync with the above

#if defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny44__)
#define TINY        1
#define SERIAL_BAUD 38400   // can only be 9600 or 38400
#define DATAFLASH   0       // do not change
#undef  LED_PIN             // do not change
#else
#define TINY        0
#define SERIAL_BAUD 57600   // adjust as needed
#define DATAFLASH   0       // set to 0 for non-JeeLinks, else 4/8/16 (Mbit)
#define LED_PIN     9       // activity LED, comment out to disable
#endif

/// Save a few bytes of flash by declaring const if used more than once.
const char INVALID1[] PROGMEM = "\rInvalid\n";
const char INITFAIL[] PROGMEM = "config save failed\n";

#if TINY
// Serial support (output only) for Tiny supported by TinyDebugSerial
// http://www.ernstc.dk/arduino/tinycom.html
// 9600, 38400, or 115200
// hardware\jeelabs\avr\cores\tiny\TinyDebugSerial.h Modified
//  to moveTinyDebugSerial from PB0 to PA3 to match the Jeenode Micro V3 PCB layout
// Connect Tiny84 PA3 to USB-BUB RXD for serial output from sketch. // Jeenode AIO2
//
// With thanks for the inspiration by 2006 David A. Mellis and his AFSoftSerial code
//  All right reserved.
// Connect Tiny84 PA2 to USB-BUB TXD for serial input to sketch.        // Jeenode DIO2
// 9600 or 38400 at present.

#if SERIAL_BAUD == 9600
#define BITDELAY 54          // 9k6 @ 8MHz, 19k2 @16MHz
#endif
#if SERIAL_BAUD == 38400
#define BITDELAY 11         // 38k4 @ 8MHz, 76k8 @16MHz
#endif

#define MAX_NODES 14
#define _receivePin 8
static int _bitDelay;
static char _receive_buffer;
static uint8_t _receive_buffer_index;

ISR (PCINT0_vect) {
    char i, d = 0;
    if (digitalRead(_receivePin))       // PA2 = Jeenode DIO2
        return;             // not ready!
    whackDelay(_bitDelay - 8);
    for (i=0; i<8; i++) {
        whackDelay(_bitDelay*2 - 6);    // digitalread takes some time
        if (digitalRead(_receivePin)) // PA2 = Jeenode DIO2
            d |= (1 << i);
    }
    whackDelay(_bitDelay*2);
    if (_receive_buffer_index)
        return;
    _receive_buffer = d;                // save data
    _receive_buffer_index = 1;  // got a byte
}

// TODO: replace with code from the std avr libc library:
//  http://www.nongnu.org/avr-libc/user-manual/group__util__delay__basic.html
void whackDelay (uint16_t delay) {
    uint8_t tmp=0;

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
    uint8_t d;
    if (! _receive_buffer_index)
        return -1;
    d = _receive_buffer; // grab first and only byte
    _receive_buffer_index = 0;
    return d;
}

#else
#define MAX_NODES 30
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
/// For the EEPROM layout, see http://jeelabs.net/projects/jeelib/wiki/RF12demo
/// FIXME: this is incorrect - byte 0x00 Key storage for encryption algorithm
///      0x1F  note: can be overwritten if T84 is Node 15 or M328 is Node 31
/// ------------------------------------------------------------------------
/// byte 0x20 Node number in bits                   ***n nnnn                    // 1 - 31
///           Collect mode flag                     **0* ****   COLLECT 0x20     // Pass incoming without sending acks
///           Band                                  00** ****   Do not use       // Will hang the hardware
///             "                                   01** ****   433MHZ  0x40
///             "                                   10** ****   868MHZ  0x80
///             "                                   11** ****   915MHZ  0xC0
/// --------------------------------------------------------------------------------------------------------------------------
/// byte 0x021 Group number                                11010100    // i.e. 212 0xD4
/// byte 0x022 Flag Spares                                 11** ****   // Perhaps we could store the output in hex flag here
///            V10 indicator                               **1* ****   // This bit is set by versions of RF12Demo less than 11
///            Quiet mode                                  ***1 ****   // don't report bad packets
///            Frequency offset most significant bite      **** nnnn   // Can't treat as a 12 bit integer
/// byte 0x023 Frequency offset less significant bits      nnnn nnnn   //  because of little endian constraint
/// byte 0x024 Text description generate by RF12Demo       "T i20 g0 @868.0000 MHz"
///      0x03D   "                                         Padded at the end with NUL
/// byte 0x03E  CRC                                        CRC of values with offset 0x20
/// byte 0x03F   "                                         through to end of Text string, except NUL's
/// byte 0x040 Node 1 first packet capture
///      0x059   "
/// byte 0x060 Node 2 first packet capture
///      0x079   "
///      .....
///      0x1E0 Node 14 first packet capture      T84 maximum
///      0x1FF   "
///      .....
///      0x3E0 Node 30 first packet capture      M328 maximum
///      0x3FF   "
/// --------------------------------------------------------------------------------------------------------------------------
/// Useful url: http://blog.strobotics.com.au/2009/07/27/rfm12-tutorial-part-3a/
// 4 bit
// ----------------
// 8 bit

// RF12 configuration area
typedef struct {
    byte nodeId;            // used by rf12_config, offset 0
    byte group;             // used by rf12_config, offset 1
    byte format;            // used by rf12_config, offset 2
    byte hex_output   :2;   // 0 = dec, 1 = hex, 2 = hex+ascii
    byte collect_mode :1;   // 0 = ack, 1 = don't send acks
    byte quiet_mode   :1;   // 0 = show all, 1 = show only valid packets
    byte spare_flags  :4;  
    word frequency_offset;  // used by rf12_config, offset 4
    byte pad[RF12_EEPROM_SIZE-8];
    word crc;
} RF12Config;

static RF12Config config;
static char cmd;
static word value;
static byte stack[RF12_MAXDATA+4], top, sendLen, dest;
static byte testbuf[RF12_MAXDATA], testCounter;

static byte nodes[MAX_NODES+1];  // [0] is unused
static byte postingsIn, postingsOut;

static void showNibble (byte nibble) {
    char c = '0' + (nibble & 0x0F);
    if (c > '9')
        c += 7;
    Serial.print(c);
}

static void showByte (byte value) {
    if (config.hex_output) {
        showNibble(value >> 4);
        showNibble(value);
    } else
        Serial.print((int) value);
}

static void loadConfig () {
    eeprom_read_block(&config, RF12_EEPROM_ADDR, sizeof config);
}

static void saveConfig () {
    config.format = MAJOR_VERSION;
    config.crc = ~0;
    for (byte i = 0; i < sizeof config - 2; ++i)
        config.crc = _crc16_update(config.crc, ((byte*) &config)[i]);

    eeprom_write_block(&config, RF12_EEPROM_ADDR, sizeof config);
    Serial.println(sizeof config);

    if (rf12_configSilent())
        rf12_configDump();
    else
        showString(INITFAIL);
}

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
#if TINY
    "?\n"
#else
    "\n"
    "Available commands:\n"
    "  <nn> i     - set node ID (standard node ids are 1..30)\n"
    "  <n> b      - set MHz band (4 = 433, 8 = 868, 9 = 915)\n"
    "  <nnnn> o   - change frequency offset within the band (default 1600)\n"
    "               96 - 3903 is the range supported by the RFM12B\n"
    "  <nnn> g    - set network group (RFM12 only allows 212, 0 = any)\n"
    "  <n> c      - set collect mode (advanced, normally 0)\n"
    "  t          - broadcast max-size test packet, request ack\n"
    "  ...,<nn> a - send data packet to node <nn>, request ack\n"
    "  ...,<nn> s - send data packet to node <nn>, no ack\n"
    "  <n>,<c> j  - eeprom tools for node <n>, c = 42 backup RF12Demo config\n"
    "               restore with c = 123. Display otherwise\n"
    "  <n>,123 n  - remove node <n> entry from eeprom\n"
    "  <n> l      - turn activity LED on PB1 on or off\n"
    "  <n>,<d> p  - post semaphore <d> for node <n> to see with its next ack\n"
    "  <n> q      - set quiet mode (1 = don't report bad packets)\n"
    "  <n> x      - set reporting format (0 = decimal, 1 = hex, 2 = hex & ascii)\n"
    "  123 z      - total power down, needs a reset to start up again\n"
    "Remote control commands:\n"
    "  <hchi>,<hclo>,<addr>,<cmd> f     - FS20 command (868 MHz)\n"
    "  <addr>,<dev>,<on> k              - KAKU command (433 MHz)\n"
#endif
;

const char helpText2[] PROGMEM =
    "Flash storage (JeeLink only):\n"
    "    d                                  - dump all log markers\n"
    "    <sh>,<sl>,<t3>,<t2>,<t1>,<t0> r    - replay from specified marker\n"
    "    123,<bhi>,<blo> e                  - erase 4K block\n"
    "    12,34 w                            - wipe entire flash memory\n"
;

static void showString (PGM_P s) {
    for (;;) {
        char c = pgm_read_byte(s++);
        if (c == 0)
            break;
        if (c == '\n')
            showString(PSTR("\r"));
        Serial.print(c);
    }
}

static void showHelp () {
    showString(helpText1);
    if (df_present())
        showString(helpText2);
#if !TINY
    showString(PSTR("Current configuration:\n"));
    rf12_configDump();
#endif
}

static void handleInput (char c) {
    if ('0' <= c && c <= '9')
        value = 10 * value + c - '0';
    else if (c == ',') {
        if (top < sizeof stack)
            stack[top++] = value;
        value = 0;
    } else if ('a' <= c && c <='z') {
        showString(PSTR("> "));
        for (byte i = 0; i < top; ++i) {
            Serial.print((int) stack[i]);
            Serial.print(',');
        }
        Serial.print((int) value);
        Serial.println(c);

        switch (c) {
        default:
            showHelp();
            break;

        case 'i': // set node id
            if ((value > 0) && (value <= MAX_NODES + 1)) {
// Node 15 may exist on T84 but only as the RF12Demo node,
//  eeprom address +0, the encryption key storage will be
//  overwritten by the 42j command, similar for n31 on MEGA
                if (value < MAX_NODES) nodes[value] = 0;
            // Prevent auto allocation of this node number
                config.nodeId = (config.nodeId & 0xE0) + (value & 0x1F);
                saveConfig();
            } else {
                 showString(INVALID1);
            }
            break;

        case 'b': // set band: 4 = 433, 8 = 868, 9 = 915
            value = bandToFreq(value);
            if (value) {
                config.nodeId = (value << 6) + (config.nodeId & 0x3F);
                config.frequency_offset = 1600;
                saveConfig();
            } else {
                showString(INVALID1);
            }
            break;

        case 'o': // Increment frequency within band
                Serial.print(config.frequency_offset);
// Stay within your country's ISM spectrum management guidelines, i.e.
// allowable frequencies and their use when selecting operating frequencies.
                if (value) {
                    if ((value > 95) && (value < 3904)) {    // supported by the RFM12B
                        config.frequency_offset = value;
                        Serial.print('>');
                        Serial.println(config.frequency_offset);
                        saveConfig();
                    }
                    else {
                        showString(INVALID1);
                    }
                }
                else {
                    Serial.println();
                }
            break;

        case 'g': // set network group
            config.group = value;
            saveConfig();
            break;

        case 'c': // set collect mode (off = 0, on = 1)
            config.collect_mode = value;
            saveConfig();
            break;

        case 't': // broadcast a maximum size test packet, request an ack
            cmd = 'a';
            sendLen = RF12_MAXDATA;
            dest = 0;
            for (byte i = 0; i < RF12_MAXDATA; ++i)
                testbuf[i] = i + testCounter;
            showString(PSTR("test "));
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

        case 'z': // put the ATmega in ultra-low power mode (reset needed)
            if (value == 123)
                Sleep();
            break;

        case 'q': // turn quiet mode on or off (don't report bad packets)
            config.quiet_mode = value;
            saveConfig();
            break;

        case 'x': // set reporting mode to hex (1) or decimal (0)
            config.hex_output = value;
            saveConfig();
            break;

        case 'v': //display the interpreter version
            displayVersion();
            Serial.println();
            break;

#if !TINY
        case 'l': // turn activity LED on or off
            activityLed(value);
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
                showString(PSTR("erased\n"));
            }
            break;

        case 'j':
            if (stack[0] <= MAX_NODES) {
                const uint8_t *ee_entry = RF12_EEPROM_ADDR + (stack[0] * 32);
                for (byte i = 0; i < RF12_EEPROM_SIZE; ++i) {
                    byte b = eeprom_read_byte(ee_entry + i);    // http://forum.arduino.cc/index.php?topic=122140.0
                    showByte(b);
                    testbuf[i] = b;
                    if ((value == 42) && (stack[0] == 0)) {
                     eeprom_write_byte(RF12_EEPROM_ADDR + (((config.nodeId & RF12_HDR_MASK)*32) + i), b);
                    }
                }
                Serial.println();
                displayASCII(testbuf, RF12_EEPROM_SIZE);
            } else {
                showString(INVALID1);
            }
            if (!value) break;
            if (value == 42) {
                showString(PSTR("Backed Up\n"));
                break;
            }
            if ((value == 123) && (stack[0] == (config.nodeId & RF12_HDR_MASK))) {   // Only restore this NodeId
                const uint8_t *ee_shadow = RF12_EEPROM_ADDR + ((config.nodeId & RF12_HDR_MASK)*32);
                // Check CRC to be restored
                word crc = ~0;
                for (byte i = 0; i < RF12_EEPROM_SIZE; ++i)
                    crc = _crc16_update(crc, eeprom_read_byte(ee_shadow + i));
                if (crc)
                    showString(PSTR("Bad CRC\n"));
                else {
                    for (byte i = 0; i < RF12_EEPROM_SIZE; ++i) {
                        byte b = eeprom_read_byte((ee_shadow) + i);
                        showByte(b);
                        eeprom_write_byte((RF12_EEPROM_ADDR) + i, b);
                    }
                    Serial.println();
                    showString(PSTR("Restored\n"));
                    }
                if (rf12_configSilent())
                    loadConfig();
                else
                    showString(INITFAIL);
            } else {
                showString(INVALID1);
            }
        break;
#endif

        case 'n': // Clear node entries in RAM & eeprom
            if ((stack[0] > 0) && (stack[0] <= MAX_NODES) && (value == 123) && (nodes[stack[0]] == 0)) {
                nodes[stack[0]] = 0xFF; // Clear RAM entry
                for (byte i = 0; i < (RF12_EEPROM_SIZE); ++i) {
                    eeprom_write_byte(RF12_EEPROM_ADDR + (stack[0]*32) + i, 0xFF);  // Clear complete eeprom entry
                }
            } else {
                showString(INVALID1);
            }
            break;

        case 'p':
            // Post a command for a remote node, to be collected along with the next ACK
            // Format is 20,127p where 20 is the node number and 127 is the desired value to be posted
            // stack[0] contains the target node
            // and value contains the command to be posted
            if ((!stack[0]) && (!value)) {
                Serial.print((int)postingsIn);
                Serial.print(',');
                Serial.println((int)postingsOut);
                nodesShow();
            } else {
                if ((stack[0] !=(config.nodeId & RF12_HDR_MASK)) && (stack[0] <= MAX_NODES) && (value < 255) && (nodes[stack[0]] == 0)) {       // No posting to special(31) or overwriting pending post
                    nodes[stack[0]] = value;
                    postingsIn++;                        // Count post
                }
                else
                {
                    showString(INVALID1);
                }
            }
            break;
        } // End Switch

        value = top = 0;
        memset(stack, 0, sizeof stack);
    } else if (c == '>') {
        // special case, send to specific band and group, and don't echo cmd
        // input: band,group,node,header,data...
        stack[top++] = value;
        // TODO: frequency offset is kept at default value here, is that ok?
        rf12_initialize(stack[2], bandToFreq(stack[0]), stack[1]);
        rf12_sendNow(stack[3], stack + 4, top - 4);
        rf12_sendWait(2);
        rf12_configSilent();
        value = top = 0;
        memset(stack, 0, sizeof stack);
    } else if (' ' < c && c < 'A') {
        showHelp();
    }
}

static void displayASCII (const uint8_t* data, byte count) {
    for (byte i = 0; i < count; ++i) {
        Serial.print(' ');
        char c = (char) data[i];
        Serial.print(c < ' ' || c > '~' ? '.' : c);
    }
    Serial.println();
}

void displayVersion () {
    showString(VERSION);
#if TINY
    showString(PSTR("Tiny "));
#endif
}

void Sleep () {
    showString(PSTR(" sleeping"));
    Serial.flush();
    rf12_sleep(RF12_SLEEP);
    cli();
    Sleepy::powerDown();
}

void setup () {
    delay(1000); // FIXME: do we need this?

#if TINY
    PCMSK0 |= (1<<PCINT2);  // tell pin change mask to listen to PA2
    GIMSK    |= (1<<PCIE0); // enable PCINT interrupt in general interrupt mask
    whackDelay(_bitDelay*2); // if we were low this establishes the end
    pinMode(_receivePin, INPUT);        // PA2
    digitalWrite(_receivePin, HIGH);    // pullup!
    _bitDelay = BITDELAY;
#endif

    Serial.begin(SERIAL_BAUD);
    displayVersion();

    if (rf12_configSilent())
        loadConfig();
    else {
        memset(&config, 0, sizeof config);
        config.nodeId = 0x81;       // 868 MHz, node 1
        config.group = 0xD4;        // default group 212
        config.frequency_offset = 1600;
        config.quiet_mode = true;   // Default flags, quiet on
        saveConfig();
        rf12_initialize(config.nodeId & RF12_HDR_MASK,
                         config.nodeId >> 6, config.group);
    }
    
    rf12_configDump();

    // Initialise node table
    Serial.print("Node Table:");
    for (byte i = 1; i <= MAX_NODES; i++) {
        nodes[i] = eeprom_read_byte(RF12_EEPROM_ADDR + (i * RF12_EEPROM_SIZE)); // http://forum.arduino.cc/index.php/topic,140376.msg1054626.html
        if (nodes[i] != 0xFF)
            nodes[i] = 0;       // Indicate no post waiting for node!
            Serial.print(nodes[i]);
    }
    Serial.println();

    nodes[(config.nodeId & RF12_HDR_MASK)] = 0;  // Prevent allocation of this nodes number.

    df_initialize();
#if !TINY
    showHelp();
#endif
} // Setup

/// Display stored nodes and show the command queued for each node
/// the command queue is not preserved through a restart of RF12Demo
void nodesShow() {
    for (byte i = 1; i <= MAX_NODES; i++) {
        if (nodes[i] != 0xFF) {                                     // Entry 0 is unused at present
            Serial.print((int)i);
            showString(PSTR("("));
            Serial.print((int)nodes[i]);
            showString(PSTR(") "));
        }
    }
    Serial.println();
}

void loop () {
#if TINY
    handleInput(inChar());
#else
    if (Serial.available())
        handleInput(Serial.read());
#endif
    if (rf12_recvDone()) {
        byte n = rf12_len;
        if (rf12_crc == 0)
            showString(PSTR("OK"));
        else {
            if (config.quiet_mode)
                return;
            showString(PSTR(" ?"));
            if (n > 20) // print at most 20 bytes if crc is wrong
                n = 20;
        }
        if (config.hex_output)
            showString(PSTR("X"));
        if (config.group == 0) {
            showString(PSTR(" G"));
            showByte(rf12_grp);
        }
        Serial.print(' ');
        showByte(rf12_hdr);
        for (byte i = 0; i < n; ++i) {
            if (!config.hex_output)
                Serial.print(' ');
            showByte(rf12_data[i]);
        }
#if RF69_COMPAT
        showString(PSTR(" ("));
        if (config.hex_output)
                showByte(RF69::rssi);
        else
                Serial.print(-(RF69::rssi>>1));
        showString(PSTR(") "));
#endif
        Serial.println();
        if (config.hex_output > 1) { // also print a line as ascii
            showString(PSTR("ASC "));
            if (config.group == 0) {
                showString(PSTR(" II "));
            }
            Serial.print(rf12_hdr & RF12_HDR_DST ? '>' : '<');
            Serial.print((char) ('@' + (rf12_hdr & RF12_HDR_MASK)));
            displayASCII((const uint8_t*) rf12_data, n);
        }
        if (rf12_crc == 0) {
            activityLed(1);
#if !TINY
            if (df_present())
                df_append((const char*) rf12_data - 2, rf12_len + 2);
#endif
                if (((rf12_hdr & (RF12_HDR_MASK | RF12_HDR_DST)) <= MAX_NODES) &&        // Source node packets only
                     (nodes[(rf12_hdr & RF12_HDR_MASK)] == 0xFF)) {
                        byte len = 32;
                        if (rf12_data[0] == 0xFF)                                                                // New nodes cannot be learned if packet begins 0xFF
                            rf12_data[0] = 0xFE;                                                                     // so lets drop the low order bit in byte 0
                        showString(PSTR("New Node "));
                        showByte(rf12_hdr & RF12_HDR_MASK);
                        Serial.println();
                        nodes[(rf12_hdr & RF12_HDR_MASK)] = 0;
                        if (rf12_len < 32)
                            len = rf12_len;
                            for (byte i = 0; i < len; ++i) {    // variable n
                            eeprom_write_byte(RF12_EEPROM_ADDR + (((rf12_hdr & RF12_HDR_MASK) * 32) + i), rf12_data[i]);
                        }
                }

            if (RF12_WANTS_ACK && (config.collect_mode) == 0) {
                showString(PSTR(" -> ack\n"));
                testCounter = 0;

                if ((rf12_hdr & (RF12_HDR_MASK | RF12_HDR_DST)) == 31) {                    // Special Node 31 source node
                    for (byte i = 1; i <= MAX_NODES; i++) {
                        if (nodes[i] == 0xFF) {
                            testbuf[0] = i + 0xE0;                                                                          // Change Node number request - matched in RF12Tune3
                            testCounter = 1;
                            showString(PSTR("Node allocation "));
                            showByte(i);
                            Serial.println();
                            break;
                        }
                    }
                } else {
                    if (!(rf12_hdr & RF12_HDR_DST) && (nodes[(rf12_hdr & RF12_HDR_MASK)] != 0) &&
                             (nodes[(rf12_hdr & RF12_HDR_MASK)] != 0xFF)) {                  // Sources Nodes only!
                        testbuf[0] = nodes[(rf12_hdr & RF12_HDR_MASK)];                      // Pick up posted value
                        nodes[(rf12_hdr & RF12_HDR_MASK)] = 0;                                       // Assume it will be delivered.
                        testCounter = 1;
                        showString(PSTR("Posted "));
                        showByte(rf12_hdr & RF12_HDR_MASK);
                        Serial.print(',');
                        showByte(testbuf[0]);
                        postingsOut++;                  // Count as delivered
                        Serial.println();
                    }
                }

                rf12_sendStart(RF12_ACK_REPLY, testbuf, testCounter);
            }
            activityLed(0);
        }
    }

    if (cmd && rf12_canSend()) {
        activityLed(1);

        showString(PSTR(" -> "));
        Serial.print((int) sendLen);
        showString(PSTR(" b\n"));
        byte header = cmd == 'a' ? RF12_HDR_ACK : 0;
        if (dest)
            header |= RF12_HDR_DST | dest;
        rf12_sendStart(header, testbuf, sendLen);
        cmd = 0;

        activityLed(0);
    }
}
