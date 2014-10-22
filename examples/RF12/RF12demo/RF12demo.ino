/// @dir RF12demo
/// Configure some values in EEPROM for easy config of the RF12 later on.
// 2009-05-06 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

// this version adds flash memory support, 2009-11-19
// Adding frequency features, author JohnO, 2013-09-05
// Major EEPROM format change, refactoring, and cleanup for v12, 2014-02-13

#define RF69_COMPAT 0 // define this to use the RF69 driver i.s.o. RF12

#include <JeeLib.h>
#include <util/crc16.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <util/parity.h>

#define MAJOR_VERSION RF12_EEPROM_VERSION // bump when EEPROM layout changes
#define MINOR_VERSION 2                   // bump on other non-trivial changes
#define VERSION "[RF12demo.12]"           // keep in sync with the above

#if defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny44__)
#define TINY        1
#define SERIAL_BAUD 38400   // can only be 9600 or 38400
#define DATAFLASH   0       // do not change
#undef  LED_PIN             // do not change
#define rf12_configDump()   // disabled
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
// hardware\jeelabs\avr\cores\tiny\TinyDebugSerial.h Modified to
// moveTinyDebugSerial from PB0 to PA3 to match the Jeenode Micro V3 PCB layout
// Connect Tiny84 PA3 to USB-BUB RXD for serial output from sketch.
// Jeenode AIO2
//
// With thanks for the inspiration by 2006 David A. Mellis and his AFSoftSerial
// code. All right reserved.
// Connect Tiny84 PA2 to USB-BUB TXD for serial input to sketch.
// Jeenode DIO2
// 9600 or 38400 at present.

#if SERIAL_BAUD == 9600
#define BITDELAY 54          // 9k6 @ 8MHz, 19k2 @16MHz
#endif
#if SERIAL_BAUD == 38400
#define BITDELAY 11         // 38k4 @ 8MHz, 76k8 @16MHz
#endif

#define _receivePin 8
static int _bitDelay;
static char _receive_buffer;
static byte _receive_buffer_index;

static void showString (PGM_P s); // forward declaration

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
void whackDelay (word delay) {
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
static byte testCounter;

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
    for (byte i = 0; i < sizeof config; ++ i)
        ((byte*) &config)[i] = eeprom_read_byte(RF12_EEPROM_ADDR + i);
}

static void saveConfig () {
    config.format = MAJOR_VERSION;
    config.crc = calcCrc(&config, sizeof config - 2);
    // eeprom_write_block(&config, RF12_EEPROM_ADDR, sizeof config);
    // this uses 170 bytes less flash than eeprom_write_block(), no idea why
    eeprom_write_byte(RF12_EEPROM_ADDR, ((byte*) &config)[0]);
    for (byte i = 0; i < sizeof config; ++ i)
        eeprom_write_byte(RF12_EEPROM_ADDR + i, ((byte*) &config)[i]);

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
    "\n"
    "Available commands:\n"
    "  <nn> i     - set node ID (standard node ids are 1..30)\n"
    "  <n> b      - set MHz band (4 = 433, 8 = 868, 9 = 915)\n"
    "  <nnnn> o   - change frequency offset within the band (default 1600)\n"
    "               96..3903 is the range supported by the RFM12B\n"
    "  <nnn> g    - set network group (RFM12 only allows 212, 0 = any)\n"
    "  <n> c      - set collect mode (advanced, normally 0)\n"
    "  t          - broadcast max-size test packet, request ack\n"
    "  ...,<nn> a - send data packet to node <nn>, request ack\n"
    "  ...,<nn> s - send data packet to node <nn>, no ack\n"
    "  <n> q      - set quiet mode (1 = don't report bad packets)\n"
    "  <n> x      - set reporting format (0: decimal, 1: hex, 2: hex+ascii)\n"
    "  123 z      - total power down, needs a reset to start up again\n"
    "Remote control commands:\n"
    "  <hchi>,<hclo>,<addr>,<cmd> f     - FS20 command (868 MHz)\n"
    "  <addr>,<dev>,<on> k              - KAKU command (433 MHz)\n"
;

const char helpText2[] PROGMEM =
    "Flash storage (JeeLink only):\n"
    "    d                                  - dump all log markers\n"
    "    <sh>,<sl>,<t3>,<t2>,<t1>,<t0> r    - replay from specified marker\n"
    "    123,<bhi>,<blo> e                  - erase 4K block\n"
    "    12,34 w                            - wipe entire flash memory\n"
;

static void showHelp () {
#if TINY
    showString(PSTR("?\n"));
#else
    showString(helpText1);
    if (df_present())
        showString(helpText2);
    showString(PSTR("Current configuration:\n"));
    rf12_configDump();
#endif
}

static void handleInput (char c) {
    if ('0' <= c && c <= '9') {
        value = 10 * value + c - '0';
        return;
    }

    if (c == ',') {
        if (top < sizeof stack)
            stack[top++] = value; // truncated to 8 bits
        value = 0;
        return;
    }

    if ('a' <= c && c <= 'z') {
        showString(PSTR("> "));
        for (byte i = 0; i < top; ++i) {
            Serial.print((word) stack[i]);
            printOneChar(',');
        }
        Serial.print(value);
        Serial.println(c);
    }

    // keeping this out of the switch reduces code size (smaller branch table)
    if (c == '>') {
        // special case, send to specific band and group, and don't echo cmd
        // input: band,group,node,header,data...
        stack[top++] = value;
        // TODO: frequency offset is taken from global config, is that ok?
        rf12_initialize(stack[2], bandToFreq(stack[0]), stack[1],
                            config.frequency_offset);
        rf12_sendNow(stack[3], stack + 4, top - 4);
        rf12_sendWait(2);
        rf12_configSilent();
    } else if (c > ' ') {
        switch (c) {

        case 'i': // set node id
            config.nodeId = (config.nodeId & 0xE0) + (value & 0x1F);
            saveConfig();
            break;

        case 'b': // set band: 4 = 433, 8 = 868, 9 = 915
            value = bandToFreq(value);
            if (value) {
                config.nodeId = (value << 6) + (config.nodeId & 0x3F);
                config.frequency_offset = 1600;
                saveConfig();
            }
            break;

        case 'o': { // Increment frequency within band
// Stay within your country's ISM spectrum management guidelines, i.e.
// allowable frequencies and their use when selecting operating frequencies.
            if ((value > 95) && (value < 3904)) { // supported by RFM12B
                config.frequency_offset = value;
                saveConfig();
            }
#if !TINY
            // this code adds about 400 bytes to flash memory use
            // display the exact frequency associated with this setting
            byte freq = 0, band = config.nodeId >> 6;
            switch (band) {
                case RF12_433MHZ: freq = 43; break;
                case RF12_868MHZ: freq = 86; break;
                case RF12_915MHZ: freq = 90; break;
            }
            uint32_t f1 = freq * 100000L + band * 25L * config.frequency_offset;
            Serial.print((word) (f1 / 10000));
            printOneChar('.');
            word f2 = f1 % 10000;
            // tedious, but this avoids introducing floating point
            printOneChar('0' + f2 / 1000);
            printOneChar('0' + (f2 / 100) % 10);
            printOneChar('0' + (f2 / 10) % 10);
            printOneChar('0' + f2 % 10);
            Serial.println(" MHz");
#endif
            break;
        }

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
                stack[i] = i + testCounter;
            showString(PSTR("test "));
            showByte(testCounter); // first byte in test buffer
            ++testCounter;
            break;

        case 'a': // send packet to node ID N, request an ack
        case 's': // send packet to node ID N, no ack
            cmd = c;
            sendLen = top;
            dest = value;
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
            if (value == 123) {
                showString(PSTR(" Zzz...\n"));
                Serial.flush();
                rf12_sleep(RF12_SLEEP);
                cli();
                Sleepy::powerDown();
            }
            break;

        case 'q': // turn quiet mode on or off (don't report bad packets)
            config.quiet_mode = value;
            saveConfig();
            break;

        case 'x': // set reporting mode to decimal (0), hex (1), hex+ascii (2)
            config.hex_output = value;
            saveConfig();
            break;

        case 'v': //display the interpreter version and configuration
            displayVersion();
            rf12_configDump();
#if TINY
            Serial.println();
#endif
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

        default:
            showHelp();
        }
    }

    value = top = 0;
}

static void displayASCII (const byte* data, byte count) {
    for (byte i = 0; i < count; ++i) {
        printOneChar(' ');
        char c = (char) data[i];
        printOneChar(c < ' ' || c > '~' ? '.' : c);
    }
    Serial.println();
}

void setup () {
    delay(100); // shortened for now. Handy with JeeNode Micro V1 where ISP
                // interaction can be upset by RF12B startup process.

#if TINY
    PCMSK0 |= (1<<PCINT2);  // tell pin change mask to listen to PA2
    GIMSK |= (1<<PCIE0);    // enable PCINT interrupt in general interrupt mask
    // FIXME: _bitDelay has not yet been initialised here !?
    whackDelay(_bitDelay*2); // if we were low this establishes the end
    pinMode(_receivePin, INPUT);        // PA2
    digitalWrite(_receivePin, HIGH);    // pullup!
    _bitDelay = BITDELAY;
#endif

    Serial.begin(SERIAL_BAUD);
    Serial.println();
    displayVersion();

    if (rf12_configSilent()) {
        loadConfig();
    } else {
        memset(&config, 0, sizeof config);
        config.nodeId = 0x81;       // 868 MHz, node 1
        config.group = 0xD4;        // default group 212
        config.frequency_offset = 1600;
        config.quiet_mode = true;   // Default flags, quiet on
        saveConfig();
        rf12_configSilent();
    }

    rf12_configDump();
    df_initialize();
#if !TINY
    showHelp();
#endif
}

void loop () {
#if TINY
    if (_receive_buffer_index)
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
            printOneChar('X');
        if (config.group == 0) {
            showString(PSTR(" G"));
            showByte(rf12_grp);
        }
        printOneChar(' ');
        showByte(rf12_hdr);
        for (byte i = 0; i < n; ++i) {
            if (!config.hex_output)
                printOneChar(' ');
            showByte(rf12_data[i]);
        }
#if RF69_COMPAT
        // display RSSI value after packet data
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
            printOneChar(rf12_hdr & RF12_HDR_DST ? '>' : '<');
            printOneChar('@' + (rf12_hdr & RF12_HDR_MASK));
            displayASCII((const byte*) rf12_data, n);
        }

        if (rf12_crc == 0) {
            activityLed(1);

            if (df_present())
                df_append((const char*) rf12_data - 2, rf12_len + 2);

            if (RF12_WANTS_ACK && (config.collect_mode) == 0) {
                showString(PSTR(" -> ack\n"));
                rf12_sendStart(RF12_ACK_REPLY, 0, 0);
            }
            activityLed(0);
        }
    }

    if (cmd && rf12_canSend()) {
        activityLed(1);

        showString(PSTR(" -> "));
        Serial.print((word) sendLen);
        showString(PSTR(" b\n"));
        byte header = cmd == 'a' ? RF12_HDR_ACK : 0;
        if (dest)
            header |= RF12_HDR_DST | dest;
        rf12_sendStart(header, stack, sendLen);
        cmd = 0;

        activityLed(0);
    }
}
