/// @dir isp_capture
/// Variant of isp_flash which supports the Flash Board with on-board EEPROM.

// Originally derived from http://arduino.cc/en/Tutorial/ArduinoISP
// see http://jeelabs.org/2010/04/24/isp-plug/

// This is an STK500-/AVRISP-compatible programmer with a twist: when used with
// the Flash Board which has EEPROM memory on board, a copy of all programming
// commands is captured in EEPROM, as well as a copy of the replies sent back to
// the programmer. Everything is stored: data bytes, fuses, verifications, etc.
// Data capture ends when no further commands have been received for 3 seconds.
//
// There is a button on the Flash Board, which starts the programmer in a
// special "playback" mode, re-playing all the commands saved in EEPROM, and
// intercepting all replies to compare them with the data stored in EEPROM.
// In other words: pressing the button will repeat the last captured AVRISP
// programming cycle, without requiring a connected PC or any software.
// Successful programming is indicated with a brief but regularly flashing LED.
//
// This mechanism is transparent: the PC can treat this programmer like any
// other STK500-/AVRISP-compatible programmer, for ATmega / ATtiny / anything.
// The only difference is that this programmer can then be used stand-alone.
//
// jcw, 2010-04-26

// Two improvements, 2010-05-18:
//  - Start with slow pulses until fuse settings have been performed, to avoid
//    with factory-fresh chips preset to a 1 Mhz clock.
//  - Ignore fuse reads in playback mode, because they may differ, depending on
//    what they were previously set to (can use serial hookup to see details).

#include <JeeLib.h>

// set to 1 to get some more output on the serial port during playback
#define DEBUG 1

// set to 1 to save in MemoryStream right away (this may have timing issues)
#define IMMEDIATE_SAVE 0

// this optimization cuts the programming time by half, approximately
// the code needs to be adjusted if the pin assignments are changed
#define OPTIMIZED   1   // use fast pin I/O, avoid digitalRead / digitalWrite

// pin assignments for the target ISP connnector
#define PIN_SCK     14  // AIO1
#define PIN_MISO    4   // DIO1
#define PIN_MOSI    17  // AIO4
#define PIN_RESET   7   // DIO4

#define LED_PMODE   15  // on while programming (LED to VCC via resistor)
#define START_BTN   5   // DIO2 - active low - starts programming target

// original comments:
//
// October 2009 by David A. Mellis
// - Added support for the read signature command
// 
// February 2009 by Randall Bohn
// - Added support for writing to EEPROM (what took so long?)
// Windows users should consider WinAVR's avrdude instead of the
// avrdude included with Arduino software.
//
// January 2008 by Randall Bohn
// - Thanks to Amplificar for helping me with the STK500 protocol
// - The AVRISP/STK500 (mk I) protocol is used in the arduino bootloader
// - The SPI functions herein were developed for the AVR910_ARD programmer 
// - More information at http://code.google.com/p/mega-isp

#define HWVER 2
#define SWMAJ 1
#define SWMIN 18

// STK Definitions
#define STK_OK      '\x10'
#define STK_FAILED  '\x11'
#define STK_UNKNOWN '\x12'
#define STK_INSYNC  '\x14'
#define STK_NOSYNC  '\x15'
#define CRC_EOP     '\x20' //ok it is a space...

// access to the 128 kbyte on-board EEPROM memory
PortI2C i2cBus (3);
MemoryPlug mem (i2cBus);
MemoryStream stream (mem, 1); // starts on page 1, page 0 is for parameters

#define DONE_TIMEOUT 3000   // recording is done when idle for 3 seconds
MilliTimer doneTimer;       // will fire when recording has finished

enum { RECORDING, PASS_THROUGH, PLAYBACK } mode; // programmer mode

int here;           // address for reading and writing, set by 'U' command
byte data[256];     // global block storage for the avrisp() code
byte recbuf[127];   // used to capture data bytes during recording
byte recfill;       // fill index into recbuf or remaining count during playback
byte recdir;        // recording direction: INPUT = cmds, OUTPUT = replies
word start;         // start time, used to track total programming time
byte speed;         // used to slow down ISP pulses in case chip runs @ 1 Mhz
byte lastC, lastA;  // need to avoid playback mismatch on fuse reads, and such

#if !IMMEDIATE_SAVE
byte buffer [500];
int bufpos;
#endif

struct {
    byte devicecode;
    byte revision;
    byte progtype;
    byte parmode;
    byte polling;
    byte selftimed;
    byte lockbytes;
    byte fusebytes;
    byte sig0, sig1, sig2; // copy of the read sig request
    int flashpoll;
    int eeprompoll;
    int pagesize;
    int eepromsize;
    long flashsize;
    // programmer info
    long programsize;
} param;

// see http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1235780325/10#10
static void (*reset)() = 0;

static void setLed (byte on) {
    digitalWrite(LED_PMODE, !on);
}

static byte getButton () {
    return digitalRead(START_BTN) == 0;
}

static void saveToStream () {
#if !IMMEDIATE_SAVE
    for (int i = 0; i < bufpos; ++i)
        stream.put(buffer[i]);
    bufpos = 0;
#endif
}

// used to record both incoming commands and outgoing replies
// don't let the recording buffer fill up beyond 127 bytes
// flush the recording buffer when full or when the recording direction changes
// store buffer count in front of the data bytes (bits 0..6), bit 7 = direction
static void record (byte direction, byte value) {
    if ((recfill >= sizeof recbuf || direction != recdir) && recfill > 0) {
        byte marker = recdir == OUTPUT ? recfill | 0x80 : recfill;
#if IMMEDIATE_SAVE
        stream.put(marker);
        for (byte i = 0; i < recfill; ++i)
            stream.put(recbuf[i]);
#else
        // safety measure: if buffer doesn't have enough room, clear it first
        // shouldn't happen due to recordFlush calls in program_page & read_page
        if (bufpos + recfill + 1 >= sizeof buffer)
            saveToStream();
        // save recbuf in an extra buffer to postpone the slow EEPROM writes
        buffer[bufpos++] = marker;
        for (byte i = 0; i < recfill; ++i)
            buffer[bufpos++] = recbuf[i];
#endif
        recfill = 0;
    }
    recbuf[recfill++] = value;
    recdir = direction;
    // keep arming (i.e. extending) the done timer as long as there is data
    doneTimer.set(DONE_TIMEOUT);
}

static void recordFlush () {
    record(!recdir, 0); // force recbuf flushing by switching mode
    recfill = 0;
    saveToStream();
    stream.flush();
}

// Success is indicated with a brief LED flash every second for 4 minutes.
// This code should be changed to use the watchdog and run in low-power mode.
// TODO: wake up from button press after power down, using pin-change interrupt.

static void successBlink () {
    MilliTimer t;
    for (byte i = 0; i < 240; ++i) {
        setLed(1);
        t.set(50);
        while (!t.poll())
            if (getButton())
                return;
        setLed(0);
        t.set(950);
        while (!t.poll())
            if (getButton())
                return;
    }
    // power down completely after 4 minutes
    Serial.flush();
    rf12_initialize(1, RF12_868MHZ);
    rf12_sleep(RF12_SLEEP);
    cli();
    Sleepy::powerDown();
    // never returns, needs hard reset to start up again
}

// play back one byte from EEPROM, as captured in an earlier recording session
// arg specifies whether the data is an INPUT or an OUTPUT capture
// if it doesn't match, then something went wrong: report on serial and reset
// if EOF is reached, then report success and some stats, and also reset
static byte playback (byte direction) {
    if (recfill == 0) {
        // Serial.print('<');
        // Serial.print(stream.position(0));
        // Serial.print('>');
        
        if (stream.position(0) >= param.programsize) {
            start = millis() / 100 - start;
            Serial.print("Done in ");
            Serial.print(start / 10);
            Serial.print('.');
            Serial.print(start % 10);
            Serial.println(" seconds.");
            // wait for button release to avoid immediately restarting
            while (getButton())
                ;
            successBlink();
            reset();
        }
        
        byte b = stream.get();
        recdir = b & 0x80 ? OUTPUT : INPUT;
        recfill = b & 0x7F;
    }
    if (direction != recdir) {
        Serial.print("Direction mismatch @ ");
        Serial.print(stream.position(0));
        Serial.print(" : ");
        Serial.println(direction, DEC);
        delay(10); // let UART drain
        reset();
    }
    --recfill;
    return stream.get();
}

static byte getch() {
    byte b;
    if (mode == PLAYBACK)
        b = playback(INPUT);
    else {
        while (!Serial.available())
            ;
        b = Serial.read();
        if (mode == RECORDING)
            record(INPUT, b);
    }     
    return b;
}

static void putch(byte c) {
    if (mode == PLAYBACK) {
        byte e = playback(OUTPUT);
        if (c != e) {
            Serial.print("Reply mismatch @ ");
            Serial.print(stream.position(0));
            Serial.print(" : ");
            Serial.print(c, HEX);
            Serial.print('/');
            Serial.print(e, HEX);
            // Special code to avoid aborting on a playback mismatch, when
            // reading fuse bits or the RC calibration byte, because these
            // will differ between brand-new and previously-used chips.
            switch (lastC == 'V' ? lastA : 0) {
                default:
                    Serial.println();
                    delay(100); // let UART drain
                    reset();
                    // never returns
                case 0x38:
                case 0x50:
                case 0x58:
                    Serial.println(" - ignored");
            }
        }
    } else {
        Serial.print((char) c);
        if (mode == RECORDING)
            record(OUTPUT, c);
    }
}

static void readbytes(int n) {
    for (byte x = 0; x < n; x++)
        data[x] = getch();
}

static void spi_init() {
    digitalWrite(PIN_SCK, 1);
    digitalWrite(PIN_MISO, 1);
    digitalWrite(PIN_MOSI, 1);
    digitalWrite(PIN_RESET, 1);

    pinMode(PIN_SCK, OUTPUT);
    pinMode(PIN_MISO, INPUT);
    pinMode(PIN_MOSI, OUTPUT);
    pinMode(PIN_RESET, OUTPUT);
}

static byte spi_send(byte b) {
    byte reply = 0;
    for (byte i = 0; i < 8; ++i) {
#if OPTIMIZED
        // this is hardwired for pin 17, i.e. PC3
        bitWrite(PORTC, 3, b >> 7);
        // this is hardwired for pin 14, i.e. PC0
        delayMicroseconds(speed);
        bitClear(PORTC, 0);
        delayMicroseconds(speed);
        bitSet(PORTC, 0);
        delayMicroseconds(speed);
        b <<= 1;
        // this is hardwired for pin 4, i.e. PD4
        reply = (reply << 1) | bitRead(PIND, 4);
#else
        digitalWrite(PIN_MOSI, b & 0x80);
        digitalWrite(PIN_SCK, 0); // slow pulse, max 60KHz
        digitalWrite(PIN_SCK, 1);
        b <<= 1;
        reply = (reply << 1) | digitalRead(PIN_MISO);
#endif
    }
    return reply;
}

static byte spi_transaction(byte a, byte b, byte c, byte d) {
    spi_send(a); 
    spi_send(b);
    spi_send(c);
    return spi_send(d);
}

static byte spi_transaction_wait(byte a, byte b, byte c, byte d) {
    byte reply = spi_transaction(a, b, c, d);
    while (spi_transaction(0xF0, 0, 0, 0) & 1)
        ;
    return reply;
}

static void empty_reply() {
    if (getch() == CRC_EOP) {
        putch(STK_INSYNC);
        putch(STK_OK);
    } else
        putch(STK_NOSYNC);
}

static void breply(byte b) {
    if (getch() == CRC_EOP) {
        putch(STK_INSYNC);
        putch(b);
        putch(STK_OK);
    } else
        putch(STK_NOSYNC);
}

static void get_version(byte c) {
    switch(c) {
        case 0x80:  breply(HWVER); break;
        case 0x81:  breply(SWMAJ); break;
        case 0x82:  breply(SWMIN); break;
        case 0x93:  breply('S'); break;
        default:    breply(0);
    }
}

static void set_parameters() {
    // call this after reading parameter packet into data[]
    if (DEBUG && mode == PLAYBACK) {
        for (byte i = 0; i < 20; ++i) {
            Serial.print(' ');
            Serial.print(data[i], HEX);
        }
        Serial.println();
    }
    memcpy(&param, data, 9);
    // following fields are big endian
    param.eeprompoll = data[10] * 0x0100 + data[11];
    param.pagesize = data[12] * 0x0100 + data[13];
    param.eepromsize = data[14] * 0x0100 + data[15];
    param.flashsize = (data[16] << 8) + data[17];
    param.flashsize <<= 16;
    param.flashsize |= (word) ((data[18] << 8) + data[19]);
}

static void start_pmode() {
    spi_init();
    // following delays may not work on all targets...
    pinMode(PIN_RESET, OUTPUT);
    digitalWrite(PIN_RESET, HIGH);
    pinMode(PIN_SCK, OUTPUT);
    digitalWrite(PIN_SCK, LOW);
    delay(50);
    digitalWrite(PIN_RESET, LOW);
    delay(50);
    pinMode(PIN_MISO, INPUT);
    pinMode(PIN_MOSI, OUTPUT);
    spi_transaction_wait(0xAC, 0x53, 0x00, 0x00);
    setLed(1); 
}

static void end_pmode() {
    setLed(0);
    pinMode(PIN_MISO, INPUT);
    pinMode(PIN_MOSI, INPUT);
    pinMode(PIN_SCK, INPUT);
    pinMode(PIN_RESET, INPUT);
}

static void universal() {
    readbytes(4);
    lastA = data[0]; // remember last type of command, see putch()
    if (DEBUG && mode == PLAYBACK) {
        Serial.print(lastA, HEX);
        Serial.print(':');
    }
    breply(spi_transaction_wait(data[0], data[1], data[2], data[3]));
}

static void flash(byte hilo, int addr, byte value) {
    spi_transaction_wait(0x40+8*hilo, addr >> 8, addr, value);
}

static void commit(int addr) {
    spi_transaction_wait(0x4C, addr >> 8, addr, 0);
}

static int current_page(int addr) {
    return here & ~(param.pagesize/2-1);
}

static byte write_flash(int length) {
    if (param.pagesize < 1) return STK_FAILED;
    int page = current_page(here);
    int x = 0;
    while (x < length) {
        if (page != current_page(here)) {
            commit(page);
            page = current_page(here);
        }
        flash(LOW, here, data[x++]);
        flash(HIGH, here, data[x++]);
        here++;
    }
    commit(page);
    return STK_OK;
}

static byte write_eeprom(int length) {
    // this writes byte-by-byte, page writing may be faster (4 bytes at a time)
    for (int x = 0; x < length; x++)
        spi_transaction_wait(0xC0, 0x00, here*2+x, data[x]);
    return STK_OK;
}

static void program_page() {
    char result = STK_FAILED;
    int length = 256 * getch();
    length += getch();
    if (length > 256) {
        putch(STK_FAILED);
        return;
    }
    char memtype = getch();
    readbytes(length);
    if (getch() == CRC_EOP) {
        putch(STK_INSYNC);
        if (memtype == 'F') result = write_flash(length);
        if (memtype == 'E') result = write_eeprom(length);
        // flush saved-up data here, while it's allowed to take some time
        if (mode == RECORDING)
            recordFlush();
        putch(result);
    } else
        putch(STK_NOSYNC);
}

static byte flash_read(byte hilo, int addr) {
    return spi_transaction(0x20 + hilo * 8, addr >> 8, addr, 0);
}

static char flash_read_page(int length) {
    for (int x = 0; x < length; x+=2) {
        putch(flash_read(LOW, here));
        putch(flash_read(HIGH, here));
        here++;
    }
    return STK_OK;
}

static char eeprom_read_page(int length) {
    for (int x = 0; x < length; x++)
        putch(spi_transaction(0xA0, 0x00, here*2+x, 0xFF));
    return STK_OK;
}

static void read_page() {
    int length = 256 * getch();
    length += getch();
    char memtype = getch();
    if (getch() == CRC_EOP) {
        putch(STK_INSYNC);
        char result = STK_FAILED;
        if (memtype == 'F') result = flash_read_page(length);
        if (memtype == 'E') result = eeprom_read_page(length);
        // flush saved-up data here, while it's allowed to take some time
        if (mode == RECORDING)
            recordFlush();
        putch(result);
    } else
        putch(STK_NOSYNC);
}

static void read_signature() {
    if (getch() == CRC_EOP) {
        param.sig0 = spi_transaction(0x30, 0, 0, 0);
        param.sig1 = spi_transaction(0x30, 0, 1, 0);
        param.sig2 = spi_transaction(0x30, 0, 2, 0);
        putch(STK_INSYNC);
        putch(param.sig0);
        putch(param.sig1);
        putch(param.sig2);
        putch(STK_OK);
    } else
        putch(STK_NOSYNC);
}

static int avrisp() { 
    char c = lastC = getch();
    if (DEBUG && mode == PLAYBACK && ' ' <= c && c <= '~')
        Serial.print(c);
    switch (c) {
        case '0': // signon
            empty_reply();
            break;
        case '1':
            if (getch() == CRC_EOP) {
                putch(STK_INSYNC);
                for (const char* p = "AVR ISP"; *p != 0; ++p)
                    putch(*p);
                putch(STK_OK);
            }
            break;
        case 'A':
            get_version(getch());
            break;
        case 'B':
            readbytes(20);
            set_parameters();
            empty_reply();
            break;
        case 'E': // extended parameters - ignore for now
            readbytes(5);
            empty_reply();
            break;
        case 'P':
            start_pmode();
            empty_reply();
            break;
        case 'U':
            here = getch();
            here += 256 * getch();
            speed = 1; // switch to fast programming once fuses have been set
            empty_reply();
            break;
        case '`': //STK_PROG_FLASH
            getch();
            getch();
            empty_reply();
            break;
        case 'a': //STK_PROG_DATA
            getch();
            empty_reply();
            break;
        case 'd': //STK_PROG_PAGE
            program_page();
            break;
        case 't': //STK_READ_PAGE
            read_page();    
            break;
        case 'V':
            universal();
            break;
        case 'Q':
            end_pmode();
            empty_reply();
            break;
        case 'u': //STK_READ_SIGN
            read_signature();
            break;
            // expecting a command, not CRC_EOP
            // this is how we can get back in sync
        case CRC_EOP:
            putch(STK_NOSYNC);
            break;
            // anything else we will return STK_UNKNOWN
        default:
            putch(getch() == CRC_EOP ? STK_UNKNOWN : STK_NOSYNC);
    }
}

static void showInfo () {
    // restore parameters from page 0
    mem.load(0, 0, &param, sizeof param);
    // report saved info for debugging
    Serial.print("ISP bytes: ");
    Serial.println(param.programsize);
    Serial.print("Code size: ");
    //FIXME: Serial.println(param.flashsize); 
    Serial.println((word) param.flashsize);
    Serial.print("Page size: ");
    Serial.println(param.pagesize);
    Serial.print("Data size: ");
    Serial.println(param.eepromsize);
    Serial.print("Signature:");
    for (byte i = 0; i < 11; ++i) {
        Serial.print(' ');
        byte b = ((const byte*) &param)[i];
        Serial.print(b >> 4, HEX);
        Serial.print(b & 0x0F, HEX);
    }
    Serial.println();
}

void setup() {
    Serial.begin(19200);

    pinMode(START_BTN, INPUT);
    digitalWrite(START_BTN, 1); // pull-up
    pinMode(LED_PMODE, OUTPUT);
    setLed(0);

#if DEBUG
    // three quick blinks after reset
    for (byte i = 1; i <= 6; ++i) {
        setLed(i & 1);
        delay(100);
    }
#endif
    
    mode = mem.isPresent() ? RECORDING : PASS_THROUGH;
    speed = 10; // start of programming slowly, speed up once data gets sent
}

void loop(void) {
    if (doneTimer.poll()) {
        recordFlush();
        param.programsize = stream.position(1); // save parameter info in page 0
        mem.save(0, 0, &param, sizeof param);

        setLed(1);
        delay(100);
        setLed(0);
        delay(100);
        setLed(1);
        delay(100);
        setLed(0);
        delay(500);

        reset();
    }
    
    if (mode == RECORDING && getButton()) {
        Serial.begin(57600); // for debugging
        Serial.println("\n[isp_capture]");
        showInfo();        
        stream.reset(); // seek to start again
        mode = PLAYBACK;
        recfill = 0;
        Serial.println("Programming...");
        start = millis() / 100;
    }
    
    if (mode == PLAYBACK || Serial.available())
        avrisp();
}
