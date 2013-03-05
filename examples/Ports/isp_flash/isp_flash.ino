/// @dir isp_flash
/// Derived from http://arduino.cc/en/Tutorial/ArduinoISP
// - 2010-04-17, jcw
//
// see http://jeelabs.org/2010/04/24/isp-plug/
// and http://jeelabs.org/2011/05/29/summary-of-isp-options/

// pin definitions
#define PIN_SCK     14
#define PIN_MISO    4
#define PIN_MOSI    17
#define PIN_RESET   7
#define LED_PMODE   15  // on while programming (LED to VCC via resistor)

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

int here;           // address for reading and writing, set by 'U' command
byte data[256];     // global block storage

struct {
    byte devicecode;
    byte revision;
    byte progtype;
    byte parmode;
    byte polling;
    byte selftimed;
    byte lockbytes;
    byte fusebytes;
    int flashpoll;
    int eeprompoll;
    int pagesize;
    // not used in this code:
    // int eepromsize;
    // long flashsize;
} param;

static byte getch() {
    while (!Serial.available())
        ;
    return Serial.read();
}

static void putch(char c) {
    Serial.print(c);
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
        digitalWrite(PIN_MOSI, b & 0x80);
        digitalWrite(PIN_SCK, 0); // slow pulse, max 60KHz
        digitalWrite(PIN_SCK, 1);
        b <<= 1;
        reply = (reply << 1) | digitalRead(PIN_MISO);
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
    // call this after reading paramter packet into data[]
    memcpy(&param, data, 9);
    // following fields are big endian
    param.eeprompoll = data[10] * 0x0100 + data[11];
    param.pagesize = data[12] * 0x0100 + data[13];
    // not used in this code:
    // param.eepromsize = data[14] * 0x0100 + data[15];
    // param.flashsize = data[16] * 0x01000000L + data[17] * 0x00010000L +
    //                     data[18] * 0x00000100 + data[19];
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
    digitalWrite(LED_PMODE, 0); 
}

static void end_pmode() {
    pinMode(PIN_MISO, INPUT);
    pinMode(PIN_MOSI, INPUT);
    pinMode(PIN_SCK, INPUT);
    pinMode(PIN_RESET, INPUT);
    digitalWrite(LED_PMODE, 1); 
}

static void universal() {
    readbytes(4);
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
        putch(result);
    } else
        putch(STK_NOSYNC);
}

static void read_signature() {
    if (getch() == CRC_EOP) {
        putch(STK_INSYNC);
        putch(spi_transaction(0x30, 0x00, 0x00, 0x00));
        putch(spi_transaction(0x30, 0x00, 0x01, 0x00));
        putch(spi_transaction(0x30, 0x00, 0x02, 0x00));
        putch(STK_OK);
    } else
        putch(STK_NOSYNC);
}

static int avrisp() { 
    switch (getch()) {
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

void setup() {
    Serial.begin(9600);
    pinMode(LED_PMODE, OUTPUT);
    digitalWrite(LED_PMODE, 1);
}

void loop(void) {
    avrisp();
}
