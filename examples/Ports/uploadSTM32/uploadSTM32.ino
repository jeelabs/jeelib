#include "ParitySerial.h"

#define RX_PIN      4   // Arduino Digital.4, ATmega portD.4, ATMega328 pin.6
#define TX_PIN      14  // Arduino Analog.0, ATmega portC.0, ATMega328 pin.23

#define BOOT_LOADER "boot-usbSerial-v01.h"

const uint8_t data[] PROGMEM = {
#include BOOT_LOADER
};

ParitySerial Target (RX_PIN, TX_PIN); // defaults to even parity
uint8_t check;

enum {
    ACK = 0x79,
    NAK = 0x1F,
};

enum {
    GET_CMD = 0x00,
    GETID_CMD = 0x02,
    WRITE_CMD = 0x31,
    ERASE_CMD = 0x43,
    EXTERA_CMD = 0x44,
    RDUNP_CMD = 0x92,
};

static uint8_t getData (uint16_t index) {
    return pgm_read_byte(data + index);
}

static uint8_t getReply () {
    for (long i = 0; i < 200000; ++i)
        if (Target.available())
            return Target.read();
    return 0;
}

static void wantAck () {
    uint8_t b = 0;
    for (int i = 0; i < 5; ++i) {
        b = getReply();
        if (b == 0) {
            Serial.print('.');
            continue;
        }
        if (b != ACK)
            break;
        check = 0;
        return;
    }
    Serial.print(" FAILED - got 0x");
    Serial.println(b, HEX);
    while (true)
        ; // halt
}

static void sendByte (uint8_t b) {
    check ^= b;
    Target.write(b);
}

static void sendCmd (uint8_t cmd) {
    //delay(100);
    Serial.flush();
    //Target.flush();
    sendByte(cmd);
    sendByte(~cmd);
    wantAck();
}

static void connectToTarget () {
    Target.begin(9600);

    uint8_t b = 0;
    do {
        Serial.print(".");
        Target.write(0x7F);
        b = getReply();
        //Serial.print(b, HEX);
    } while (b != ACK && b != NAK);
}

static uint8_t getBootVersion () {
    sendCmd(GET_CMD);
    uint8_t n = getReply();
    uint8_t bootRev = getReply();
    for (int i = 0; i < n; ++i)
        getReply();
    wantAck();
    return bootRev;
}

static uint16_t getChipType () {
    sendCmd(GETID_CMD);
    getReply(); // should be 1
    uint16_t chipType = getReply() << 8;
    chipType |= getReply();
    wantAck();
    return chipType;
}

static void massErase () {
#if 1
    sendCmd(ERASE_CMD);
    sendByte((uint8_t) 0xFF);
    sendByte((uint8_t) 0x00);
    wantAck();
#else
    sendCmd(EXTERA_CMD);
    sendByte((uint8_t) 0xFF);
    sendByte((uint8_t) 0xFF);
    sendByte((uint8_t) 0xFF);
    wantAck();
#endif
}

void setup () {
    Serial.begin(115200);
    Serial.print("[uploadSTM32] ");
    Serial.println(BOOT_LOADER);
    Serial.println();
    Serial.println("(When you see a question mark: RESET your TARGET board!)");
    Serial.println();

    Serial.print("  Connecting? ");
    connectToTarget();
    Serial.println(" OK");

    uint8_t bootRev = getBootVersion();
    Serial.print("Boot version: 0x");
    Serial.println(bootRev, HEX);

    uint16_t chipType = getChipType();
    Serial.print("   Chip type: 0x");
    Serial.println(chipType, HEX);

    Serial.print("Unprotecting: ");
    sendCmd(RDUNP_CMD);
    wantAck();
    Serial.println("OK");

    Serial.print("    Resuming? ");
    connectToTarget();
    Serial.println(" OK");

    Serial.print("     Erasing: ");
    massErase();
    Serial.println("OK");
    
    Serial.print("     Writing: ");
    for (uint16_t offset = 0; offset < sizeof data; offset += 256) {
        Serial.print('.');
        sendCmd(WRITE_CMD);
        uint32_t addr = 0x08000000 + offset;
        sendByte(addr >> 24);
        sendByte(addr >> 16);
        sendByte(addr >> 8);
        sendByte(addr);
        sendByte(check);
        wantAck();
        sendByte(256-1);
        //check = 0;
        for (int i = 0; i < 256; ++i)
            sendByte(getData(i));
        sendByte(check);
        wantAck();
    }
    Serial.println(" OK");

    Serial.print("        Done: ");
    Serial.print(sizeof data);
    Serial.println(" bytes uploaded.");
}

void loop () {}
