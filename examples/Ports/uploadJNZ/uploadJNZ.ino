// This turns a JeeNode (or other 3.3V Arduino clone) into a programmer for the
// JeeNode Zero, loading a copy of Mecrisp Forth on it via the ROM boot loader.

#include "ParitySerial.h"

#define RX_PIN      7   // P4D: digital pin on JeePort #4
#define TX_PIN      17  // P4A: analog  pin on JeePort #4

#define RESET_PIN   4   // P1D: digital pin on JeePort #1
#define BOOT0_PIN   14  // P1A: analog  pin on JeePort #1

#define GREEN_PIN   6   // P3D: digital pin on JeePort #3
#define ERROR_PIN   16  // P3A: analog  pin on JeePort #3
#define START_PIN   3   // IRQ: interrupt pin on all JeePorts

#define BOOT_LOADER "l052-mecrisp.h"

#define RESET() ((void (*)()) 0)()

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
    WRUNP_CMD = 0x73,
    RDUNP_CMD = 0x92,
};

static uint8_t getData (uint16_t index) {
    return pgm_read_byte(data + index);
}

static uint8_t getReply () {
    for (long i = 0; i < 50000; ++i)
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
    digitalWrite(ERROR_PIN, 1);

    while (digitalRead(START_PIN) == 0)
        ;
    RESET();
}

static void sendByte (uint8_t b) {
    check ^= b;
    Target.write(b);
}

static void send2Bytes (uint16_t b) {
    sendByte(b >> 8);
    sendByte(b);
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
    Target.begin(57600);

    uint8_t b = 0;
    do {
        digitalWrite(GREEN_PIN, 1);
        Serial.print(".");
        delay(20);
        Target.write(0x7F);
        digitalWrite(GREEN_PIN, 0);
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
#if 0
    sendCmd(ERASE_CMD);
    sendByte((uint8_t) 0xFF);
    sendByte((uint8_t) 0x00);
    wantAck();
#else
    // for some reason, a "full" mass erase is rejected with a NAK
    // ... so erase a list of segments instead, 1 more than needed
    // this will only erase the pages to be programmed!
    sendCmd(EXTERA_CMD);
    uint16_t pages = (sizeof data + 127) / 128;
    send2Bytes(pages - 1);
    for (int i = 0; i < pages; ++i)
        send2Bytes(i);
    sendByte(check);
    wantAck();
#endif
}

static void waitForStart () {
    do {
        delay(500);
        digitalWrite(GREEN_PIN, 1);
        delay(500);
        digitalWrite(GREEN_PIN, 0);
    } while (digitalRead(START_PIN));  // zero starts, inverted logic
}

static void resetTarget (bool enterBoot) {
    digitalWrite(RESET_PIN, 1); pinMode(RESET_PIN, OUTPUT);
    digitalWrite(BOOT0_PIN, 0); pinMode(BOOT0_PIN, OUTPUT);
    delay(2);
    digitalWrite(BOOT0_PIN, enterBoot ? 1 : 0);
    delay(2);
    digitalWrite(RESET_PIN, 0);
    delay(10);
    digitalWrite(RESET_PIN, 1);
}

void setup () {
    Serial.begin(115200);
    Serial.print("[uploadJNZ] ");
    Serial.print(BOOT_LOADER);
    Serial.print(' ');
    Serial.println(sizeof data);
    Serial.println();

    digitalWrite(ERROR_PIN, 0); pinMode(ERROR_PIN, OUTPUT);
    digitalWrite(GREEN_PIN, 0); pinMode(GREEN_PIN, OUTPUT);
    pinMode(START_PIN, INPUT_PULLUP);

    waitForStart();

    resetTarget(true);

    Serial.print("  Connecting: ");
    connectToTarget();
    Serial.println(" OK");

    uint8_t bootRev = getBootVersion();
    Serial.print("Boot version: 0x");
    Serial.println(bootRev, HEX);

    uint16_t chipType = getChipType();
    Serial.print("   Chip type: 0x");
    Serial.println(chipType, HEX);

    Serial.print(" Read unprot: ");
    sendCmd(RDUNP_CMD);
    wantAck();
    Serial.println("OK");

    Serial.print("    Resuming: ");
    connectToTarget();
    Serial.println(" OK");

    Serial.print("Write unprot: ");
    sendCmd(WRUNP_CMD);
    wantAck();
    Serial.println("OK");

    Serial.print("    Resuming: ");
    connectToTarget();
    Serial.println(" OK");

    Serial.print("     Erasing: ");
    massErase();
    Serial.println("OK");
    
    Serial.print("     Writing: ");
    for (uint16_t offset = 0; offset < sizeof data; offset += 256) {
        digitalWrite(ERROR_PIN, 1);
        Serial.print('.');
        sendCmd(WRITE_CMD);
        uint32_t addr = 0x08000000 + offset;
        sendByte(addr >> 24);
        sendByte(addr >> 16);
        sendByte(addr >> 8);
        sendByte(addr);
        sendByte(check);
        digitalWrite(ERROR_PIN, 0);
        wantAck();
        sendByte(256-1);
        //check = 0;
        for (int i = 0; i < 256; ++i)
            sendByte(getData(offset + i));
        sendByte(check);
        wantAck();
    }
    Serial.println(" OK");

    Serial.print("        Done: ");
    Serial.print(sizeof data);
    Serial.println(" bytes uploaded.");
    digitalWrite(GREEN_PIN, 1);

    Target.begin(115200, false, false); // disable parity
    resetTarget(false);
}

void loop () {
    if (Serial.available())
        Target.write(Serial.read());
    if (Target.available())
        Serial.write(Target.read());
    if (digitalRead(START_PIN))  // inverted logicgg
        RESET();
}
