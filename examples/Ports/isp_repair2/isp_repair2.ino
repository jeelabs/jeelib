/// @dir isp_repair2
/// Reflash a boot loader and a sketch an a second ATmega.
// 2010-05-29 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php
//
// see http://jeelabs.org/2011/05/29/summary-of-isp-options/

// This code is adapted from isp_prepare. It omits the button and run LED,
// and starts right away. To use, open a serial console and wait until done.
//
// The 6 ISP pins of the target board need to be connected to the board running
// this sketch as follows (using Arduino pin naming):
//
//   ISP pin 1  <->  digital 4  (MISO)          ISP CONNECTOR
//   ISP pin 2  <->  VCC                          +---+---+
//   ISP pin 3  <->  analog 0   (SCK)             | 1 | 2 |
//   ISP pin 4  <->  analog 3   (MOSI)            | 3 | 4 |
//   ISP pin 5  <->  digital 7  (RESET)           | 5 | 6 |
//   ISP pin 6  <->  ground                       +---+---+
//
// The same hookup, using JeeNode port/pin names:
//
//   ISP pin 1  <->  DIO1
//   ISP pin 2  <->  +3V
//   ISP pin 3  <->  AIO1
//   ISP pin 4  <->  AIO4
//   ISP pin 5  <->  DIO4
//   ISP pin 6  <->  GND
//
// Boot Cloner
// adapted from http://www.arduino.cc/playground/BootCloner/BootCloner
// original copyright notice: 2007 by Amplificar <mailto:amplificar@gmail.com>

#define DISABLE_RF12  1   // comment out to remove dependency on JeeLib
#define FAST_SPI      1   // comment out to revert to digitalWrite() calls

#if DISABLE_RF12
#include <JeeLib.h>
#endif
#include <avr/pgmspace.h>
#include <avr/sleep.h>

// pin definitions
#define PIN_SCK     14  // PC0 - AIO1 - serial clock to target avr
#define PIN_MISO    4   // PD4 - DIO1 - input from target avr
#define PIN_MOSI    17  // PC3 - AIO4 - output to target avr
#define RESET       7   // PD7 - DIO4 - reset pin of the target avr
#define DONE_LED    9   // B1 - blue LED on JN USB, blinks on start and when ok

// pins used for the optional config switches
#define CONFIG1     5   // DIO2
#define CONFIG2     15  // AIO2
#define CONFIG3     16  // AIO3
#define CONFIG4     6   // DIO3

// MPU-specific values
#define PAGE_BYTES      128  // ATmega168 and ATmega328
#define LOCK_BITS       0xCF
#define FUSE_LOW_XTAL   0xFF
#define FUSE_LOW_FAST   0xDE
#define FUSE_HIGH_512   0xDE
#define FUSE_HIGH_1024  0xDC
#define FUSE_HIGH_2048  0xDA
#define FUSE_HIGH_4096  0xD8 // not in ATmega168
#define FUSE_EXTENDED   0xFD

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

/* Include code using a file generated in isp_prepare/ dir with this cmd:

        ./hex2c.tcl Blink.cpp.hex \
                    RF12demo.cpp.hex \
                    optiboot_atmega328.hex \
                    ATmegaBOOT_168_atmega328.hex \
                    optiboot_atmega328_1s.hex \
                    optiboot_atmega328.hex >../isp_repair2/data.h

    Code choices are fixed: section 0 is RF12demo, section 1 is blink
    Boot choices are entries 2..5 in the sections[] array in data.h
*/

#include "data.h"

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// ISP Command Words
#define CMD_Program_Enable      0xAC53
#define CMD_Erase_Flash         0xAC80
#define CMD_Poll                0xF000
#define CMD_Read_Flash_Low      0x2000
#define CMD_Read_Flash_High     0x2800
#define CMD_Load_Page_Low       0x4000
#define CMD_Load_Page_High      0x4800
#define CMD_Write_Page          0x4C00
#define CMD_Read_EEPROM         0xA000
#define CMD_Write_EEPROM        0xC000
#define CMD_Read_Lock           0x5800
#define CMD_Write_Lock          0xACE0
#define CMD_Read_Signature      0x3000
#define CMD_Write_Fuse_Low      0xACA0
#define CMD_Write_Fuse_High     0xACA8
#define CMD_Write_Fuse_Extended 0xACA4
#define CMD_Read_Fuse_Low       0x5000
#define CMD_Read_Fuse_High      0x5808
#define CMD_Read_Fuse_Extended  0x5008
#define CMD_Read_Fuse_High      0x5808
#define CMD_Read_Calibration    0x3800

static bool fastSPI = false; // don't start in fast mode right away

// transfer a byte using software SPI, using a faster mode when possible
static byte XferByte(byte v) {
    byte result = 0;
    if (fastSPI)
        for (byte i = 0; i < 8; ++i) {
            bitWrite(PORTC, 3, v & 0x80);
            v <<= 1;
            bitClear(PORTC, 0);
            result <<= 1;
            bitSet(PORTC, 0);
            result |= bitRead(PIND, 4);
        }
    else
        for (byte i = 0; i < 8; ++i) {
            digitalWrite(PIN_MOSI, v & 0x80);
            digitalWrite(PIN_SCK, 0); // slow pulse, max 60KHz
            digitalWrite(PIN_SCK, 1);
            v <<= 1;
            result = (result << 1) | digitalRead(PIN_MISO);
        }
    return result;
}

// send 4 bytes to target microcontroller, returns the fourth MISO byte
static byte Send_ISP (word v01, byte v2 =0, byte v3 =0) {
    XferByte(v01 >> 8);
    XferByte(v01);
    XferByte(v2);
    return XferByte(v3);
}

// send 4 bytes to target microcontroller and wait for completion
static void Send_ISP_wait (word v01, byte v2 =0, byte v3 =0) {
    Send_ISP(v01, v2, v3);
    while (Send_ISP(CMD_Poll) & 1)
        ;
}

// reset the target microcontroller
static void Reset_Target() {
    digitalWrite(RESET, 1);
    digitalWrite(PIN_SCK, 0); // has to be set LOW at startup, or PE fails
    delay(30);
    digitalWrite(RESET, 0);
    delay(30); // minimum delay here is 20ms for the ATmega8
}

// print the 16 signature bytes (device codes)
static void Read_Signature() {
    Serial.print("Signatures:");
    for (byte x = 0 ; x < 8 ; ++x) {
        Serial.print(" ");
        Serial.print(Send_ISP(CMD_Read_Signature, x), HEX);
    }
    Serial.println("");
}

// prints the lock and fuse bits (no leading zeros)
static byte Read_Fuses(byte flo, byte fhi) {
    Serial.print("Lock Bits: ");
    Serial.println(Send_ISP(CMD_Read_Lock), HEX);
    Serial.print("Fuses: low ");
    Serial.print(Send_ISP(CMD_Read_Fuse_Low), HEX);
    Serial.print(", high ");
    Serial.print(Send_ISP(CMD_Read_Fuse_High), HEX);
    Serial.print(", extended ");
    Serial.println(Send_ISP(CMD_Read_Fuse_Extended), HEX);
    return Send_ISP(CMD_Read_Lock) == LOCK_BITS &&
           Send_ISP(CMD_Read_Fuse_Low) == flo &&
           Send_ISP(CMD_Read_Fuse_High) == fhi &&
           Send_ISP(CMD_Read_Fuse_Extended) == FUSE_EXTENDED;
}

static word addr2page (word addr) {
    return (word)(addr & ~ (PAGE_BYTES-1)) >> 1;
}

static void LoadPage(word addr, const byte* ptr) {
    word cmd = addr & 1 ? CMD_Load_Page_High : CMD_Load_Page_Low;
    Send_ISP(cmd | (addr >> 9), addr >> 1, pgm_read_byte(ptr));
}

static void WritePage (word page) {
    Send_ISP_wait(CMD_Write_Page | (page >> 8), page);
}

static void WriteData (word start, const byte* data, word count) {
    word page = addr2page(start);
    for (word i = 0; i < count; i += 2) {
        if (page != addr2page(start)) {
            WritePage(page);
            Serial.print('.');
            page = addr2page(start);
        }
        LoadPage(start++, data + i);
        LoadPage(start++, data + i + 1);
    }
    WritePage(page);
    Serial.println();
}

static byte EnableProgramming () {
    Reset_Target();
    if (Send_ISP(CMD_Program_Enable, 0x22, 0x22) != 0x22) {
        Serial.println("Program Enable FAILED");
        return 0;
    }
    return 1;
}

static void blink () {
  pinMode(DONE_LED, OUTPUT);
  digitalWrite(DONE_LED, 0); // inverted logic
  delay(100); // blink briefly
  pinMode(DONE_LED, INPUT);
}

static byte readConfig () {
    static byte pins[] = { CONFIG1, CONFIG2, CONFIG3, CONFIG4 };
    byte switches = 0;
    for (byte i = 0; i < 4; ++i) {
        pinMode(pins[i], INPUT);
        digitalWrite(pins[i], 1); // enable pull-up
        bitWrite(switches, i, digitalRead(pins[i]));
        digitalWrite(pins[i], 0); // disable pull-up
    }
    return switches; // a 4-bit value, i.e. 0..15
}

static byte programSection (byte index) {
    Serial.print(index, DEC);
    byte f = EnableProgramming();
    if (f) {
        fastSPI = FAST_SPI && PIN_SCK == 14 && PIN_MISO == 4 && PIN_MOSI == 17;
        WriteData(sections[index].start, progdata + sections[index].off,
                      sections[index].count);
        fastSPI = false;
    }
    return f;
}

void setup () {
    Serial.begin(57600);
    Serial.println("\n[isp_repair2.2]");
    blink();

    digitalWrite(PIN_SCK, 1);
    digitalWrite(PIN_MOSI, 1);
    digitalWrite(RESET, 1);
  
    pinMode(PIN_SCK, OUTPUT);
    pinMode(PIN_MOSI, OUTPUT);
    pinMode(RESET, OUTPUT);
  
    byte config = readConfig();
    byte sketch = config & 1;           // 0..1 : CONFIG1
    byte xspeed = (config >> 1) & 1;    // 0..1 : CONFIG2
    byte bootld = 2 + (config >> 2);    // 2..5 : CONFIG4 & CONFIG3
    
    Serial.print("Configuration: ");
    Serial.print(config, HEX);
    Serial.println(xspeed ? " (resonator)" : " (crystal)");
    Serial.println();
    Serial.println(sections[sketch].title);
    Serial.println(sections[bootld].title);
    Serial.println();
    
    if (EnableProgramming()) {
        Serial.println("Erasing Flash");
        Send_ISP_wait(CMD_Erase_Flash, 0x22, 0x22);

        if (EnableProgramming()) {
            byte fuseLo = xspeed ? FUSE_LOW_FAST : FUSE_LOW_XTAL;
            // derive the boot size from its starting address
            byte fuseHi = FUSE_HIGH_2048;
            switch (sections[bootld].start & 0x0FFF) {
                case 0x0E00: fuseHi = FUSE_HIGH_512; break;
                case 0x0C00: fuseHi = FUSE_HIGH_1024; break;
                case 0x0800: fuseHi = FUSE_HIGH_2048; break;
                case 0x0000: fuseHi = FUSE_HIGH_4096; break;
            }

            // set the fuses and lock bits
            Serial.println("Setting Fuses");
            Send_ISP_wait(CMD_Write_Fuse_Low, 0, fuseLo);
            Send_ISP_wait(CMD_Write_Fuse_High, 0, fuseHi);
            Send_ISP_wait(CMD_Write_Fuse_Extended, 0, FUSE_EXTENDED);
            Send_ISP_wait(CMD_Write_Lock, 0, LOCK_BITS);

            // burn the sketch and bootstrap code
            if (programSection(sketch) && programSection(bootld)) {
                Read_Signature();
                if (Read_Fuses(fuseLo, fuseHi)) {
                    Serial.println("\nDone.");
                    blink();    
                } else
                    Serial.println("Fuses NOT OK!");
            }
        }
    }

    pinMode(PIN_SCK, INPUT);
    pinMode(PIN_MOSI, INPUT);
    pinMode(RESET, INPUT);
    
    digitalWrite(PIN_SCK, 0);
    digitalWrite(PIN_MOSI, 0);
    digitalWrite(RESET, 0);

#if ARDUINO >= 100
    Serial.flush();
#endif
    delay(10); // let the serial port finish
#if DISABLE_RF12
    // disable the radio for deep power power down on a JeeNode
    rf12_initialize(1, RF12_868MHZ);
    rf12_sleep(RF12_SLEEP);
#endif
    cli(); // stop responding to interrupts
    ADCSRA &= ~ bit(ADEN); // disable the ADC
    PRR = 0xFF; // disable all subsystems
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_mode();
    // total power down, can only wake up with a hardware reset
}

void loop () {}
