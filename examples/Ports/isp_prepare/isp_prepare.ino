/// @dir isp_prepare
/// Upload a built-in range of data bytes to a target board attached via ISP.
// 2010-04-18 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php
//
// see http://jeelabs.org/2010/04/25/preparing-atmegas-with-isp/

// see http://jeelabs.org/tag/isp/ for related weblog posts
// and http://jeelabs.org/2011/05/29/summary-of-isp-options/

// Boot Cloner
// adapted from http://www.arduino.cc/playground/BootCloner/BootCloner
// original copyright notice: 2007 by Amplificar <mailto:amplificar@gmail.com>

#include <avr/pgmspace.h>

// select the proper configuration with these three settings:

#define USE_OPTI_BOOT   1   // boot loader: OptiBoot = 1, Duemilanove = 0
#define USE_RF12DEMO    1   // sketch: RF12demo = 1, Blink = 0
#define USE_FAST_WAKEUP 1   // clock: resonator = 1, crystal (std) = 0

// MPU-specific values, properly adjusted for ATmega168 and ATmega328

#define PAGE_BYTES      128
#define LOCK_BITS       0x3F
#define FUSE_EXTENDED   0x05

#if USE_FAST_WAKEUP
  #define FUSE_LOW      0xDE  // start in 258/14 CK + 65 ms
#else                   
  #define FUSE_LOW      0xFF  // start in 16K/14 CK + 65 ms
#endif                  

#if USE_OPTI_BOOT       
  #define FUSE_HIGH     0xDE  // 512 bytes for boot loader
#else                   
  #define FUSE_HIGH     0xDA  // 2048 bytes for boot loader
#endif

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Selects appropriate data file, depending on USE_OPTI_BOOT and USE_RF12DEMO

#if USE_OPTI_BOOT
  #if USE_RF12DEMO
    #include "opti_rf12demo.h"
  #else
    #include "opti_blink.h"
  #endif
#else
  #if USE_RF12DEMO
    #include "data_rf12demo.h"
  #else
    #include "data_blink.h"
  #endif
#endif

// pin definitions
#define SCK         14   // AIO1 - serial clock to target avr
#define MISO        4    // DIO1 - input from target avr
#define MOSI        17   // AIO4 - output to target avr
#define RESET       7    // DIO4 - reset pin of the target avr
// on Flash Board
#define START_BTN   5    // DIO2 - active low - starts programming target
#define RUN_LED     15   // AIO2
#define ERR_LED     9    // B1 - blue LED

// ISP Command Words
#define CMD_Program_Enable	    0xAC53
#define CMD_Erase_Flash	        0xAC80
#define CMD_Poll                0xF000
#define CMD_Read_Flash_Low	    0x2000
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

static byte XferByte(byte v) {
    byte result = 0;
    for (byte i = 0; i < 8; ++i) {
        digitalWrite(MOSI, v & 0x80);
        digitalWrite(SCK, 0); // slow pulse, max 60KHz
        digitalWrite(SCK, 1);
        v <<= 1;
        result = (result << 1) | digitalRead(MISO);
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

static void Send_ISP_wait (word v01, byte v2 =0, byte v3 =0) {
    Send_ISP(v01, v2, v3);
    while (Send_ISP(CMD_Poll) & 1)
        ;
}

static void Reset_Target() {
    digitalWrite(RESET, 1);
    digitalWrite(SCK, 0); // has to be set LOW at startup, or PE fails
    delay(30);
    digitalWrite(RESET, 0);
    delay(30); // minimum delay here is 20ms for the ATmega8
}

// prints the 16 signature bytes (device codes)
static void Read_Signature() {
    Serial.print("Signatures:");
    for (byte x = 0 ; x < 8 ; ++x) {
        Serial.print(" ");
        Serial.print(Send_ISP(CMD_Read_Signature, x), HEX);
    }
    Serial.println("");
}

// prints the lock and fuse bits (no leading zeros)
static void Read_Fuses() {
    Serial.print("Lock Bits: ");
    Serial.println(Send_ISP(CMD_Read_Lock), HEX);
    Serial.print("Fuses: low ");
    Serial.print(Send_ISP(CMD_Read_Fuse_Low), HEX);
    Serial.print(", high ");
    Serial.print(Send_ISP(CMD_Read_Fuse_High), HEX);
    Serial.print(", extended ");
    Serial.println(Send_ISP(CMD_Read_Fuse_Extended), HEX);
    // if (Send_ISP(CMD_Read_Lock) == LOCK_BITS &&
    //         Send_ISP(CMD_Read_Fuse_Low) == FUSE_LOW &&
    //             Send_ISP(CMD_Read_Fuse_High) == FUSE_HIGH
    //                 Send_ISP(CMD_Read_Fuse_Extended) == FUSE_EXTENDED)
    //     Serial.println("Fuse bits OK.");
}

static void SetLed (char c) {
    digitalWrite(ERR_LED, c != '?');
    digitalWrite(RUN_LED, c != '>');
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
        Serial.print("Program Enable ");
        return 0;
    }
    return 1;
}

void setup () {
    Serial.begin(57600);
    Serial.println("\n[isp_prepare]");

    Serial.println("\nProgrammer data:");
    for (byte i = 0; i < sizeof sections / sizeof sections[0]; ++i) {
        Serial.print("  ");
        Serial.println(sections[i].title);
    }

    pinMode(START_BTN, INPUT);
    digitalWrite(START_BTN, 1); // pull-up
    pinMode(RUN_LED, OUTPUT);
    pinMode(ERR_LED, OUTPUT);
}

void loop () {  
    pinMode(SCK, INPUT);
    pinMode(MISO, INPUT);
    pinMode(MOSI, INPUT);
    pinMode(RESET, INPUT);
  
    Serial.println("\nType 'G' to start the ISP programmer:");

    SetLed(' ');
    while (digitalRead(START_BTN) && Serial.read() != 'G')
        ;
    SetLed('>');
    
    digitalWrite(SCK, 1);
    digitalWrite(MOSI, 1);
    digitalWrite(RESET, 1);
  
    pinMode(SCK, OUTPUT);
    pinMode(MOSI, OUTPUT);
    pinMode(RESET, OUTPUT);
  
    Serial.println("\nStarting...");
    
    delay(100);
    while (!digitalRead(START_BTN))
        ;
    delay(100);
    
    if (EnableProgramming()) {
        Serial.println("Erasing Flash");
        Send_ISP_wait(CMD_Erase_Flash, 0x22, 0x22);

        if (EnableProgramming()) {
            Serial.println("Setting Fuses");
            Send_ISP_wait(CMD_Write_Fuse_Low, 0, FUSE_LOW);
            Send_ISP_wait(CMD_Write_Fuse_High, 0, FUSE_HIGH);
            Send_ISP_wait(CMD_Write_Fuse_Extended, 0, FUSE_EXTENDED);
            Send_ISP_wait(CMD_Write_Lock, 0, LOCK_BITS);
    
            byte ok = 1;
            for (byte i = 0; i < sizeof sections / sizeof sections[0]; ++i) {
                Serial.println(sections[i].title);
                if (EnableProgramming())
                    WriteData(sections[i].start,
                                progdata + sections[i].off,
                                  sections[i].count);
                else
                    ok = 0;
            }
            if (ok) {
                Read_Fuses();
                Read_Signature();
                Serial.println("Done.");    
                return;
            }
        }
    }
    
    Serial.println("ERROR");    
    while (digitalRead(START_BTN)) {
        SetLed('>');
        delay(100);
        SetLed('?');
        delay(500);
    }
}
