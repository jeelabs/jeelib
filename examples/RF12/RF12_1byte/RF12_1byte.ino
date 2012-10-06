/// @dir RF12_1byte
/// Configure some values in EEPROM for easy config of the RF12 later on.
// 2009-05-06 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

// this version adds flash memory support, 2009-11-19

#include <JeeLib.h>
#include <util/crc16.h>
#include <util/parity.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>

#define DATAFLASH   1   // check for presence of DataFlash memory on JeeLink
#define FLASH_4MBIT 1   // original JL2 had 8 Mbit flash, new ones have 4 Mbit

#define LED_PIN     9   // activity LED

#define COLLECT 0x20 // collect mode, i.e. pass incoming without sending acks

static unsigned long now () {
    // FIXME 49-day overflow
    return millis() / 1000;
}

static void activityLed (byte on) {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, !on);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// RF12 configuration setup code

// @struct RF12Config
/// Definition of the data stored in EEPROM to remember the %RF12 configuration.
typedef struct {
    byte nodeId;
    byte group;
    char msg[RF12_EEPROM_SIZE-4];
    word crc;
} RF12Config;

static RF12Config config;

static char cmd;
static byte value, stack[RF12_MAXDATA], top, sendLen, dest;
static byte testbuf[RF12_MAXDATA];

static void addCh (char* msg, char c) {
    byte n = strlen(msg);
    msg[n] = c;
}

static void addInt (char* msg, word v) {
    if (v >= 10)
        addInt(msg, v / 10);
    addCh(msg, '0' + v % 10);
}

static void saveConfig () {
    // set up a nice config string to be shown on startup
    memset(config.msg, 0, sizeof config.msg);
    strcpy(config.msg, " ");
    
    byte id = config.nodeId & 0x1F;
    addCh(config.msg, '@' + id);
    strcat(config.msg, " i");
    addInt(config.msg, id);
    if (config.nodeId & COLLECT)
        addCh(config.msg, '*');
    
    strcat(config.msg, " g");
    addInt(config.msg, config.group);
    
    strcat(config.msg, " @ ");
    static word bands[4] = { 315, 433, 868, 915 };
    word band = config.nodeId >> 6;
    addInt(config.msg, bands[band]);
    strcat(config.msg, " MHz ");
    
    config.crc = ~0;
    for (byte i = 0; i < sizeof config - 2; ++i)
        config.crc = _crc16_update(config.crc, ((byte*) &config)[i]);

    // save to EEPROM
    for (byte i = 0; i < sizeof config; ++i) {
        byte b = ((byte*) &config)[i];
        eeprom_write_byte(RF12_EEPROM_ADDR + i, b);
    }
    
    if (!rf12_config())
        Serial.println("config save failed");
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// OOK transmit code

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

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// DataFlash code

#if DATAFLASH

#define DF_ENABLE_PIN   8           // PB0

#if FLASH_8MBIT
// settings for 1 Mbyte flash in JLv2
#define DF_PAGE_SIZE    256         // bytes
#define DF_BLOCK_SIZE   16          // number of pages erased at same time
#define DF_LOG_BEGIN    32          // first 2 blocks reserved for future use
#define DF_LOG_LIMIT    0x0F00      // last 64k is not used for logging
#define DF_MEM_TOTAL    0x1000      // 4096 pages, i.e. 1 Mbyte
#define DF_DEVICE_ID    0x1F45      // see AT26DF081A datasheet
#endif

#if FLASH_4MBIT
// settings for 0.5 Mbyte flash in JLv2
#define DF_PAGE_SIZE    256         // bytes
#define DF_BLOCK_SIZE   16          // number of pages erased at same time
#define DF_LOG_BEGIN    32          // first 2 blocks reserved for future use
#define DF_LOG_LIMIT    0x0700      // last 64k is not used for logging
#define DF_MEM_TOTAL    0x0800      // 2048 pages, i.e. 0.5 Mbyte
#define DF_DEVICE_ID    0x1F44      // see AT25DF041A datasheet
#endif

/// @struct FlashPage
/// Structure of each page in the log buffer, size must be exactly 256 bytes.
typedef struct {
    byte data [248];
    word seqnum;
    long timestamp;
    word crc;
} FlashPage;

/// @struct FlashEntry
/// Structure of consecutive entries in the data area of each FlashPage.
typedef struct {
    byte length;
    byte offset;
    byte header;
    byte data[RF12_MAXDATA];
} FlashEntry;

static FlashPage dfBuf;     // for data not yet written to flash
static word dfLastPage;     // page number last written
static byte dfFill;         // next byte available in buffer to store entries

static byte df_present () {
    return dfLastPage != 0;
}

static void df_enable () {
    // digitalWrite(ENABLE_PIN, 0);
    bitClear(PORTB, 0);
}

static void df_disable () {
    // digitalWrite(ENABLE_PIN, 1);
    bitSet(PORTB, 0);
}

static byte df_xfer (byte cmd) {
    SPDR = cmd;
    while (!bitRead(SPSR, SPIF))
        ;
    return SPDR;
}

void df_command (byte cmd) {
    for (;;) {
        cli();
        df_enable();
        df_xfer(0x05); // Read Status Register
        byte status = df_xfer(0);
        df_disable();
        sei();
        // don't wait for ready bit if there is clearly no dataflash connected
        if (status == 0xFF || (status & 1) == 0)
            break;
    }    

    cli();
    df_enable();
    df_xfer(cmd);
}

static void df_writeCmd (byte cmd) {
    df_command(0x06); // Write Enable
    df_deselect();
    df_command(cmd);
}

static void df_deselect () {
    df_disable();
    sei();
}

void df_read (word block, word off, void* buf, word len) {
    df_command(0x03); // Read Array (Low Frequency)
    df_xfer(block >> 8);
    df_xfer(block);
    df_xfer(off);
    for (word i = 0; i < len; ++i)
        ((byte*) buf)[(byte) i] = df_xfer(0);
    df_deselect();
}

void df_write (word block, const void* buf) {
    df_writeCmd(0x02); // Byte/Page Program
    df_xfer(block >> 8);
    df_xfer(block);
    df_xfer(0);
    for (word i = 0; i < DF_PAGE_SIZE; ++i)
        df_xfer(((const byte*) buf)[(byte) i]);
    df_deselect();
}

// wait for current command to complete
void df_flush () {
    df_read(0, 0, 0, 0);
}

static void df_wipe () {
    Serial.println("DF W");
    
    df_writeCmd(0x60);      // Chip Erase
    df_deselect();
    df_flush();
}

static void df_erase (word block) {
    Serial.print("DF E ");
    Serial.println(block);
    
    df_writeCmd(0x20);      // Block Erase (4 Kbytes)
    df_xfer(block >> 8);
    df_xfer(block);
    df_xfer(0);
    df_deselect();
    df_flush();
}

static word df_wrap (word page) {
    return page < DF_LOG_LIMIT ? page : DF_LOG_BEGIN;
}

static void df_saveBuf () {
    if (dfFill == 0)
        return;

    dfLastPage = df_wrap(dfLastPage + 1);
    if (dfLastPage == DF_LOG_BEGIN)
        ++dfBuf.seqnum; // bump to next seqnum when wrapping
    
    // set remainder of buffer data to 0xFF and calculate crc over entire buffer
    dfBuf.crc = ~0;
    for (byte i = 0; i < sizeof dfBuf - 2; ++i) {
        if (dfFill <= i && i < sizeof dfBuf.data)
            dfBuf.data[i] = 0xFF;
        dfBuf.crc = _crc16_update(dfBuf.crc, dfBuf.data[i]);
    }
    
    df_write(dfLastPage, &dfBuf);
    dfFill = 0;
    
    // wait for write to finish before reporting page, seqnum, and time stamp
    df_flush();
    Serial.print("DF S ");
    Serial.print(dfLastPage);
    Serial.print(' ');
    Serial.print(dfBuf.seqnum);
    Serial.print(' ');
    Serial.println(dfBuf.timestamp);
    
    // erase next block if we just saved data into a fresh block
    // at this point in time dfBuf is empty, so a lengthy erase cycle is ok
    if (dfLastPage % DF_BLOCK_SIZE == 0)
        df_erase(df_wrap(dfLastPage + DF_BLOCK_SIZE));
}

static void df_append (const void* buf, byte len) {
    //FIXME the current logic can't append incoming packets during a save!

    // fill in page time stamp when appending to a fresh page
    if (dfFill == 0)
        dfBuf.timestamp = now();
    
    long offset = now() - dfBuf.timestamp;
    if (offset >= 255 || dfFill + 1 + len > sizeof dfBuf.data) {
        df_saveBuf();

        dfBuf.timestamp = now();
        offset = 0;
    }

    // append new entry to flash buffer
    dfBuf.data[dfFill++] = offset;
    memcpy(dfBuf.data + dfFill, buf, len);
    dfFill += len;
}

// go through entire log buffer to figure out which page was last saved
static void scanForLastSave () {
    dfBuf.seqnum = 0;
    dfLastPage = DF_LOG_LIMIT - 1;
    // look for last page before an empty page
    for (word page = DF_LOG_BEGIN; page < DF_LOG_LIMIT; ++page) {
        word currseq;
        df_read(page, sizeof dfBuf.data, &currseq, sizeof currseq);
        if (currseq != 0xFFFF) {
            dfLastPage = page;
            dfBuf.seqnum = currseq + 1;
        } else if (dfLastPage == page - 1)
            break; // careful with empty-filled-empty case, i.e. after wrap
    }
}

static void df_initialize () {
    // assumes SPI has already been initialized for the RFM12B
    df_disable();
    pinMode(DF_ENABLE_PIN, OUTPUT);
    df_command(0x9F); // Read Manufacturer and Device ID
    word info = df_xfer(0) << 8;
    info |= df_xfer(0);
    df_deselect();

    if (info == DF_DEVICE_ID) {
        df_writeCmd(0x01);  // Write Status Register ...
        df_xfer(0);         // ... Global Unprotect
        df_deselect();

        scanForLastSave();
        
        Serial.print("DF I ");
        Serial.print(dfLastPage);
        Serial.print(' ');
        Serial.println(dfBuf.seqnum);
    
        // df_wipe();
        df_saveBuf(); //XXX
    }
}

static void discardInput () {
    while (Serial.read() >= 0)
        ;
}

static void df_dump () {
    struct { word seqnum; long timestamp; word crc; } curr;
    discardInput();
    for (word page = DF_LOG_BEGIN; page < DF_LOG_LIMIT; ++page) {
        if (Serial.read() >= 0)
            break;
        // read marker from page in flash
        df_read(page, sizeof dfBuf.data, &curr, sizeof curr);
        if (curr.seqnum == 0xFFFF)
            continue; // page never written to
        Serial.print(" df# ");
        Serial.print(page);
        Serial.print(" : ");
        Serial.print(curr.seqnum);
        Serial.print(' ');
        Serial.print(curr.timestamp);
        Serial.print(' ');
        Serial.println(curr.crc);
    }
}

static word scanForMarker (word seqnum, long asof) {
    word lastPage = 0;
    struct { word seqnum; long timestamp; } last, curr;
    last.seqnum = 0xFFFF;
    // go through all the pages in log area of flash
    for (word page = DF_LOG_BEGIN; page < DF_LOG_LIMIT; ++page) {
        // read seqnum and timestamp from page in flash
        df_read(page, sizeof dfBuf.data, &curr, sizeof curr);
        if (curr.seqnum == 0xFFFF)
            continue; // page never written to
        if (curr.seqnum >= seqnum && curr.seqnum < last.seqnum) {
            last = curr;
            lastPage = page;
        }
        if (curr.seqnum == last.seqnum && curr.timestamp <= asof)
            lastPage = page;
    }
    return lastPage;
}

static void df_replay (word seqnum, long asof) {
    word page = scanForMarker(seqnum, asof);
    Serial.print("r: page ");
    Serial.print(page);
    Serial.print(' ');
    Serial.println(dfLastPage);
    discardInput();
    word savedSeqnum = dfBuf.seqnum;
    while (page != dfLastPage) {
        if (Serial.read() >= 0)
            break;
        page = df_wrap(page + 1);
        df_read(page, 0, &dfBuf, sizeof dfBuf); // overwrites ram buffer!
        if (dfBuf.seqnum == 0xFFFF)
            continue; // page never written to
        // skip and report bad pages
        word crc = ~0;
        for (word i = 0; i < sizeof dfBuf; ++i)
            crc = _crc16_update(crc, dfBuf.data[i]);
        if (crc != 0) {
            Serial.print("DF C? ");
            Serial.print(page);
            Serial.print(' ');
            Serial.println(crc);
            continue;
        }
        // report each entry as "R seqnum time <data...>"
        byte i = 0;
        while (i < sizeof dfBuf.data && dfBuf.data[i] < 255) {
            if (Serial.available())
                break;
            Serial.print("R ");
            Serial.print(dfBuf.seqnum);
            Serial.print(' ');
            Serial.print(dfBuf.timestamp + dfBuf.data[i++]);
            Serial.print(' ');
            Serial.print((int) dfBuf.data[i++]);
            byte n = dfBuf.data[i++];
            while (n-- > 0) {
                Serial.print(' ');
                Serial.print((int) dfBuf.data[i++]);
            }
            Serial.println();
        }
        // at end of each page, report a "DF R" marker, to allow re-starting
        Serial.print("DF R ");
        Serial.print(page);
        Serial.print(' ');
        Serial.print(dfBuf.seqnum);
        Serial.print(' ');
        Serial.println(dfBuf.timestamp);
    }
    dfFill = 0; // ram buffer is no longer valid
    dfBuf.seqnum = savedSeqnum + 1; // so next replay will start at a new value
    Serial.print("DF E ");
    Serial.print(dfLastPage);
    Serial.print(' ');
    Serial.print(dfBuf.seqnum);
    Serial.print(' ');
    Serial.println(millis());
}

#else // DATAFLASH

#define df_present() 0
#define df_initialize()
#define df_dump()
#define df_replay(x,y)
#define df_erase(x)

#endif

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

char helpText1[] PROGMEM = 
    "\n"
    "Available commands:" "\n"
    "  <nn> i     - set node ID (standard node ids are 1..26)" "\n"
    "               (or enter an uppercase 'A'..'Z' to set id)" "\n"
    "  <n> b      - set MHz band (4 = 433, 8 = 868, 9 = 915)" "\n"
    "  <nnn> g    - set network group (RFM12 only allows 212)" "\n"
    "  <n> c      - set collect mode (advanced, normally 0)" "\n"
    "  t          - broadcast max-size test packet, with ack" "\n"
    "  ...,<nn> a - send data packet to node <nn>, with ack" "\n"
    "  ...,<nn> s - send data packet to node <nn>, no ack" "\n"
    "  <n> l      - turn activity LED on PB1 on or off" "\n"
    "Remote control commands:" "\n"
    "  <hchi>,<hclo>,<addr>,<cmd> f     - FS20 command (868 MHz)" "\n"
    "  <addr>,<dev>,<on> k              - KAKU command (433 MHz)" "\n"
;

char helpText2[] PROGMEM = 
    "Flash storage (JeeLink v2 only):" "\n"
    "  d                                - dump all log markers" "\n"
    "  <sh>,<sl>,<t3>,<t2>,<t1>,<t0> r  - replay from specified marker" "\n"
    "  123,<bhi>,<blo> e                - erase 4K block" "\n"
    "  12,34 w                          - wipe entire flash memory" "\n"
;

static void showString (PGM_P s) {
    for (;;) {
        char c = pgm_read_byte(s++);
        if (c == 0)
            break;
        if (c == '\n')
            Serial.print('\r');
        Serial.print(c);
    }
}

static void showHelp () {
    showString(helpText1);
    if (df_present())
        showString(helpText2);
    Serial.println("Current configuration:");
    rf12_config();
}

static void handleInput (char c) {
    if ('0' <= c && c <= '9')
        value = 10 * value + c - '0';
    else if (c == ',') {
        if (top < sizeof stack)
            stack[top++] = value;
        value = 0;
    } else if ('a' <= c && c <='z') {
        Serial.print("> ");
        Serial.print((int) value);
        Serial.println(c);
        switch (c) {
            default:
                showHelp();
                break;
            case 'i': // set node id
                config.nodeId = (config.nodeId & 0xE0) + (value & 0x1F);
                saveConfig();
                break;
            case 'b': // set band: 4 = 433, 8 = 868, 9 = 915
                value = value == 8 ? RF12_868MHZ :
                        value == 9 ? RF12_915MHZ : RF12_433MHZ;
                config.nodeId = (value << 6) + (config.nodeId & 0x3F);
                saveConfig();
                break;
            case 'g': // set network group
                config.group = value;
                saveConfig();
                break;
            case 'c': // set collect mode (off = 0, on = 1)
                if (value)
                    config.nodeId |= COLLECT;
                else
                    config.nodeId &= ~COLLECT;
                saveConfig();
                break;
            case 't': // broadcast a maximum size test packet, request an ack
                cmd = 'a';
                sendLen = RF12_MAXDATA;
                dest = 0;
                for (byte i = 0; i < RF12_MAXDATA; ++i)
                    testbuf[i] = i;
                break;
            case 'a': // send packet to node ID N, request an ack
            case 's': // send packet to node ID N, no ack
                cmd = c;
                sendLen = top;
                dest = value;
                memcpy(testbuf, stack, top);
                break;
            case 'l': // turn activity LED on or off
                activityLed(value);
                break;
            case 'f': // send FS20 command: <hchi>,<hclo>,<addr>,<cmd>f
                rf12_initialize(0, RF12_868MHZ);
                activityLed(1);
                fs20cmd(256 * stack[0] + stack[1], stack[2], value);
                activityLed(0);
                rf12_config(); // restore normal packet listening mode
                break;
            case 'k': // send KAKU command: <addr>,<dev>,<on>k
                rf12_initialize(0, RF12_433MHZ);
                activityLed(1);
                kakuSend(stack[0], stack[1], value);
                activityLed(0);
                rf12_config(); // restore normal packet listening mode
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
                if (df_present() && stack[0] == 12 && value == 34)
                    df_wipe();
                break;
        }
        value = top = 0;
        memset(stack, 0, sizeof stack);
    } else if ('A' <= c && c <= 'Z') {
        config.nodeId = (config.nodeId & 0xE0) + (c & 0x1F);
        saveConfig();
    } else if (c > ' ')
        showHelp();
}

void setup() {
    Serial.begin(57600);
    Serial.print("\n[RF12demo.5_1byte]");

    if (rf12_config()) {
        config.nodeId = eeprom_read_byte(RF12_EEPROM_ADDR);
        config.group = eeprom_read_byte(RF12_EEPROM_ADDR + 1);
    } else {
        config.nodeId = 0x41; // node A1 @ 433 MHz
        config.group = 0xD4;
        saveConfig();
    }

    df_initialize();
    
    showHelp();
}

void loop() {
    if (Serial.available())
        handleInput(Serial.read());

    if (rf12_recvDone()) {
        byte n = rf12_len;
        if (rf12_crc == 0)
{
            Serial.print("OKG ");
            extern volatile uint8_t group;
            Serial.print(group, DEC);
            Serial.print(' ');
}
        else {
return;
            Serial.print(" ? ");
            if (n > 20) // print at most 20 bytes if crc is wrong
                n = 20;
        }
        Serial.print((int) rf12_hdr);
        for (byte i = 0; i < n; ++i) {
            Serial.print(' ');
            Serial.print((int) rf12_data[i]);
        }
        Serial.println();
        
        if (rf12_crc == 0) {
            activityLed(1);
            
            if (df_present())
                df_append((const char*) rf12_data - 2, rf12_len + 2);

            if (RF12_WANTS_ACK && (config.nodeId & COLLECT) == 0) {
                Serial.println(" -> ack");
                rf12_sendStart(RF12_ACK_REPLY, 0, 0);
            }
            
            activityLed(0);
        }
    }

    if (cmd && rf12_canSend()) {
        activityLed(1);

        Serial.print(" -> ");
        Serial.print((int) sendLen);
        Serial.println(" b");
        byte header = cmd == 'a' ? RF12_HDR_ACK : 0;
        if (dest)
            header |= RF12_HDR_DST | dest;
        rf12_sendStart(header, testbuf, sendLen);
        cmd = 0;

        activityLed(0);
    }
}
