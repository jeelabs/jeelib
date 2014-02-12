// extracted from RF12demo
// 2009-05-06 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#define DF_ENABLE_PIN 8         // PB0

#if DATAFLASH == 4
// settings for 0.5 Mbyte flash in JLv2
#define DF_BLOCK_SIZE 16        // number of pages erased at same time
#define DF_LOG_BEGIN  32        // first 2 blocks reserved for future use
#define DF_LOG_LIMIT  0x0700    // last 64k is not used for logging
#define DF_MEM_TOTAL  0x0800    // 2048 pages, i.e. 0.5 Mbyte
#define DF_DEVICE_ID  0x1F44    // see AT25DF041A datasheet
#define DF_PAGE_ERASE 0x20      // erase one block of flash memory
#endif

#if DATAFLASH == 8
// settings for 1 Mbyte flash in JLv2
#define DF_BLOCK_SIZE 16        // number of pages erased at same time
#define DF_LOG_BEGIN  32        // first 2 blocks reserved for future use
#define DF_LOG_LIMIT  0x0F00    // last 64k is not used for logging
#define DF_MEM_TOTAL  0x1000    // 4096 pages, i.e. 1 Mbyte
#define DF_DEVICE_ID  0x1F45    // see AT26DF081A datasheet
#define DF_PAGE_ERASE 0x20      // erase one block of flash memory
#endif

#if DATAFLASH == 16
// settings for 2 Mbyte flash in JLv3
#define DF_BLOCK_SIZE 256       // number of pages erased at same time
#define DF_LOG_BEGIN  512       // first 2 blocks reserved for future use
#define DF_LOG_LIMIT  0x1F00    // last 64k is not used for logging
#define DF_MEM_TOTAL  0x2000    // 8192 pages, i.e. 2 Mbyte
#define DF_DEVICE_ID  0x2020    // see M25P16 datasheet
#define DF_PAGE_ERASE 0xD8      // erase one block of flash memory
#endif

// structure of each page in the log buffer, size must be exactly 256 bytes
typedef struct {
    byte data [248];
    word seqnum;
    long timestamp;
    word crc;
} FlashPage;

// structure of consecutive entries in the data area of each FlashPage
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

static void df_deselect () {
    df_disable();
    sei();
}

static void df_writeCmd (byte cmd) {
    df_command(0x06); // Write Enable
    df_deselect();
    df_command(cmd);
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
    for (word i = 0; i < 256; ++i)
        df_xfer(((const byte*) buf)[(byte) i]);
    df_deselect();
}

// wait for current command to complete
void df_flush () {
    df_read(0, 0, 0, 0);
}

static void df_wipe () {
    showString(PSTR("DF W\n"));
    
    df_writeCmd(0xC7); // Chip Erase
    df_deselect();
    df_flush();
}

static void df_erase (word block) {
    showString(PSTR("DF E "));
    Serial.println(block);
    
    df_writeCmd(DF_PAGE_ERASE); // Block Erase
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
    showString(PSTR("DF S "));
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
        
        showString(PSTR("DF I "));
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
        showString(PSTR(" df# "));
        Serial.print(page);
        showString(PSTR(" : "));
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
    showString(PSTR("r: page "));
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
            showString(PSTR("DF C? "));
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
            showString(PSTR("R "));
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
        showString(PSTR("DF R "));
        Serial.print(page);
        Serial.print(' ');
        Serial.print(dfBuf.seqnum);
        Serial.print(' ');
        Serial.println(dfBuf.timestamp);
    }
    dfFill = 0; // ram buffer is no longer valid
    dfBuf.seqnum = savedSeqnum + 1; // so next replay will start at a new value
    showString(PSTR("DF E "));
    Serial.print(dfLastPage);
    Serial.print(' ');
    Serial.print(dfBuf.seqnum);
    Serial.print(' ');
    Serial.println(millis());
}
