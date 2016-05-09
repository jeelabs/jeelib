#include <stdint.h>
#include <RF69.h>
#include <RF69_avr.h>
#define __PROG_TYPES_COMPAT__
#include <avr/pgmspace.h>

#define ROM_UINT8       const uint8_t // Does this change storage to RAM?
#define ROM_READ_UINT8  pgm_read_byte
#define ROM_DATA        PROGMEM

#define LIBRARY_VERSION     13      // Stored in REG_SYNCVALUE6 by initRadio 

#define REG_FIFO            0x00
#define REG_OPMODE          0x01
#define DATAMODUL           0x02 
#define REG_BITRATEMSB      0x03
#define REG_BITRATELSB      0x04
#define REG_FRFMSB          0x07
#define REG_OSC1            0x0A
#define REG_OCP             0x13
#define REG_LNA             0x18
#define REG_AFCFEI          0x1E
#define REG_AFCMSB          0x1F
#define REG_AFCLSB          0x20
#define REG_FEIMSB          0x21
#define REG_FEILSB          0x22
#define REG_RSSICONFIG      0x23
#define REG_RSSIVALUE       0x24
#define REG_DIOMAPPING1     0x25
#define REG_IRQFLAGS1       0x27
#define REG_IRQFLAGS2       0x28
#define REG_RSSITHRESHOLD   0x29
#define REG_SYNCCONFIG      0x2E
#define REG_SYNCVALUE1      0x2F
#define REG_SYNCVALUE2      0x30
#define REG_SYNCVALUE3      0x31
#define REG_SYNCVALUE4      0x32
#define REG_SYNCVALUE5      0x33
#define REG_SYNCVALUE6      0x34
#define REG_SYNCVALUE7      0x35
#define REG_SYNCVALUE8      0x36
#define REG_SYNCGROUP       0x33
#define REG_NODEADRS        0x39
#define REG_FIFOTHRESH      0x3C
#define REG_PACKETCONFIG2   0x3D
#define REG_AESKEY1         0x3E
#define REG_TEMP1           0x4E
#define REG_TEMP2           0x4F
#define REG_TESTLNA         0x58
#define REG_TESTPA1         0x5A
#define REG_TESTPA2         0x5C

#define MODE_SLEEP          0x00
#define MODE_STANDBY        0x04
#define MODE_FS             0x08    // Unpredictable results with this value
#define MODE_RECEIVER       0x10
#define MODE_LISTENABORT    0x20
#define MODE_LISTENON       0x40
#define MODE_TRANSMITTER    0x0C
#define MODE_MASK           0x1C
#define TESTLNA_NORMAL      0x1B
#define TESTLNA_BOOST       0x2D
#define TESTPA1_NORMAL      0x55
#define TESTPA1_20dBm       0x5D
#define TESTPA2_NORMAL      0x70
#define TESTPA2_20dBm       0x7C

#define COURSE_TEMP_COEF      -89 // starter callibration figure
#define RF_TEMP1_MEAS_START   0x08
#define RF_TEMP1_MEAS_RUNNING 0x04

#define IRQ1_MODEREADY      0x80
#define IRQ1_RXREADY        0x40
#define IRQ1_RSSI           0x08
#define IRQ1_TIMEOUT        0x04

#define START_TX            0xA0  // With 125Khz SPI a minimum
#define DELAY_TX            0x20  // 22 byte head start required, 32 to be safe 


#define IRQ2_FIFOFULL       0x80
#define IRQ2_FIFONOTEMPTY   0x40
#define IRQ2_FIFOOVERRUN    0x10
#define IRQ2_PACKETSENT     0x08
#define IRQ2_PAYLOADREADY   0x04
#define DIO0_PACKETSENT     0x00
#define DIO0_PAYLOADREADY   0x40
#define DIO0_SYNCADDRESS    0x80

#define DIO3_FIFOFULL       0x00
#define DIO3_RSSI           0x01
#define DIO3_FIFOFULL_TX    0x00
#define DIO3_NOTHING_IN_TX  0x02

#define RcCalStart          0x81
#define RcCalDone           0x40
#define FeiStart            0x20
#define FeiDone             0x40
#define RssiStart           0x01
#define RssiDone            0x02
#define oneByteSync         0x80
#define twoByteSync         0x88
#define threeByteSync       0x90
#define fourByteSync        0x98
#define fiveByteSync        0xA0

#define AFC_CLEAR           0x02

#define RF_MAX   72

// transceiver states, these determine what to do with each interrupt
enum { TXCRC1, TXCRC2,/* TXTAIL,*/ TXDONE, TXIDLE, TXRECV };

namespace RF69 {
    uint32_t frf;
    uint8_t  group;
    uint8_t  node;
    uint16_t crc;
    uint8_t  rssi;
    uint8_t  startRSSI;
    uint8_t  sendRSSI;
    uint8_t  rssiDelay;
    uint32_t rssiActive;
    uint32_t rssiSilent;
    uint16_t rssiChanged;
    uint8_t  lastState;
    uint32_t interruptMicros;
    uint8_t  interruptRSSI;
    uint8_t  interruptLNA;
    uint16_t countRSSI;
    uint16_t RSSIrestart;
    uint8_t  REGIRQFLAGS1;
    int16_t  afc;                  // I wonder how to make sure these 
    int16_t  fei;                  // are volatile
    uint8_t  lna;
    uint16_t interruptCount;
    uint16_t rxP;
    uint16_t txP;
    uint16_t discards;
    uint16_t unexpected;
    uint8_t  unexpectedFSM;
    uint8_t  unexpectedIRQFLAGS2;
    uint8_t  modeChange1;
    uint8_t  modeChange2;
    uint8_t  modeChange3;
    uint16_t byteCount;
    uint16_t underrun;
    uint8_t  present;
    uint16_t pcIntCount;
    uint8_t  pcIntBits;
    int8_t   payloadLen;
    uint16_t badLen;
    uint16_t packetShort;
    uint16_t nestedInterrupts;
    uint8_t  IRQFLAGS2;
    uint8_t  DIOMAPPING1;
    }

static volatile uint8_t rxfill;      // number of data bytes in rf12_buf
static volatile uint8_t rxdone;      // 
static volatile int8_t rxstate;      // current transceiver state
static volatile uint8_t packetBytes; // Count of bytes in packet
static volatile uint16_t discards;   // Count of packets discarded
static volatile uint8_t reentry = false;
static volatile uint8_t rf69_skip;   // header bytes to skip
static volatile uint8_t rf69_fix;    // Maximum for fixed length packet
//static volatile uint16_t interval;
static volatile uint32_t RSSIinterruptMicros;
static volatile uint16_t rtp;
static volatile uint16_t rst;

static ROM_UINT8 configRegs_compat [] ROM_DATA = {
  0x2E, 0xA0, // SyncConfig = sync on, sync size = 5
  0x2F, 0xAA, // SyncValue1 = 0xAA
  0x30, 0xAA, // SyncValue2 = 0xAA
  0x31, 0xAA, // SyncValue3 = 0xAA
  0x32, 0x2D, // SyncValue4 = 0x2D
  0x33, 0xD4, // SyncValue5 = 212, Group
// 0x01, 0x04, // OpMode = standby
//  0x02, 0x03, // DataModul = packet mode, fsk, Gaussian filter, BT = 0.3
  0x03, 0x02, // BitRateMsb, data rate = 49,261 khz
  0x04, 0x8A, // BitRateLsb, divider = 32 MHz / 650 == 49,230 khz
//  0x05, 0x00, // FdevMsb = 9.943 KHz
//  0x06, 0xA3, // FdevLsb = 9.943 KHz
//  0x05, 0x01, // FdevMsb = 23 KHz
//  0x06, 0x79, // FdevLsb = 23 KHz
//  0x05, 0x02, // FdevMsb = 45 KHz
//  0x06, 0xE1, // FdevLsb = 45 KHz
//  0x05, 0x05, // FdevMsb = 81 KHz
//  0x06, 0x2F, // FdevLsb = 81 KHz
  0x05, 0x05, // FdevMsb = 90 KHz
  0x06, 0xC3, // FdevLsb = 90 KHz
  // 0x07, 0xD9, // FrfMsb, freq = 868.000 MHz
  // 0x08, 0x00, // FrfMib, divider = 14221312
  // 0x09, 0x00, // FrfLsb, step = 61.03515625
//  0x0B, 0x20, // AfcCtrl, afclowbetaon
/*
// Mismatching PA1 below with the RFM69x module present risks blowing a hole in the LNA
// 0x11, 0x5F, // PA1 enable, Pout = max // uncomment this for RFM69H
*/
//  0x18, 0x02, // Manual LNA = 2 = -6dB
  0x19, 0x42, // RxBw 125 KHz
  0x1A, 0x42, // AfcBw 125 KHz Channel filter BW

// AFC is broken on the RFM69. However leaving factory default
// does adjust the receiver on initial packets.
//#define AfcClear            0x02
#define AfcClear              0x11
//  0x1E, 0x00,   // 
//  0x1E, 0x0C, // AFC each time RX mode entered USELESS!
// Radio frequency wanders off into the wilderness/ 
//  0x25, 0x80, // DioMapping1 = RSSI threshold
  0x26, 0x07, // disable clkout

//  0x29, 0xA0, // RssiThresh ... -80dB
  0x29, 0xFF, // RssiThresh ... -127.5dB
  0x2B, 0x05,
//  0x2D, 0x01, // Only 1 preamble byte
  0x2E, 0xA7, // SyncConfig = sync on, sync size = 5
  0x2F, 0xAA, // SyncValue1 = 0xAA
  0x30, 0xAA, // SyncValue2 = 0xAA
  0x31, 0xAA, // SyncValue3 = 0xAA
  0x32, 0x2D, // SyncValue4 = 0x2D
  0x33, 0xD4, // SyncValue5 = 212, Group
  0x37, 0x00, // PacketConfig1 = fixed, no crc, filt off
  0x38, 0x00, // PayloadLength = 0, unlimited
  0x3C, 0x8F, // FifoTresh, not empty, level 15 bytes
  0x3D, 0x10, // PacketConfig2, interpkt = 1, autorxrestart off
  0x58, 0x2D, // High sensitivity mode
//  0x6F, 0x20, // 0x30, // TestDagc ...
//  0x71, 0x01, // AFC offset set for low modulation index systems, used if
              // AfcLowBetaOn=1. Offset = LowBetaAfcOffset x 488 Hz 
  0
};  
/*

0x13 RegOcp default is Current limiter active, threshold at 45+ 5*trim bits.
( i.e 45 +5*10 = 95mA).
The intent here is to run PA1 only, to keep out of the turbo boost range for the
moment. The spec sheet doesn't have a specific current draw table for PA1 alone,
but you can infer that PA1 is probably an identical TX stage to PA0, so using 
that data suggests a maximum TX current draw of ~45mA.  
So the default cap of 95mA leaves plenty of head room.
The alternative would be just to disable the feature - it is only needed in the 
"nearly flat battery" case.


*/
uint8_t RF69::control(uint8_t cmd, uint8_t val) {
    PreventInterrupt appropriate_IRQ;
    return spiTransfer(cmd, val);
}

static void writeReg (uint8_t addr, uint8_t value) {
    RF69::control(addr | 0x80, value);
}

static uint8_t readReg (uint8_t addr) {
    return RF69::control(addr, 0);
}

static void flushFifo () {
    while (readReg(REG_IRQFLAGS2) & (IRQ2_FIFONOTEMPTY | IRQ2_FIFOOVERRUN))
        readReg(REG_FIFO);
}

uint8_t setMode (uint8_t mode) {
    byte c = 0;
    writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | mode);
    while ((readReg(REG_IRQFLAGS1) & IRQ1_MODEREADY) == 0) {
        c++; if (c == 255) break;
    }
    return c;
}

static uint8_t initRadio (ROM_UINT8* init) {
    InitIntPin();

    spiInit();
// Validate SPI bus operation
    writeReg(REG_SYNCVALUE6, LIBRARY_VERSION);
    writeReg(REG_SYNCVALUE7, 0xAA);
    writeReg(REG_SYNCVALUE8, 0x55);
    if ((readReg(REG_SYNCVALUE7) == 0xAA) && (readReg(REG_SYNCVALUE8) == 0x55)) {
// Configure radio
        for (;;) {
            uint8_t cmd = ROM_READ_UINT8(init);
            if (cmd == 0) break;
            writeReg(cmd, ROM_READ_UINT8(init+1));
            init += 2;
        }
        return 1;
    }
    return 0;
}

void RF69::setFrequency (uint32_t freq) {
    // Frequency steps are in units of (32,000,000 >> 19) = 61.03515625 Hz
    // use multiples of 64 to avoid multi-precision arithmetic, i.e. 3906.25 Hz
    // due to this, the lower 6 bits of the calculated factor will always be 0
    // this is still 4 ppm, i.e. well below the radio's 32 MHz crystal accuracy
    // 868.0 MHz = 0xD90000, 868.3 MHz = 0xD91300, 915.0 MHz = 0xE4C000 
    frf = (((freq << 2) / (32000000L >> 11)) << 6) + 31;  // middle of 'o' band
    rf69_skip = 0;    // Ensure default Jeenode RF12 operation
}

uint8_t RF69::canSend (uint8_t clearAir) {
  if (/*rxstate == TXRECV && */((rxfill == 0) || (rxdone))) {
  // Need to better understand the FSM since buffer is filled without interrupts
    /*
        uint8_t storedMode = (readReg(REG_OPMODE) & MODE_MASK);
        // avoid delay in changing modes unless required
        if(storedMode != MODE_RECEIVER) setMode(MODE_RECEIVER);
        if(readReg(REG_IRQFLAGS1) & IRQ1_RSSI) { // Is there traffic around?
            if (storedMode != MODE_RECEIVER) setMode(storedMode);
            return false;
        }
    */ 
        sendRSSI = currentRSSI();
        if(sendRSSI >= clearAir) {
            rxstate = TXIDLE;
            setMode(MODE_STANDBY);
            return sendRSSI;
        }
    }
    return false;
}

bool RF69::sending () {
    return (rxstate < TXIDLE);
}

//  Note: RF12_WAKEUP returns with receiver mode disabled!
void RF69::sleep (bool off) {
    setMode(off ? MODE_SLEEP : MODE_STANDBY);
    rxstate = TXIDLE;
}

// returns raw temperature from chip
int8_t RF69::readTemperature(int8_t userCal) {
  sleep(false);        // this ensures the mode is in standby, using setMode directly had undesirable side effects.
  writeReg(REG_TEMP1, RF_TEMP1_MEAS_START);
  while ((readReg(REG_TEMP1) & RF_TEMP1_MEAS_RUNNING));
  return ~readReg(REG_TEMP2) + COURSE_TEMP_COEF + userCal; //'complement' corrects the slope, rising temp = rising val
}

uint8_t* RF69::SPI_pins() {
  return (SPI_Pins());  // {OPTIMIZE_SPI, PINCHG_IRQ, RF69_COMPAT, RFM_IRQ, 
                        //  SPI_SS, SPI_MOSI, SPI_MISO, SPI_SCK }
}

uint8_t RF69::currentRSSI() {
  if (/*rxstate == TXRECV && */((rxfill == 0) || (rxdone))) {

      uint8_t storedMode = (readReg(REG_OPMODE) & MODE_MASK);
      uint8_t storeDIOM = readReg(REG_DIOMAPPING1); // Collect Interrupt trigger
      uint8_t noiseFloor = readReg(REG_RSSITHRESHOLD);// Store current threshold

      setMode(MODE_STANDBY); 
      writeReg(REG_DIOMAPPING1, (DIO0_PAYLOADREADY | DIO3_FIFOFULL));// Suppress Interrupts
      writeReg(REG_RSSITHRESHOLD, 0xFF);              // Open up threshold

      setMode(MODE_RECEIVER);   // Looses contents of FIFO and 36 spins
      rssiDelay = 0;
      writeReg(REG_RSSICONFIG, RssiStart);
      while (!(readReg(REG_IRQFLAGS1) & IRQ1_RSSI)) rssiDelay++;
      uint8_t r = readReg(REG_RSSIVALUE);           // Collect RSSI value
      
      setMode(MODE_STANDBY);                        // Get out of RX mode 
      writeReg(REG_RSSITHRESHOLD, noiseFloor);      // Restore threshold
      if (storedMode != MODE_RECEIVER) setMode(storedMode); // Restore mode
      else setMode(MODE_RECEIVER);                  // Restart RX mode
      // The above is required to clear RSSI threshold mechanism

      // REG_DIOMAPPING1 is mode sensitive so can only restore to correct mode
      writeReg(REG_DIOMAPPING1, storeDIOM);         // Restore Interrupt trigger
      return r;
      
  } else return 0;
}

// References to the RF12 driver above this line will generate compiler errors!
#include <RF69_compat.h>
#include <RF12.h>

void RF69::configure_compat () {
    present = 0;                                    // Assume radio is absent
    if (initRadio(configRegs_compat)) {
        writeReg(REG_SYNCGROUP, group);
        if (group == 0) {
            writeReg(REG_SYNCCONFIG, fourByteSync);
        } else {
            writeReg(REG_SYNCCONFIG, fiveByteSync);
        }   

        writeReg(REG_FRFMSB, frf >> 16);
        writeReg(REG_FRFMSB+1, frf >> 8);
        writeReg(REG_FRFMSB+2, frf);
        setMode(MODE_STANDBY);
        writeReg(REG_OSC1, RcCalStart);             // Calibrate
        while(!(readReg(REG_OSC1) & RcCalDone));    // Wait for completion
        writeReg(REG_IRQFLAGS2, IRQ2_FIFOOVERRUN);  // Clear FIFO
        rxstate = TXIDLE;
        present = 1;                                // Radio is present
    }
}

uint8_t* recvBuf;

uint16_t RF69::recvDone_compat (uint8_t* buf) {
    switch (rxstate) {
    case TXIDLE:
        rxdone = false;
        rxfill = rf69_buf[2] = 0;
        crc = _crc16_update(~0, group);
        recvBuf = buf;
        rxstate = TXRECV;
        flushFifo();
        startRSSI = currentRSSI();
        RSSIinterruptMicros = 0;
        
        modeChange1 = setMode(MODE_RECEIVER);// setting RX mode uses 33-36 spins
        writeReg(REG_DIOMAPPING1, (DIO0_SYNCADDRESS | DIO3_RSSI));// Interrupt trigger
        break;
    case TXRECV:
        if (rxdone) {
            rxstate = TXIDLE;
            
            // Move attributes & packet into rf12_buf
            rf12_rssi = rssi;
            rf12_lna = lna;
            rf12_afc = afc;
            rf12_fei = fei;
            rf12_rtp = rtp; // Delay between RSSI & Data Packet
            rf12_rst = rst; // Count of resets used to capture packet
            for (byte i = 0; i <= (rf69_len + 5); i++) {
                rf12_buf[i] = rf69_buf[i];
            }     
            rf12_crc = crc;

            if (crc == 0) {
                if (!(rf69_hdr & RF12_HDR_DST) || node == 31 ||
                    (rf69_hdr & RF12_HDR_MASK) == node) {
                    return 0; // it's for us, good packet received
                } else {
                    discards++;
                    // Packet wasn't for us so we want too discard it silently
                    // This will happen on next entry to recvDone_compat
                    // because rxstate == TXIDLE
                }
            } else return 1;
        } else if (readReg(REG_IRQFLAGS1) & IRQ1_TIMEOUT) {
//            if ((micros() - RSSIinterruptMicros) > 2000ul) {
//                REGIRQFLAGS1 = IRQ1_TIMEOUT;// Timeout?                
                RSSIrestart++;
                setMode(MODE_STANDBY);
                rxstate = TXIDLE;   // Looses contents of FIFO and 36 spins
                // Noise interrupt, abort RX cycle and restart
//            }
        }
        break;
    }
    return ~0; // keep going, not done yet
}

void RF69::skip_hdr (uint8_t skip) {
    rf69_skip = skip;
}

void RF69::fix_len (uint8_t fix) {
    rf69_fix = fix;
}

uint16_t rf69_status () {
    return (rxstate << 8) | rxfill;   
}

// Uses rf12_buf as the send buffer, rf69_buf reserved for RX
void RF69::sendStart_compat (uint8_t hdr, const void* ptr, uint8_t len) {

    rf12_len = len;
    for (int i = 0; i < len; ++i)
        rf12_data[i] = ((const uint8_t*) ptr)[i];
    rf12_hdr = hdr & RF12_HDR_DST ? hdr : (hdr & ~RF12_HDR_MASK) + node; 
// TODO instruction below worries me now I have 4/5 byte sync
    rxstate = - (2 + rf12_len); // preamble and SYN1/SYN2 are sent by hardware
    flushFifo();
    
/*  All packets are transmitted with a 5 byte header SYN1/SYN2/SYN3/2D/Group  
    even when the group is zero                                               */
    
    // REG_SYNCGROUP must have been set to an appropriate group before this.
    writeReg(REG_SYNCCONFIG, fiveByteSync);
    crc = _crc16_update(~0, readReg(REG_SYNCGROUP));
    
    modeChange2 = setMode(MODE_TRANSMITTER);
    writeReg(REG_DIOMAPPING1, (DIO0_PACKETSENT | DIO3_NOTHING_IN_TX));     // Interrupt trigger
    
/*  We must being transmission to avoid overflowing the FIFO since
    jeelib packet size can exceed FIFO size. We also want to avoid the
    transmissions of sync etc before payload is presented.                    */
    
/* Page 54
The transmission of packet data is initiated by the Packet Handler only if the 
module is in Tx mode and the transmission condition defined by TxStartCondition 
is fulfilled. If transmission condition is not fulfilled then the packet handler
transmits a preamble sequence until the condition is met. This happens only if 
the preamble length /= 0, otherwise it transmits a zero or one until the 
condition is met to transmit the packet data.
*/    
    
//    if (rf12_len > 9)                       // Expedite short packet TX
//      writeReg(REG_FIFOTHRESH, DELAY_TX);   // Wait for FIFO to hit 32 bytes
                                            // transmission will then begin
// the above code was to permit slow SPI bus speeds - may not be possible.  
                                              
    // use busy polling until the last byte fits into the buffer
    // this makes sure it all happens on time, and that sendWait can sleep
    //
    // TODO This is not interrupt code, how can sendWait sleep, control isn't
    // passed back. JohnO
    
    while (rxstate < TXDONE)
        if ((readReg(REG_IRQFLAGS2) & IRQ2_FIFOFULL) == 0) { // FIFO is 66 bytes
//            uint8_t out = 0xAA; // To be used at end of packet
            uint8_t out;
            if (rxstate < 0) {
                // rf12_buf used since rf69_buf reserved for RX
                out = rf12_buf[3 + rf12_len + rf69_skip + rxstate];
                crc = _crc16_update(crc, out);
            } else {
                switch (rxstate) {
                    case TXCRC1: out = crc; break;
                    case TXCRC2: out = crc >> 8;
                    rf12_crc = crc; 
                    break;
                }
            }
            writeReg(REG_FIFO, out);
            ++rxstate;
        }
//        writeReg(REG_FIFOTHRESH, START_TX);     // if < 32 bytes, release FIFO
                                                  // for transmission
/*  At this point packet is typically in the FIFO but not fully transmitted.
    transmission complete will be indicated by an interrupt.                   
*/

}
void RF69::RSSIinterrupt() {
        RSSIinterruptMicros = micros();
//        interruptRSSI = readReg(REG_RSSIVALUE);
//        interruptLNA  = readReg(REG_LNA);
//        countRSSI++;
}

void RF69::interrupt_compat () {
        interruptMicros = micros() - RSSIinterruptMicros;
        RSSIinterruptMicros = 0;
        interruptCount++;
        // Interrupt will ONLY remain asserted until FIFO empty or exit RX mode
        // FIFO can pass through empty during reception since it is also draining 
        if (rxstate == TXRECV) {
            // The following line attempts to stop further interrupts
            writeReg(REG_DIOMAPPING1, (DIO0_PAYLOADREADY | DIO3_FIFOFULL));
                                                        // Turns off RSSI DIO3
            if (reentry) {
                nestedInterrupts++;
                uint8_t f = readReg(REG_IRQFLAGS2);
                if(f != 64) IRQFLAGS2 = f;
//                if(f == 0) IRQFLAGS2 = 0xFF;
                DIOMAPPING1 = readReg(REG_DIOMAPPING1);
                return;
            }   
            reentry = true;
            IRQ_ENABLE;       // allow nested interrupts from here on
            while (!readReg(REG_IRQFLAGS1) & IRQ1_RXREADY)
                ;
            rssi = readReg(REG_RSSIVALUE);
            lna = readReg(REG_LNA);
            afc  = readReg(REG_AFCMSB);
            afc  = (afc << 8) | readReg(REG_AFCLSB);
            writeReg(REG_AFCFEI, FeiStart);
            while (!readReg(REG_AFCFEI) & FeiDone)
                ;
            fei  = readReg(REG_FEIMSB);
            fei  = (fei << 8) | readReg(REG_FEILSB);
            
            writeReg(REG_AFCFEI, AFC_CLEAR);  // Clear the AFC
            
            // The window for grabbing the above values is quite small
            // values available during transfer between the ether
            // and the inbound fifo buffer.
            rtp = interruptMicros;
            rst = RSSIrestart;
            volatile uint8_t stillCollecting = true;
            rxP++;
            crc = ~0;
            packetBytes = 0;
            payloadLen = rf69_fix; // Assumed value if no Jee header used            

            for (;;) { // busy loop, to get each data byte as soon as it comes in 
                if (readReg(REG_IRQFLAGS2) & 
                   (IRQ2_FIFONOTEMPTY /*| IRQ2_FIFOOVERRUN*/)) {
                    if (rxfill == 0 && group != 0) { 
                      recvBuf[rxfill++] = group;
                      packetBytes++;
                      crc = _crc16_update(crc, group);
                    } 
                    volatile uint8_t in = readReg(REG_FIFO);
                    
                    if ((rxfill == 2) && (rf69_skip == 0)) {
                        if (in <= RF12_MAXDATA) {  // capture and
                            payloadLen = in;       // validate length byte
                        } else {
                            recvBuf[rxfill++] = 0; // Set rf69_len to zero!
                            payloadLen = -2;       // skip CRC in payload
                            in = ~0;               // fake CRC 
                            recvBuf[rxfill++] = in;// into buffer
                            packetBytes+=2;        // don't trip underflow
                            crc = 1;               // set bad CRC
                            badLen++;
                        }
                    }
                    
                    recvBuf[rxfill++] = in;
                    packetBytes++;
                    crc = _crc16_update(crc, in);              
                    if (rxfill >= (payloadLen + (5 - rf69_skip))) {  // Trap end of payload
                        setMode(MODE_STANDBY);  // Get radio out of RX mode
                        stillCollecting = false;
                        break;
                    }
            } 

        }
        byteCount = rxfill;
        if (packetBytes < (5 - rf69_skip)) underrun++;
            
        if (stillCollecting) {
            // We are exiting before a successful packet completion
            packetShort++;
        }    
        setMode(MODE_STANDBY);
        rxdone = true;      // force TXRECV in RF69::recvDone_compat
    } else if (readReg(REG_IRQFLAGS2) & IRQ2_PACKETSENT) {
          writeReg(REG_TESTPA1, TESTPA1_NORMAL);    // Turn off high power 
          writeReg(REG_TESTPA2, TESTPA2_NORMAL);    // transmit
          // rxstate will be TXDONE at this point
          IRQ_ENABLE;       // allow nested interrupts from here on
          txP++;
          setMode(MODE_STANDBY);
          rxstate = TXIDLE;
          // Restore sync bytes configuration
          if (group == 0) {               // Allow receiving from all groups
              writeReg(REG_SYNCCONFIG, fourByteSync);
          }
    } else {
          // We get here when a interrupt that is neither for RX or as a TX
          // completion. Appears related to receiving noise when the bad CRC
          // packet display is enabled using "0q".
          unexpected++;
          unexpectedFSM = rxstate; // Save Finite State Machine status
                uint8_t f = readReg(REG_IRQFLAGS2);
                if(f) unexpectedIRQFLAGS2 = f;
                else unexpectedIRQFLAGS2 = 0xFF;
    }
    reentry = false;
}
