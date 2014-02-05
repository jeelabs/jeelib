#include <stdint.h>
#include <RF69.h>
#include <RF69_avr.h>

#define REG_FIFO            0x00
#define REG_OPMODE          0x01
#define REG_FRFMSB          0x07
#define REG_RSSIVALUE       0x24
#define REG_DIOMAPPING1     0x25
#define REG_IRQFLAGS1       0x27
#define REG_IRQFLAGS2       0x28
#define REG_SYNCVALUE1      0x2F
#define REG_SYNCVALUE2      0x30
#define REG_NODEADRS        0x39
#define REG_PACKETCONFIG2   0x3D
#define REG_AESKEY1         0x3E

#define MODE_SLEEP          0x00
#define MODE_STANDBY        0x04
#define MODE_RECEIVER       0x10
#define MODE_TRANSMITTER    0x0C

#define IRQ1_MODEREADY      0x80
#define IRQ1_RXREADY        0x40

#define IRQ2_FIFOFULL       0x80
#define IRQ2_FIFONOTEMPTY   0x40
#define IRQ2_FIFOOVERRUN    0x10
#define IRQ2_PACKETSENT     0x08
#define IRQ2_PAYLOADREADY   0x04

#define RF_MAX   72

// transceiver states, these determine what to do with each interrupt
enum { TXCRC1, TXCRC2, TXTAIL, TXDONE, TXIDLE, TXRECV };

namespace RF69 {
    uint32_t frf;
    uint8_t  group;
    uint8_t  node;
    uint16_t crc;
    uint8_t  rssi;
}

static volatile uint8_t rxfill;     // number of data bytes in rf12_buf
static volatile int8_t rxstate;     // current transceiver state

static ROM_UINT8 configRegs_compat [] ROM_DATA = {
  0x01, 0x04, // OpMode = standby
  0x02, 0x00, // DataModul = packet mode, fsk
  0x03, 0x02, // BitRateMsb, data rate = 49,261 khz
  0x04, 0x8A, // BitRateLsb, divider = 32 MHz / 650
  0x05, 0x05, // FdevMsb = 90 KHz
  0x06, 0xC3, // FdevLsb = 90 KHz
  // 0x07, 0xD9, // FrfMsb, freq = 868.000 MHz
  // 0x08, 0x00, // FrfMib, divider = 14221312
  // 0x09, 0x00, // FrfLsb, step = 61.03515625
  0x0B, 0x20, // AfcCtrl, afclowbetaon
  0x19, 0x42, // RxBw ...
  0x25, 0x80, // DioMapping1 ...
  // 0x29, 0xDC, // RssiThresh ...
  0x2E, 0x88, // SyncConfig = sync on, sync size = 2
  0x2F, 0x2D, // SyncValue1 = 0x2D
  // 0x30, 0x05, // SyncValue2 = 0x05
  0x37, 0x00, // PacketConfig1 = fixed, no crc, filt off
  0x38, 0x00, // PayloadLength = 0, unlimited
  0x3C, 0x8F, // FifoTresh, not empty, level 15
  0x3D, 0x10, // PacketConfig2, interpkt = 1, autorxrestart off
  0x6F, 0x20, // TestDagc ...
  0
};

static void writeReg (uint8_t addr, uint8_t value) {
    PreventInterrupt irq0;
    spiTransfer(addr | 0x80, value);
}

static uint8_t readReg (uint8_t addr) {
    PreventInterrupt irq0;
    return spiTransfer(addr, 0);
}

static void flushFifo () {
    while (readReg(REG_IRQFLAGS2) & (IRQ2_FIFONOTEMPTY | IRQ2_FIFOOVERRUN))
        readReg(REG_FIFO);
}

static void setMode (uint8_t mode) {
    writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | mode);
    while ((readReg(REG_IRQFLAGS1) & IRQ1_MODEREADY) == 0)
        ;
}

static void initRadio (ROM_UINT8* init) {
    spiInit();
    do
        writeReg(REG_SYNCVALUE1, 0xAA);
    while (readReg(REG_SYNCVALUE1) != 0xAA);
    do
        writeReg(REG_SYNCVALUE1, 0x55);
    while (readReg(REG_SYNCVALUE1) != 0x55);
    for (;;) {
        uint8_t cmd = ROM_READ_UINT8(init);
        if (cmd == 0) break;
        writeReg(cmd, ROM_READ_UINT8(init+1));
        init += 2;
    }
}

void RF69::setFrequency (uint32_t freq) {
    // Frequency steps are in units of (32,000,000 >> 19) = 61.03515625 Hz
    // use multiples of 64 to avoid multi-precision arithmetic, i.e. 3906.25 Hz
    // due to this, the lower 6 bits of the calculated factor will always be 0
    // this is still 4 ppm, i.e. well below the radio's 32 MHz crystal accuracy
    // 868.0 MHz = 0xD90000, 868.3 MHz = 0xD91300, 915.0 MHz = 0xE4C000  
    frf = ((freq << 2) / (32000000L >> 11)) << 6;
}

bool RF69::canSend () {
    if (rxstate == TXRECV && rxfill == 0) {
        rxstate = TXIDLE;
        setMode(MODE_STANDBY);
        return true;
    }
    return false;
}

// References to the RF12 driver above this line will generate compiler errors!
#include <RF69_compat.h>
#include <RF12.h>

void RF69::configure_compat () {
    initRadio(configRegs_compat);    
    writeReg(REG_SYNCVALUE2, group);

    writeReg(REG_FRFMSB, frf >> 16);
    writeReg(REG_FRFMSB+1, frf >> 8);
    writeReg(REG_FRFMSB+2, frf);

    rxstate = TXIDLE;
}

uint16_t RF69::recvDone_compat (uint8_t* buf) {
    if (rxstate == TXIDLE) {
        rxfill = rf12_len = 0;
        crc = _crc16_update(~0, group);
        rxstate = TXRECV;
        flushFifo();
        setMode(MODE_RECEIVER);
    } else {
        uint8_t irq2 = readReg(REG_IRQFLAGS2);
        if (rxstate == TXRECV) {
            if (irq2 & (IRQ2_FIFONOTEMPTY | IRQ2_FIFOOVERRUN)) {
                uint8_t in = readReg(REG_FIFO);
                if (rxfill == 0)
                    buf[rxfill++] = group;
                buf[rxfill++] = in;
                crc = _crc16_update(crc, in);
              
                if (rxfill >= rf12_len + 5 || rxfill >= RF_MAX) {
                    rxstate = TXIDLE;
                    setMode(MODE_STANDBY);
                    if (rf12_len > RF12_MAXDATA)
                        crc = 1; // force bad for invalid packet
                    if (!(rf12_hdr & RF12_HDR_DST) || node == 31 ||
                            (rf12_hdr & RF12_HDR_MASK) == node)
                        // it's a broadcast or addressed to this node
                        return crc;
                }
            }
        } else if ((irq2 & IRQ2_FIFOFULL) == 0) {
            uint8_t out;
            if (rxstate < 0) {
                uint8_t pos = 3 + rf12_len + rxstate++;
                out = buf[pos];
                crc = _crc16_update(crc, out);
            } else if (rxstate == TXDONE) {
                if ((irq2 & IRQ2_PACKETSENT)) {
                    rxstate = TXIDLE;
                    setMode(MODE_STANDBY);
                }
                return ~0; // keep transmitting until the packet has been sent
            } else
                switch (rxstate++) {
                    case TXCRC1: out = crc; break;
                    case TXCRC2: out = crc >> 8; break;
                    default:     out = 0xAA;
                }
                writeReg(REG_FIFO, out);
        }
    }
    return ~0;
}

void RF69::sendStart_compat (uint8_t hdr, const void* ptr, uint8_t len) {
    rf12_len = len;
    for (int i = 0; i < len; ++i)
        rf12_data[i] = ((const uint8_t*) ptr)[i];
    rf12_hdr = hdr & RF12_HDR_DST ? hdr : (hdr & ~RF12_HDR_MASK) + node;  
    rf12_crc = _crc16_update(~0, group);
    rxstate = - (2 + rf12_len); // preamble and SYN1/SYN2 are sent by hardware
    flushFifo();
    setMode(MODE_TRANSMITTER);
}

void RF69::interrupt_compat () {
    rssi = readReg(REG_RSSIVALUE);
}
