#ifndef RF69_h
#define RF69_h

namespace RF69 {
    extern uint32_t frf;
    extern uint8_t  group;
    extern uint8_t  node;
    extern uint8_t microOffset;
    extern uint8_t  rssi;
    extern uint8_t  sendRSSI;
    extern uint8_t noiseFloorMin;
    extern uint8_t noiseFloorMax;
    extern uint8_t  rssiDelay;
    extern uint8_t lastState;
    extern uint32_t interruptMicros;
    extern uint16_t RSSIrestart;
    extern uint8_t REGIRQFLAGS1;
    extern int16_t  afc;
    extern int16_t  fei;
    extern uint8_t  lna;
    extern uint16_t interruptCount;
    extern uint16_t rxP;
    extern uint16_t txP;
    extern uint16_t discards;
    extern uint16_t unexpected;
    extern uint8_t  unexpectedFSM;
    extern uint8_t  unexpectedIRQFLAGS2;
    extern uint8_t  modeChange1;
    extern uint8_t  modeChange2;
    extern uint8_t  modeChange3;
    extern uint16_t byteCount;
    extern uint16_t underrun;
    extern uint8_t  present;
    extern uint16_t pcIntCount;
    extern uint8_t  pcIntBits;
    extern int8_t   payloadLen;
    extern uint16_t badLen;
    extern uint16_t packetShort;
    extern uint16_t nestedInterrupts;
    extern uint8_t  IRQFLAGS2;
    extern uint8_t  DIOMAPPING1;

    
    void setFrequency (uint32_t freq);
    uint8_t canSend (uint8_t clearAir);
    bool sending ();
    void sleep (bool off);
    uint8_t control(uint8_t cmd, uint8_t val);
    int8_t readTemperature(int8_t);
    uint8_t* SPI_pins();  // {OPTIMIZE_SPI, PINCHG_IRQ, RF69_COMPAT, RFM_IRQ, 
                          //  SPI_SS, SPI_MOSI, SPI_MISO, SPI_SCK }
    uint8_t currentRSSI();     
    void configure_compat ();
    uint16_t recvDone_compat (uint8_t* buf);
/// Call this to skip transmission of specific bytes in rf12_buf
/// Default value 2 skips the Jeelib header enabling non-Jeelib FSK packets 
    void skip_hdr (uint8_t skip = 2);
    void fix_len (uint8_t skip = 16);
    void afc_mode (uint8_t afc);
    void sendStart_compat (uint8_t hdr, const void* ptr, uint8_t len);
    void RSSIinterrupt();
    void interrupt_compat(uint8_t RSSI_INTERRUPT);
    void interrupt_spare();
}

#endif
