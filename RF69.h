#ifndef RF69_h
#define RF69_h

namespace RF69 {
    extern uint32_t frf;
    extern uint8_t  group;
    extern uint8_t  node;
    extern uint8_t  rssi;
    extern int16_t  afc;
    extern int16_t  fei;
    extern uint16_t interruptCount;
    extern uint16_t rxP;
    extern uint16_t txP;
    extern uint16_t overrun;

    void setFrequency (uint32_t freq);
    bool canSend ();
    bool sending ();
    void sleep (bool off);
    uint8_t control(uint8_t cmd, uint8_t val);
    
    void configure_compat ();
    uint16_t recvDone_compat (uint8_t* buf);
    void sendStart_compat (uint8_t hdr, const void* ptr, uint8_t len);
    void interrupt_compat();
};

#endif
