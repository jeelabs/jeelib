#ifndef RF69_h
#define RF69_h

namespace RF69 {
    extern uint32_t frf;
    extern uint8_t  group;
    extern uint8_t  node;

    void setFrequency (uint32_t freq);
    bool canSend ();
    
    void configure_compat ();
    uint16_t recvDone_compat (uint8_t* buf);
    void sendStart_compat (uint8_t hdr, const void* ptr, uint8_t len);
};

#endif
