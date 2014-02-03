#ifndef RF69_h
#define RF69_h

namespace RF69 {
    extern uint32_t freq;
    extern uint8_t  group;
    extern uint8_t  node;

    void configure ();
    uint16_t recvDone (uint8_t* buf);
    void sendStart (uint8_t hdr, const void* ptr, uint8_t len);
};

#endif
