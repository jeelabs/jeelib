#include <JeeLib.h>

namespace RF69 {
    uint32_t freq;
    uint8_t  group;
    uint8_t  node;
    uint16_t crc;
}

void RF69::configure() {
}

uint16_t RF69::recvDone(uint8_t* buf) {
    return 0;
}

void RF69::sendStart(uint8_t hdr, const void* ptr, uint8_t len) {
}
