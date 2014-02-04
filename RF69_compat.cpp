#include <JeeLib.h>

volatile uint16_t rf69_crc;
volatile uint8_t rf69_buf[72];

// void rf69_set_cs (uint8_t pin) {
// }

// void rf69_spiInit () {
// }

uint8_t rf69_initialize (uint8_t id, uint8_t band, uint8_t group) {
    RF69::frf = band == RF12_433MHZ ? 0x6C4000L : // or 0x6C8000 for 434 MHz?
                band == RF12_868MHZ ? 0xD90000L : 0xE4C000L;
    RF69::group = group;
    RF69::node = id;
    delay(20); // needed to make RFM69 work properly on power-up
    RF69::configure_compat();
    return id;
}

uint8_t rf69_config (uint8_t show) {
    rf69_initialize(31, RF12_868MHZ, 5);
    return 31; // TODO
}

uint8_t rf69_recvDone () {
    rf69_crc = RF69::recvDone_compat((uint8_t*) rf69_buf);
    return rf69_crc != ~0;
}

uint8_t rf69_canSend () {
    return RF69::canSend();
}

// void rf69_sendStart (uint8_t hdr) {
// }

void rf69_sendStart (uint8_t hdr, const void* ptr, uint8_t len) {
    RF69::sendStart_compat(hdr, ptr, len);
}

// void rf69_sendStart (uint8_t hdr, const void* ptr, uint8_t len, uint8_t sync) {
// }

void rf69_sendNow (uint8_t hdr, const void* ptr, uint8_t len) {
    while (!rf69_canSend())
        rf69_recvDone();
    rf69_sendStart(hdr, ptr, len);
}

void rf69_sendWait (uint8_t mode) {
    // TODO
}

void rf69_onOff (uint8_t value) {
    // TODO
}

void rf69_sleep (char n) {
    // TODO
}

// char rf69_lowbat () {
// }

// void rf69_easyInit (uint8_t secs) {
// }

// char rf69_easyPoll () {
// }

// char rf69_easySend (const void* data, uint8_t size) {
// }

// void rf69_encrypt (const uint8_t*) {
// }

// uint16_t rf69_control (uint16_t cmd) {
// }
