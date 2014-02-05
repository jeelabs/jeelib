#include <JeeLib.h>
#include <avr/eeprom.h>
#include <util/crc16.h>

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
    RF69::node = id & RF12_HDR_MASK;
    delay(20); // needed to make RFM69 work properly on power-up
    RF69::configure_compat();
    if (RF69::node != 0)
        attachInterrupt(0, RF69::interrupt_compat, RISING);
    else
        detachInterrupt(0);
    return id;
}

// same code as rf12_config, just calling rf69_initialize() instead
uint8_t rf69_config (uint8_t show) {
    uint16_t crc = ~0;
    for (uint8_t i = 0; i < RF12_EEPROM_SIZE; ++i)
        crc = _crc16_update(crc, eeprom_read_byte(RF12_EEPROM_ADDR + i));
    if (crc != 0)
        return 0;
        
    uint8_t nodeId = 0, group = 0;
    for (uint8_t i = 0; i < RF12_EEPROM_SIZE - 2; ++i) {
        uint8_t b = eeprom_read_byte(RF12_EEPROM_ADDR + i);
        if (i == 0)
            nodeId = b;
        else if (i == 1)
            group = b;
        else if (b == 0)
            break;
        else if (show)
            Serial.print((char) b);
    }
    if (show)
        Serial.println();
    
    rf69_initialize(nodeId, nodeId >> 6, group);
    return nodeId & RF12_HDR_MASK;
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

uint8_t rf69_getRssi () {
    return RF69::rssi;
}
