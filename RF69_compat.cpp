#include <JeeLib.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>
#include <util/crc16.h>

// maximum transmit / receive buffer: 3 header + data + 2 crc bytes
#define RF_MAX   (RF12_MAXDATA + 5)

volatile uint16_t rf69_crc;
volatile uint8_t rf69_buf[72];

static byte nodeid; // only used in the easyPoll code
static uint8_t group;               // network group
static uint16_t frequency;          // Frequency within selected band
static int8_t matchRF = 0;          // Hardware matching value
static uint8_t txPower = 0;         // Transmitter power from eeprom
static uint8_t rxThreshold = 0;     // Receiver threshold from eeprom

// same as in RF12
#define RETRIES     8               // stop retrying after 8 times
#define RETRY_MS    1000            // resend packet every second until ack'ed

// same as in RF12
static uint8_t ezInterval;          // number of seconds between transmits
static uint8_t ezSendBuf[RF12_MAXDATA]; // data to send
static char ezSendLen;              // number of bytes to send
static uint8_t ezPending;           // remaining number of retries
static long ezNextSend[2];          // when was last retry [0] or data [1] sent

// void rf69_set_cs (uint8_t pin) {
// }

// void rf69_spiInit () {
// }
/*
#if defined(__AVR_ATmega1284P__) // Moteino MEGA
    #define IRQ_NUMBER 2
#else
    #define IRQ_NUMBER 0
#endif
*/
uint8_t rf69_initialize (uint8_t id, uint8_t band, uint8_t group=0xD4, uint16_t off=1600) {
// If we get here by calling rf12_initialize then the default values above are not used 
// defaults are collected from rf12_initialize in the rf12.h file.
    uint8_t freq = 0;
    switch (band) {
        case RF12_433MHZ: freq = 43; break;
        case RF12_868MHZ: freq = 86; break;
        case RF12_915MHZ: freq = 90; break;
    }
    RF69::setFrequency(freq * 10000000L + band * 2500L * ((off + matchRF) & 0x0FFF));
    RF69::group = group;
    RF69::node = id & RF12_HDR_MASK;
    delay(20); // needed to make RFM69 work properly on power-up
    
// I have moved the interrupt code below into RF69_avr.h routine
// where better decisions can be made about pin_change interrupts or elsewise
/*
    if (RF69::node != 0)
        attachInterrupt(IRQ_NUMBER, RF69::interrupt_compat, RISING);
    else
        detachInterrupt(IRQ_NUMBER);
////////////////////////////////////////////////////////////////////////////////
*/  
          
    RF69::configure_compat(); 

    if (txPower) RF69::control(0x91, txPower);
    if (rxThreshold) RF69::control(0xA9, rxThreshold);

    return nodeid = id;
}
/// @details
/// This replaces rf12_config(0), to be called after rf12_configSilent(). Can be
/// used to avoid pulling in the Serial port code in cases where it's not used.

void rf69_configDump () {
    uint8_t nodeId = eeprom_read_byte(RF12_EEPROM_ADDR);
    uint8_t flags = eeprom_read_byte(RF12_EEPROM_ADDR + 3);
    frequency = eeprom_read_byte(RF12_EEPROM_ADDR + 5);
    frequency = (frequency << 8) + (eeprom_read_byte(RF12_EEPROM_ADDR + 4));
    txPower = eeprom_read_byte(RF12_EEPROM_ADDR + 6);     // Store from eeprom
    rxThreshold = eeprom_read_byte(RF12_EEPROM_ADDR + 7); // Store from eeprom
    matchRF = eeprom_read_byte(RF12_EEPROM_ADDR + 8);     // Store from eeprom
    
    // " A i1 g178 @ 868 MHz "
    Serial.print(' ');
    Serial.print((char) ('@' + (nodeId & RF12_HDR_MASK)));
    if (flags & 0x80) // Defaulted config
        Serial.print('*');
    Serial.print(" i");
    Serial.print((word)nodeId & RF12_HDR_MASK);
    if (flags & 0x04)
        Serial.print('*');
    Serial.print(" g");
    Serial.print((word)eeprom_read_byte(RF12_EEPROM_ADDR + 1));
    Serial.print(" @ ");
    uint8_t band = nodeId >> 6;
    Serial.print((word)band == RF12_433MHZ ? 434 :
                 band == RF12_868MHZ ? 868 :
                 band == RF12_915MHZ ? 912 : 0);
    Serial.print(" MHz");
    if (frequency != 1600) {
        Serial.print(" o");
        Serial.print(frequency);
    }
    if (matchRF) {
          Serial.print(" ");
          if (matchRF > (-1)) Serial.print("+");           
          Serial.print(matchRF);
    }
    if (flags & 0x08) {
        Serial.print(" q1");
    }
    if (flags & 0x04) {
        Serial.print(" c1");
    }
    if (flags & 0x03) {
        Serial.print(" x");
        Serial.print(flags & 0x03);
    }
    if (txPower) {
        if (txPower != 0x9F) {
            Serial.print(" tx");
            Serial.print(txPower, HEX);
       }
    }
    if (rxThreshold) {
        if (rxThreshold != 0xA0) {
            Serial.print(" rx");
            Serial.print(rxThreshold, HEX);
       }
    }
    Serial.println();
}

// same code as rf12_config(Silent), just calling rf69_initialize() instead
uint8_t rf69_configSilent () {
    uint16_t crc = ~0;
    for (uint8_t i = 0; i < RF12_EEPROM_SIZE; ++i) {
        byte e = eeprom_read_byte(RF12_EEPROM_ADDR + i);
        crc = _crc16_update(crc, e);
    }
    if (crc || (eeprom_read_byte(RF12_EEPROM_ADDR + 2) != RF12_EEPROM_VERSION))
        return 0;
        
    uint8_t nodeId = 0, group = 0, RegPaLvl = 0,  RegRssiThresh = 0;   
    uint16_t frequency = 0;  
     
    nodeId = eeprom_read_byte(RF12_EEPROM_ADDR + 0);
    group  = eeprom_read_byte(RF12_EEPROM_ADDR + 1);
    txPower = eeprom_read_byte(RF12_EEPROM_ADDR + 6);     // Store from eeprom
    rxThreshold = eeprom_read_byte(RF12_EEPROM_ADDR + 7); // Store from eeprom
    matchRF = eeprom_read_byte(RF12_EEPROM_ADDR + 8); // Store hardware matching 

    frequency = eeprom_read_byte(RF12_EEPROM_ADDR + 5);// Avoid eeprom_read_word
    frequency = (frequency << 8) + (eeprom_read_byte(RF12_EEPROM_ADDR + 4));
                                                            
    rf69_initialize(nodeId, nodeId >> 6, group, frequency);
    return nodeId & RF12_HDR_MASK;
}

/// @deprecated Please switch over to rf12_configSilent() and rf12_configDump().
uint8_t rf69_config (uint8_t show) {
    uint8_t id = rf69_configSilent();
    if (show)
        rf12_configDump();
    return id;
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

void rf69_skip_hdr (uint8_t skip) {
    RF69::skip_hdr(skip);
}

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
    while (RF69::sending())
        if (mode) {
            set_sleep_mode(mode == 3 ? SLEEP_MODE_PWR_DOWN :
#ifdef SLEEP_MODE_STANDBY
                           mode == 2 ? SLEEP_MODE_STANDBY :
#endif
                                       SLEEP_MODE_IDLE);
            sleep_mode();
        }
}

void rf69_onOff (uint8_t value) {
    // TODO: not yet implemented
}

void rf69_sleep (char n) {
    RF69::sleep(n == RF12_SLEEP);
}

// char rf69_lowbat () {
// }

// same as in RF12
void rf69_easyInit (uint8_t secs) {
    ezInterval = secs;
}

// same as in RF12, but with rf69_* calls i.s.o. rf12_*
char rf69_easyPoll () {
    if (rf69_recvDone() && rf12_crc == 0) { // TODO is rf12_crc correct here?
        byte myAddr = nodeid & RF12_HDR_MASK;
        if (rf12_hdr == (RF12_HDR_CTL | RF12_HDR_DST | myAddr)) {
            ezPending = 0;
            ezNextSend[0] = 0; // flags succesful packet send
            if (rf12_len > 0)
                return 1;
        }
    }
    if (ezPending > 0) {
        byte newData = ezPending == RETRIES;
        long now = millis();
        if (now >= ezNextSend[newData] && rf69_canSend()) {
            ezNextSend[0] = now + RETRY_MS;
            if (newData)
                ezNextSend[1] = now +
                    (ezInterval > 0 ? 1000L * ezInterval
                                    : (nodeid >> 6) == RF12_868MHZ ?
                                            13 * (ezSendLen + 10) : 100);
            rf69_sendStart(RF12_HDR_ACK, ezSendBuf, ezSendLen);
            --ezPending;
        }
    }
    return ezPending ? -1 : 0;
}

// same as in RF12
char rf69_easySend (const void* data, uint8_t size) {
    if (data != 0 && size != 0) {
        if (ezNextSend[0] == 0 && size == ezSendLen &&
                                    memcmp(ezSendBuf, data, size) == 0)
            return 0;
        memcpy(ezSendBuf, data, size);
        ezSendLen = size;
    }
    ezPending = RETRIES;
    return 1;
}

void rf69_encrypt (const uint8_t*) {
    // TODO: not yet implemented
}

uint16_t rf69_control (uint16_t cmd) {
    // the RF69's API is different: use top 8 bits as reg + w/r flag, and
    // bottom 8 bits as the value to store, result is only 8 bits, not 16
    return RF69::control(cmd >> 8, cmd);
}
