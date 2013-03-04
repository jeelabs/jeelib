/// @dir groupRelay
/// Relay packets from one net group to another to extend the range.
// 2011-01-11 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>

byte buf[66], inputValue;
MilliTimer timer;

// configuration settings, saved in eeprom, can be changed from serial port
struct {
    byte magic;
    byte freq, in_group, in_node, out_group, out_node;
    byte acks_enable, multi_node;
} config;

static word code2freq(byte code) {
    return code == 4 ? 433 : code == 9 ? 915 : 868;
}

static word code2type(byte code) {
    return code == 4 ? RF12_433MHZ : code == 9 ? RF12_915MHZ : RF12_868MHZ;
}

static void showConfig() {
    Serial.print(' ');
    Serial.print(code2freq(config.freq));
    Serial.print(':');
    Serial.print((int) config.in_group);
    Serial.print(':');
    Serial.print((int) config.in_node);
    Serial.print(" -> ");
    Serial.print(code2freq(config.freq));
    Serial.print(':');
    Serial.print((int) config.out_group);
    Serial.print(':');
    Serial.print((int) config.out_node);
    Serial.print(' ');
    Serial.print(config.acks_enable ? "ACK " : "");
    Serial.print(config.multi_node ? "MULTI" : "");
    Serial.println();
}

static void loadConfig() {
    byte* p = (byte*) &config;
    for (byte i = 0; i < sizeof config; ++i)
        p[i] = eeprom_read_byte((byte*) i);
    // if loaded config is not valid, replace it with defaults
    if (config.magic != 123) {
        config.magic = 123;
        config.freq = 8;
        config.in_group = 5;
        config.in_node = 31;
        config.out_group = 6;
        config.out_node = 30;
        config.acks_enable = 1; // 1 to support round-trip ACK relaying
        config.multi_node = 1;  // 1 to insert orig header byte when relaying
    }
    showConfig();
    rf12_initialize(config.in_node, code2type(config.freq), config.in_group);
}

static void saveConfig() {
    byte* p = (byte*) &config;
    for (byte i = 0; i < sizeof config; ++i)
        eeprom_write_byte((byte*) i, p[i]);
    loadConfig();
}

static void showString(PGM_P s) {
    for (;;) {
        char c = pgm_read_byte(s++);
        if (c == 0)
            break;
        if (c == '\n')
            Serial.print('\r');
        Serial.print(c);
    }
}

char helpText[] PROGMEM = 
    "\n"
    "Available commands:" "\n"
    "  <n> b      - set MHz band (4 = 433, 8 = 868, 9 = 915)" "\n"
    "  <nnn> g    - input network group (1..250, RFM12 only allows 212)" "\n"
    "  <nn> n     - input node ID (standard relay ids are 27..30)" "\n"
    "  <nnn> G    - output network group (1..250, RFM12 only allows 212)" "\n"
    "  <nn> N     - output node ID (standard relay ids are 27..30)" "\n"
    "  <n> a      - set ack mode (1 = send acks back, 0 = lurk)" "\n"
    "  <n> m      - set multi-node mode (1 = insert node id, 0 = pass)" "\n"
;

void showHelp() {
    showConfig();
    showString(helpText);
}

static void handleInput(char ch) {
    if ('0' <= ch && ch <= '9')
        inputValue = 10 * inputValue + ch - '0';
    else {
        switch (ch) {
            default:    showString(PSTR("Type '?' for help.\n")); return;
            case '?':   showHelp(); return;
            case 'b':   config.freq = inputValue; break;
            case 'g':   config.in_group = inputValue; break;
            case 'n':   config.in_node = inputValue; break;
            case 'G':   config.out_group = inputValue; break;
            case 'N':   config.out_node = inputValue; break;
            case 'a':   config.acks_enable = inputValue; break;
            case 'm':   config.multi_node = inputValue != 0; break;
        }
        inputValue = 0;
        saveConfig();
    }
}

void setup () {
    Serial.begin(57600);
    Serial.print("\n[groupRelay]");
    loadConfig();
}

void loop () {
    if (Serial.available())
        handleInput(Serial.read());

    if (!rf12_recvDone() || rf12_crc != 0)
        return; // nothing to do
    
    // make copies, because rf12_* will change in next rf12_recvDone
    byte hdr = rf12_hdr, len = rf12_len;
    if (config.multi_node) {
        // special case: insert original header (src node ID) as first data byte
        // careful with max-length packets in multi-node mode: drop last byte!
        // this is necessary because we're inserting an extra byte at the front
        if (len >= sizeof buf)
            --len;
        buf[0]= hdr;
    }
    memcpy(buf + config.multi_node, (void*) rf12_data, len);

    // save these for later as well, same reason as above
    byte wantsAck = RF12_WANTS_ACK, ackReply = RF12_ACK_REPLY;
    if (config.acks_enable) {
        // if we're not supposed to send back ACKs, then don't ask for 'em
        wantsAck = false;
        hdr &= ~ RF12_HDR_ACK;
    }
    
    // switch to outgoing group
    rf12_initialize(config.out_node, code2type(config.freq), config.out_group);
    
    // send our packet, once possible
    rf12_sendNow(hdr, buf, len + config.multi_node);
    
    if (wantsAck) {
        timer.set(100); // wait up to 100 ms for a valid ack packet
        wantsAck = false;
        while (!wantsAck && !timer.poll())
            wantsAck = rf12_recvDone() && rf12_crc == 0;
    }
    
    // switch back to incoming group      
    rf12_initialize(config.in_node, code2type(config.freq), config.in_group);
    
    if (wantsAck) {
        // copy ack packet to our temp buffer, same reason as above
        len = rf12_len;
        memcpy(buf, (void*) rf12_data, rf12_len);

        // send ACK packet back, once possible
        rf12_sendNow(ackReply, buf, len);
    }
}
