/// @dir packetBuf
/// This example shows how to fill a packet buffer with strings and send them.
// 2010-09-27 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

// Note: this demo code sends with broadcasting, so each node will see data
// from every other node running this same sketch. The node ID's don't matter
// (they can even be the same!) but all the nodes have to be in the same group.
// Use the RF12demo sketch to initialize the band/group/nodeid in EEPROM.
 
#include <JeeLib.h>

/// Utility class to fill a buffer with string data.
class PacketBuffer : public Print {
public:
    PacketBuffer () : fill (0) {}
    
    const byte* buffer() { return buf; }
    byte length() { return fill; }
    void reset() { fill = 0; }

#if ARDUINO < 100
    virtual void write(uint8_t ch)
        { if (fill < sizeof buf) buf[fill++] = ch; }
#else
    virtual size_t write(uint8_t ch) {
        if (fill < sizeof buf) {
            buf[fill++] = ch;
            return 1;
        }
        return 0;
    }
#endif
    
private:
    byte fill, buf[RF12_MAXDATA];
};

byte myId;              // remember my own node ID
byte needToSend;        // set when we want to send
word counter;           // incremented each second
MilliTimer sendTimer;   // used to send once a second
PacketBuffer payload;   // temp buffer to send out

void setup () {
    Serial.begin(57600);
    Serial.print("\n[rfStrings]");
    myId = rf12_config();
}

void loop () {
    if (rf12_recvDone() && rf12_crc == 0) {
        // a packet has been received
        Serial.print("GOT ");
        for (byte i = 0; i < rf12_len; ++i)
            Serial.print(rf12_data[i]);
        Serial.println();
    }
    
    // we intend to send once a second
    if (sendTimer.poll(1000)) {
        needToSend = 1;
        ++counter;
        Serial.print(" SEND ");
        Serial.println(counter);
    }

    // can only send when the RF12 driver allows us to
    if (needToSend && rf12_canSend()) {
        needToSend = 0;
        // fill the packet buffer with text to send
        payload.print("myId = ");
        payload.print(myId, DEC);
        payload.print(", counter = ");
        payload.print(counter);
        payload.print(", millis = ");
        payload.print(millis());
        // send out the packet
        rf12_sendStart(0, payload.buffer(), payload.length());
        payload.reset();
    }
}
