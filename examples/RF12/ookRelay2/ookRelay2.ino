/// @dir ookRelay2
/// Generalized decoder and relay for 868 MHz and 433 MHz OOK signals.
// 2010-04-11 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>
#include "decoders.h"

#define DEBUG 1     // set to 1 to also report results on the serial port
//#define DEBUG_LED // define as pin 1..19 to blink LED on each pin change
#define NODO 0      // use the Nodo shield hardware (only 433 MHz receiver)

// RF12 communication settings
#define NODEID 19
#define NETGRP 5

// I/O pin allocations, leave any of these undefined to omit the code for it
#if NODO
#define PIN_433 2     // D.2 = 433 MHz receiver
#define POWER_433 12  // must be 1 to supply power to the receiver
#define DEBUG_LED 13  // std Arduino led is also red status led
#else
#define USE_RF12 1    // only set if the RFM12B hardware is present
#define PIN_868 14    // AIO1 = 868 MHz receiver
#define PIN_433 17    // AIO4 = 433 MHz receiver
#define PIN_DCF 15    // AIO2 = DCF77 receiver
#endif

#if PIN_868
VisonicDecoder viso;
EMxDecoder emx;
KSxDecoder ksx;
FSxDecoder fsx;

DecoderInfo di_868[] = {
    { 1, "VISO", &viso },
    { 2, "EMX", &emx },
    { 3, "KSX", &ksx },
    { 4, "FSX", &fsx },
    { -1, 0, 0 }
};

// State to track pulse durations measured in the interrupt code
volatile word pulse_868;
word last_868; // never accessed outside ISR's

ISR(ANALOG_COMP_vect) {
    word now = micros();
    pulse_868 = now - last_868;
    last_868 = now;
}
#endif

#if PIN_433
OregonDecoder orsc;
CrestaDecoder cres;
KakuDecoder kaku;
KakuADecoder kakuA; //WvD
XrfDecoder xrf;
HezDecoder hez;
FlamingoDecoder fmgo;
SmokeDecoder smk;
ByronbellDecoder byr;
ElroDecoder elro;

DecoderInfo di_433[] = {
    { 5, "ORSC", &orsc },
    { 6, "CRES", &cres },
    { 7, "KAKU", &kaku },
    { 8, "XRF", &xrf },
    { 9, "HEZ", &hez },
    { 10, "ELRO", &elro },
    { 11, "FMGO", &fmgo },
    { 12, "SMK", &smk },
    { 13, "BYR", &byr },
    { 14, "KAKUA", &kakuA },
    { -1, 0, 0 }
};

// State to track pulse durations measured in the interrupt code
volatile word pulse_433;
word last_433; // never accessed outside ISR's

#if PIN_433 >= 14
#define VECT PCINT1_vect
#elif PIN_433 >= 8
#define VECT PCINT0_vect
#else
#define VECT PCINT2_vect
#endif

ISR(VECT) {
    word now = micros();
    pulse_433 = now - last_433;
    last_433 = now;
}
#endif

// Outgoing data buffer for RF12
byte packetBuffer [RF12_MAXDATA], packetFill;

// Timer to only relay packets up to 10x per second, even if more come in.
MilliTimer sendTimer;

static void setupPinChangeInterrupt () {
#if PIN_868
    pinMode(PIN_868, INPUT);
    digitalWrite(PIN_868, 1);   // pull-up
    
    // enable analog comparator with fixed voltage reference
    ACSR = _BV(ACBG) | _BV(ACI) | _BV(ACIE);
    ADCSRA &= ~ _BV(ADEN);
    ADCSRB |= _BV(ACME);
    ADMUX = PIN_868 - 14;
#endif

#if PIN_433
    pinMode(PIN_433, INPUT);
    digitalWrite(PIN_433, 1);   // pull-up

    // interrupt on pin change
#if PIN_433 >= 14
    bitSet(PCMSK1, PIN_433 - 14);
    bitSet(PCICR, PCIE1);
#elif PIN_433 >= 8
    bitSet(PCMSK0, PIN_433 - 8);
    bitSet(PCICR, PCIE0);
#else
    bitSet(PCMSK2, PIN_433);
    bitSet(PCICR, PCIE2);
#endif
#endif
}

// Append a new data item to the outgoing packet buffer (if there is room
static void addToBuffer (byte code, const char* name, const byte* buf, byte len) {
#if DEBUG
    Serial.print(name);
    for (byte i = 0; i < len; ++i) {
        Serial.print(' ');
        Serial.print((int) buf[i]);
    }
    // Serial.print(' ');
    // Serial.print(millis() / 1000);
    Serial.println();
#endif

    if (packetFill + len < sizeof packetBuffer) {
        packetBuffer[packetFill++] = code + (len << 4);
        memcpy(packetBuffer + packetFill, buf, len);
        packetFill += len;
    } else {
#if DEBUG
        Serial.print(" dropped: ");
        Serial.print(name);
        Serial.print(", ");
        Serial.print((int) len);
        Serial.println(" bytes");
#endif        
    }
}

static void addDecodedData (DecoderInfo& di) {
    byte size;
    const byte* data = di.decoder->getData(size);
    addToBuffer(di.typecode, di.name, data, size);
    di.decoder->resetDecoder();
}

// Check for a new pulse and run the corresponding decoders for it
static void runPulseDecoders (DecoderInfo* pdi, volatile word& pulse) {
    // get next pulse with and reset it - need to protect against interrupts
    cli();
    word p = pulse;
    pulse = 0;
    sei();

    // if we had a pulse, go through each of the decoders
    if (p != 0) { 
#if DEBUG_LED
        digitalWrite(DEBUG_LED, 1);
#endif
        while (pdi->typecode >= 0) {
            if (pdi->decoder->nextPulse(p))
                addDecodedData(*pdi);
            ++pdi;
        }
#if DEBUG_LED
        digitalWrite(DEBUG_LED, 0);
#endif
    }
}

// see http://jeelabs.org/2011/01/27/ook-reception-with-rfm12b-2/
static void rf12_init_OOK () {
    rf12_initialize(0, RF12_868MHZ);

    rf12_control(0x8027); // 8027    868 Mhz;disabel tx register; disable RX
                          //         fifo buffer; xtal cap 12pf, same as xmitter
    rf12_control(0x82c0); // 82C0    enable receiver; enable basebandblock 
    rf12_control(0xA68a); // A68A    868.2500 MHz
    rf12_control(0xc691); // C691    c691 datarate 2395 kbps 0xc647 = 4.8kbps 
    rf12_control(0x9489); // 9489    VDI; FAST;200khz;GAIn -6db; DRSSI 97dbm 
    rf12_control(0xC220); // C220    datafiltercommand; ** not documented cmd 
    rf12_control(0xCA00); // CA00    FiFo and resetmode cmd; FIFO fill disabeld
    rf12_control(0xC473); // C473    AFC run only once; enable AFC; enable
                          //         frequency offset register; +3 -4
    rf12_control(0xCC67); // CC67    pll settings command
    rf12_control(0xB800); // TX register write command not used
    rf12_control(0xC800); // disable low dutycycle 
    rf12_control(0xC040); // 1.66MHz,2.2V not used see 82c0  
}

// DCF77 radio clock signal decoder
#if PIN_DCF

static word dcfWidth;
static byte dcfLevels, dcfBits, dcfParity, dcfValue[8], dcfBuf[6];

static byte dcfExtract (byte pos, byte len) {
    word *p = (word*) (dcfValue + (pos >> 3));
    byte val = (*p >> (pos & 7)) & ((1 << len) - 1);
    return val - (val / 16) * 6; // bcd -> dec
}

static byte dcfMinute () {
    dcfBuf[0] = dcfExtract(50, 8);
    dcfBuf[1] = dcfExtract(45, 5);
    dcfBuf[2] = dcfExtract(36, 6);
    dcfBuf[3] = dcfExtract(29, 6);
    dcfBuf[4] = dcfExtract(21, 7);
    dcfBuf[5] = dcfExtract(17, 1);
    return 1 <= dcfBuf[0] && dcfBuf[0] <= 99 &&
            1 <= dcfBuf[1] && dcfBuf[1] <= 12 &&
             1 <= dcfBuf[2] && dcfBuf[2] <= 31 &&
              dcfBuf[3] <= 23 && dcfBuf[4] <= 59;
}

static void dcf77setup () {
    pinMode(PIN_DCF, INPUT);
    digitalWrite(PIN_DCF, 1); // pull-up
}

static byte dcf77poll () {
    byte ok = 0;
    static word last;
    word now = millis();
    if (now != last) {
        // track signal levels using an 8-bit shift register
        dcfLevels = (dcfLevels << 1) | digitalRead(PIN_DCF);
#if DEBUG_LED
        digitalWrite(DEBUG_LED, dcfLevels & 1);
#endif
        if (dcfLevels == 0x07F) {
            // found one 0 followed by seven 1's
            if (dcfWidth > 1000) {
                if (dcfBits == 59)
                    ok = dcfMinute();
                memset(dcfValue, 0, sizeof dcfValue);
                dcfBits = 0;
            }
            dcfWidth = 0;
        } else if (dcfLevels == 0xFE) {
            // found seven 1's followed by one 0
// Serial.print(" >");
// Serial.println((int) dcfWidth);
            if (dcfWidth >= 144) {
                dcfValue[dcfBits>>3] |= _BV(dcfBits & 7);
                dcfParity ^= 1;
            }
            switch (++dcfBits) {
                case 15: dcfParity = 0;
                case 29: case 36: case 59: if (dcfParity) dcfBits = 0;
            }
            dcfWidth = 0;
        }

        dcfWidth += now - last;
        last = now;
    }
    return ok;
}

#endif // PIN_DCF

void setup () {
#if DEBUG_LED   
    pinMode(DEBUG_LED, 1);
    // brief LED flash on startup to make sure it works
    digitalWrite(DEBUG_LED, 1);
    delay(100);
    digitalWrite(DEBUG_LED, 0);
#endif

#if DEBUG
    Serial.begin(57600);
    Serial.println("\n[ookRelay2]");
#endif

#if USE_RF12
#if PIN_868
    rf12_init_OOK();
#else
    rf12_initialize(NODEID, RF12_868MHZ, NETGRP);
#endif
#endif

#if PIN_433
    setupPinChangeInterrupt();
#endif

#if POWER_433
    pinMode(POWER_433, 1);
    digitalWrite(POWER_433, 1);
#endif

#if PIN_DCF
    dcf77setup();
#endif
}

void loop () {
#if PIN_868
    runPulseDecoders(di_868, pulse_868);    
#endif

#if PIN_433
    runPulseDecoders(di_433, pulse_433);   
#endif

#if PIN_DCF
    if (dcf77poll())
        addToBuffer(0, "DCF", dcfBuf, sizeof dcfBuf);
#endif 

    if (sendTimer.poll(100) && packetFill > 0) {
#if USE_RF12
#if PIN_868
        rf12_initialize(NODEID, RF12_868MHZ, NETGRP);
#endif
        rf12_sendNow(0, packetBuffer, packetFill);
        rf12_sendWait(1);
#if PIN_868
        rf12_init_OOK();
#endif
#endif
        packetFill = 0;
    }
}
