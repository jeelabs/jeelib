// Serial support (output only) for Tiny supported by TinyDebugSerial
// http://www.ernstc.dk/arduino/tinycom.html
// 9600, 38400, or 115200
// hardware\jeelabs\avr\cores\tiny\TinyDebugSerial.h Modified to
// move TinyDebugSerial from PB0 to PA3 to match the Jeenode Micro V3 PCB layout
// Connect Tiny84 PA3 to USB-BUB RXD for serial output from sketch.
// Jeenode AIO2
//
// With thanks for the inspiration by 2006 David A. Mellis and his AFSoftSerial
// code. All right reserved.
// Connect Tiny84 PA2 to USB-BUB TXD for serial input to sketch.
// Jeenode DIO2
// 9600 or 38400 at present.

#define SERIAL_BAUD 38400   // can only be 9600 or 38400

#if SERIAL_BAUD == 9600
#define BITDELAY 54          // 9k6 @ 8MHz, 19k2 @16MHz
#endif
#if SERIAL_BAUD == 38400
#define BITDELAY 12          // 28/5/14 from value 11 // 38k4 @ 8MHz, 76k8 @16MHz
#endif
#define _receivePin 8
static char _receive_buffer;
static byte _receive_buffer_index;

ISR (PCINT0_vect) {
    char i, d = 0;
    if (digitalRead(_receivePin))       // PA2 = Jeenode DIO2
        return;                         // not ready!
    whackDelay(BITDELAY - 8);
    for (i=0; i<8; i++) {
        whackDelay(BITDELAY*2 - 6);     // digitalread takes some time
        if (digitalRead(_receivePin))   // PA2 = Jeenode DIO2
            d |= (1 << i);
    }
    whackDelay(BITDELAY*2);
    if (_receive_buffer_index)
        return;
    _receive_buffer = d;                // save data
    _receive_buffer_index = 1;          // got a byte
}

// TODO: replace with code from the std avr libc library:
//  http://www.nongnu.org/avr-libc/user-manual/group__util__delay__basic.html
static void whackDelay (word delay) {
    byte tmp=0;

    asm volatile("sbiw      %0, 0x01 \n\t"
                 "ldi %1, 0xFF \n\t"
                 "cpi %A0, 0xFF \n\t"
                 "cpc %B0, %1 \n\t"
                 "brne .-10 \n\t"
                 : "+r" (delay), "+a" (tmp)
                 : "0" (delay)
                 );
}

static byte inChar () {
    byte d;
    if (! _receive_buffer_index)
        return -1;
    d = _receive_buffer; // grab first and only byte
    _receive_buffer_index = 0;
    return d;
}
void setup() {
//    bitSet(DDRB, 0);
//    bitClear(PORTB, 0);
    
    PCMSK0 |= (1<<PCINT2);  // tell pin change mask to listen to PA2
    GIMSK |= (1<<PCIE0);    // enable PCINT interrupt in general interrupt mask
    whackDelay(BITDELAY*2); // if we were low this establishes the end
    pinMode(_receivePin, INPUT);        // PA2
    digitalWrite(_receivePin, HIGH);    // pullup!

    Serial.begin(SERIAL_BAUD);
    Serial.println("Tiny Input Tester");
}
static void handleInput (char c) {
    Serial.print(c);
}

void loop() {
    if (_receive_buffer_index) {
        Serial.print("Received:");
        handleInput(inChar());
        Serial.println();
    }
    
}
