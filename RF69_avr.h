#include <avr/interrupt.h>
#include <util/crc16.h>            
#include <avr/sleep.h>
#if ARDUINO >= 100
#include <Arduino.h>  // Arduino 1.0
#else
#include <WProgram.h> // Arduino 0022
#endif

///////////////////////////////////////////////////////////////////////////////
#define PINCHG_IRQ  0    // Set this true to use pin-change interrupts
#define RF69_COMPAT 1    // Set this true to use the RF69 driver
                         // The above flags must be set similarly in RF12.cpp
///////////////////////////////////////////////////////////////////////////////

// For pin change interrupts make sure you adjust the RFM_IRQ around line 130

// The interrupt routine (ISR) defined by rf12.cpp routine may also set up
// determined by the RF69_COMPAT flag setting


// prog_uint8_t appears to be deprecated in avr libc, this resolves it for now
#define __PROG_TYPES_COMPAT__
#include <avr/pgmspace.h>

#define ROM_UINT8       const prog_uint8_t
#define ROM_READ_UINT8  pgm_read_byte
#define ROM_DATA        PROGMEM

#define IRQ_ENABLE      sei()

volatile byte lastPCInt;

#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)

#define INT         INT0
#define INT_NUMBER  0
#define RFM_IRQ     2
#define SS_DDR      DDRB
#define SS_PORT     PORTB
#define SS_BIT      0

#define SPI_SS      53    // PB0, pin 19
#define SPI_MOSI    51    // PB2, pin 21
#define SPI_MISO    50    // PB3, pin 22
#define SPI_SCK     52    // PB1, pin 20

static void spiConfigPins () {
    SS_PORT |= _BV(SS_BIT);
    SS_DDR |= _BV(SS_BIT);
    PORTB |= _BV(SPI_SS);
    DDRB |= _BV(SPI_SS) | _BV(SPI_MOSI) | _BV(SPI_SCK);
}

#elif defined(__AVR_ATmega644P__)

#define INT         INT0
#define INT_NUMBER  0
#define RFM_IRQ     10
#define SS_DDR      DDRB
#define SS_PORT     PORTB
#define SS_BIT      4

#define SPI_SS      4
#define SPI_MOSI    5
#define SPI_MISO    6
#define SPI_SCK     7

static void spiConfigPins () {
    SS_PORT |= _BV(SS_BIT);
    SS_DDR |= _BV(SS_BIT);
    PORTB |= _BV(SPI_SS);
    DDRB |= _BV(SPI_SS) | _BV(SPI_MOSI) | _BV(SPI_SCK);
}


#elif defined(__AVR_ATmega1284P__) // Moteino MEGA
// http://lowpowerlab.com/moteino/#whatisitMEGA

#define INT         INT2
#define INT_NUMBER  2
#define RFM_IRQ     2
#define SS_DDR      DDRB
#define SS_PORT     PORTB
#define SS_BIT      4

#define SPI_SS      4
#define SPI_MOSI    5
#define SPI_MISO    6
#define SPI_SCK     7

static void spiConfigPins () {
    SS_PORT |= _BV(SS_BIT);
    SS_DDR |= _BV(SS_BIT);
    PORTB |= _BV(SPI_SS);
    DDRB |= _BV(SPI_SS) | _BV(SPI_MOSI) | _BV(SPI_SCK);
}

#elif defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny44__)

#define INT         INT0
#define INT_NUMBER  0
#if PINCHG_IRQ
    #define RFM_IRQ     10     // 10 for pin change on PB2
#else
    #define RFM_IRQ     2      // 2 for INT0 on PB2
#endif 
#define SS_DDR      DDRB
#define SS_PORT     PORTB
#define SS_BIT      1

#define SPI_SS      1     // PB1, pin 3 Output
#define SPI_MISO    6     // PA6, pin 7 Input
#define SPI_MOSI    5     // PA5, pin 8 Output
#define SPI_SCK     4     // PA4, pin 9 Output

static void spiConfigPins () {
    SS_PORT |= _BV(SS_BIT);                 // PB1 TriState interim Pull up
    SS_DDR |= _BV(SS_BIT);                  // PB1 SS_BIT Output
    PORTB |= _BV(SPI_SS);                   // PB1 SPI_SS High    
    DDRA &= ~ _BV(SPI_MISO);                // PA6 Input
    DDRA |= _BV(SPI_MOSI) | _BV(SPI_SCK);   // Output PA5 - MOSI | PA4 - SCK
    DDRB &= ~ _BV(RFM_IRQ);                 // PB2 Input
        
}

static void setPrescaler (uint8_t mode) {
    cli();
    CLKPR = 128;      // Set CLKPCE to 1 and rest to 0;
    CLKPR = mode;
    sei();
}

#elif defined(__AVR_ATmega32U4__) //Arduino Leonardo 

#define INT         INT0
#define INT_NUMBER  0
#define RFM_IRQ     3	  // PD0, INT0, Digital3 
#define SS_DDR      DDRB
#define SS_PORT     PORTB
#define SS_BIT      6	  // Dig10, PB6

#define SPI_SS      17    // PB0, pin 8, Digital17
#define SPI_MISO    14    // PB3, pin 11, Digital14
#define SPI_MOSI    16    // PB2, pin 10, Digital16
#define SPI_SCK     15    // PB1, pin 9, Digital15

static void spiConfigPins () {
    SS_PORT |= _BV(SS_BIT);
    SS_DDR |= _BV(SS_BIT);
    PORTB |= _BV(SPI_SS);
    DDRB |= _BV(SPI_SS) | _BV(SPI_MOSI) | _BV(SPI_SCK);
}

#else // ATmega168, ATmega328, etc.

#define INT         INT0
#define INT_NUMBER  0
#if PINCHG_IRQ
    #define RFM_IRQ     18  // 18 for pin change on PB2
#else
    #define RFM_IRQ     2   // 2 for INT0 on PB2
#endif 
#define SS_DDR      DDRB
#define SS_PORT     PORTB
#define SS_BIT      2       // for PORTB: 2 = d.10, 1 = d.9, 0 = d.8

#define SPI_SS      2       // PB2: Required to enable SPI 
#define SPI_MOSI    3       // PB3
#define SPI_MISO    4       // PB4
#define SPI_SCK     5       // PB5

static void spiConfigPins () {
    SS_PORT |= _BV(SS_BIT);
    SS_DDR |= _BV(SS_BIT);  // Slave select, maybe same pin as SPI_SS
    PORTB |= _BV(SPI_SS);   // Required to enable SPI
    DDRB |= _BV(SPI_SS) | _BV(SPI_MOSI) | _BV(SPI_SCK);
}

#endif

#ifdef EIMSK    // ATMega
    #define XXMSK EIMSK
    #if PINCHG_IRQ && RF69_COMPAT
        #if RFM_IRQ < 8
            #define INT_BIT PCIE0
            ISR(PCINT0_vect) {// Create appropriate pin change interrupt handler
                volatile byte pinB = PINB;      // Read port data
                sleep_disable();
                RF69::pcIntBits |=  pinB ^ lastPCInt;
                lastPCInt = pinB;
                // Prevent more pcinterrupts(s) until pcIntBits bit is cleared
                PCMSK0 = ~(RF69::pcIntBits);
                PCMSK0 |= (1 << RFM_IRQ);  //Except Radio IRQ
                RF69::pcIntCount++;
                                
                if (pinB & (1 << RFM_IRQ)) {
                    XXMSK &= ~(1 << INT_BIT);   //Prevent nested IRQ 
                    RF69::interrupt_compat();   //Process the RFM69x IRQ
                    XXMSK |= (1 << INT_BIT);    //Restore IRQ function
                } 
            }
        #elif RFM_IRQ < 16
            #define INT_BIT PCIE1
            ISR(PCINT1_vect) {// Create appropriate pin change interrupt handler 
                volatile byte pinC = PINC;      // Read port data
                sleep_disable();
                RF69::pcIntBits |=  pinC ^ lastPCInt;
                lastPCInt = pinC;
                // Prevent more pcinterrupts(s) until pcIntBits bit is cleared
                PCMSK1 = ~(RF69::pcIntBits);
                PCMSK1 |= (1 << RFM_IRQ - 8);   //Except Radio IRQ
                RF69::pcIntCount++;
                                
                if (pinC & (1 << RFM_IRQ - 8)) {
                    XXMSK &= ~(1 << INT_BIT);   //Prevent nested IRQ 
                    RF69::interrupt_compat();   //Process the RFM69x IRQ
                    XXMSK |= (1 << INT_BIT);    //Restore IRQ function
                }
            }
            // PCINT15 is not available in ATMega328
        #else
            #define INT_BIT PCIE2
            ISR(PCINT2_vect) {// Create appropriate pin change interrupt handler
                volatile byte pinD = PIND;     // Read port data
                sleep_disable();
                RF69::pcIntBits |=  pinD ^ lastPCInt;
                lastPCInt = pinD;  
                // Prevent more pcinterrupts(s) until pcIntBits bit is cleared
                PCMSK2 = ~(RF69::pcIntBits);
                PCMSK2 |= (1 << RFM_IRQ - 16);  //Except Radio IRQ
                RF69::pcIntCount++;
                                
                if (pinD & (1 << RFM_IRQ - 16)) {
                    XXMSK &= ~(1 << INT_BIT);   //Prevent nested IRQ 
                    RF69::interrupt_compat();   //Process the RFM69x IRQ
                    XXMSK |= (1 << INT_BIT);    //Restore IRQ function
                }
            }
        #endif
    #else
        #define INT_BIT INT 
    #endif
#endif
#ifdef GIMSK    // ATTiny
    #define XXMSK GIMSK    
    #if PINCHG_IRQ && RF69_COMPAT
        #if RFM_IRQ < 8
            #define INT_BIT PCIE0
            ISR(PCINT0_vect) {// Create appropriate pin change interrupt handler
                volatile byte pinA = PINA;      // Read port data
                sleep_disable();
                RF69::pcIntBits |=  pinA ^ lastPCInt;
                lastPCInt = pinA;
                // Prevent more pcinterrupts(s) until pcIntBits bit is cleared
                PCMSK0 = ~(RF69::pcIntBits);
                PCMSK0 |= (1 << RFM_IRQ);  // Except Radio IRQ
                RF69::pcIntCount++;
                                
                if (pinA & (1 << RFM_IRQ)) {
                    XXMSK &= ~(1 << INT_BIT);
                    RF69::interrupt_compat();// Process the RFM69x interrupt
                    XXMSK |= (1 << INT_BIT);
                }
        #elif RFM_IRQ > 7 && RFM_IRQ < 12
            #define INT_BIT PCIE1
            ISR(PCINT1_vect) {// Create appropriate pin change interrupt handler
                volatile byte pinB = PINB;      // Read port data
                sleep_disable();
                RF69::pcIntBits |=  pinB ^ lastPCInt;
                lastPCInt = pinB;
                // Prevent more pcinterrupts(s) until pcIntBits bit is cleared
                PCMSK1 = ~(RF69::pcIntBits);
                PCMSK1 |= (1 << RFM_IRQ - 8);  // Except Radio IRQ
                RF69::pcIntCount++;
                                
                if (pinB & (1 << RFM_IRQ - 8)) {
                    XXMSK &= ~(1 << INT_BIT);
                    RF69::interrupt_compat();// Process the RFM69x interrupt
                    XXMSK |= (1 << INT_BIT);
                }                        
            }
        #endif
    #else
        #define INT_BIT INT
    #endif
#endif

struct PreventInterrupt {
    PreventInterrupt () { XXMSK &= ~ _BV(INT_BIT); }
    ~PreventInterrupt () { XXMSK |= _BV(INT_BIT); }
};  // Semicolon is requird

static void spiInit (void) {
    spiConfigPins();
    
#ifdef SPCR    
    SPCR = _BV(SPE) | _BV(MSTR);    
    
//    SPCR |= _BV(SPR0);  // Divide SPI by 4
    
    SPSR |= _BV(SPI2X);
#else
    USICR = _BV(USIWM0); // ATtiny
#endif    
    
    // pinMode(RFM_IRQ, INPUT);
    // digitalWrite(RFM_IRQ, 1); // pull-up
}

static uint8_t spiTransferByte (uint8_t out) {
#ifdef SPDR
    SPDR = out;
    while (!(SPSR & _BV(SPIF)))
        ;
    return SPDR;
#else
//setPrescaler(2);  // div 4, i.e. 2 MHz
    USIDR = out; // ATtiny
    uint8_t v1 = _BV(USIWM0) | _BV(USITC);
    uint8_t v2 = _BV(USIWM0) | _BV(USITC) | _BV(USICLK);
    for (uint8_t i = 0; i < 8; ++i) {
        USICR = v1;
        USICR = v2;
    }
    
//setPrescaler(0);  // div 1, i.e. 8 MHz
    return USIDR;
#endif
}

static uint8_t spiTransfer (uint8_t cmd, uint8_t val) {
    SS_PORT &= ~ _BV(SS_BIT);
    spiTransferByte(cmd);
    uint8_t in = spiTransferByte(val);
    SS_PORT |= _BV(SS_BIT);
    return in;
}

static void InitIntPin () {
#ifdef EIMSK    // ATMega
    #if PINCHG_IRQ && RF69_COMPAT
        EIMSK &= ~ (1 << INT);                // Disable INTx
        #if RFM_IRQ < 8
            if (RF69::node != 0) {
                bitClear(DDRB, RFM_IRQ);      // input
//                bitSet(PORTB, RFM_IRQ);       // pull-up
                bitSet(PCMSK0, RFM_IRQ);      // pin-change
                bitSet(PCICR, PCIE0);         // enable
            } else
                bitClear(PCMSK0, RFM_IRQ);
                
        #elif RFM_IRQ < 16
            if (RF69::node != 0) {
                bitClear(DDRC, RFM_IRQ - 8);  // input
//                bitSet(PORTC, RFM_IRQ - 8);   // pull-up
                bitSet(PCMSK1, RFM_IRQ - 8);  // pin-change
                bitSet(PCICR, PCIE1);         // enable
            } else
                bitClear(PCMSK1, RFM_IRQ - 8);
        // PCINT15 is not available in ATMega328

        #else
            if (RF69::node != 0) {
                bitClear(DDRD, RFM_IRQ - 16); // input
//                bitSet(PORTD, RFM_IRQ - 16);  // pull-up
                bitSet(PCMSK2, RFM_IRQ - 16); // pin-change
                bitSet(PCICR, PCIE2);         // enable
                lastPCInt = PIND;             // Init pin change from value                      
            } else
                bitClear(PCMSK2, RFM_IRQ - 16);
        #endif
    #elif RF69_COMPAT
        if (RF69::node != 0)
            attachInterrupt(INT_NUMBER, RF69::interrupt_compat, RISING);
        else
            detachInterrupt(INT_NUMBER);
    #endif
#endif

#ifdef GIMSK    // ATTiny
    #if PINCHG_IRQ && RF69_COMPAT
        GIMSK &= ~ (1 << INT);            // Disable INTx
        #if RFM_IRQ < 8   // Be aware of conflict with JNu serial input
            if (RF69::node != 0) {
                bitClear(DDRA, RFM_IRQ);      // input
//               bitSet(PORTA, RFM_IRQ);       // pull-up
                bitSet(PCMSK0, RFM_IRQ);      // pin-change
                bitSet(GIMSK, PCIE0);         // enable
            } else
                bitClear(PCMSK0, RFM_IRQ);                   

        #elif RFM_IRQ > 7
            if (RF69::node != 0) {
                bitClear(DDRB, RFM_IRQ - 8);  // input
//                bitSet(PORTB, RFM_IRQ - 8);   // pull-up
                bitSet(PCMSK1, RFM_IRQ - 8);  // pin-change
                bitSet(GIMSK, PCIE1);         // enable
            } else
                bitClear(PCMSK1, RFM_IRQ - 8);
        #endif                   
    #elif RF69_COMPAT
        if (RF69::node != 0)
            attachInterrupt(INT_NUMBER, RF69::interrupt_compat, RISING);
        else
            detachInterrupt(INT_NUMBER);
    #endif
#endif
}
