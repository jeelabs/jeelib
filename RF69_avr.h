#include <avr/interrupt.h>
#include <util/crc16.h>            
#include <avr/sleep.h>
#if ARDUINO >= 100
#include <Arduino.h>  // Arduino 1.0
#else
#include <WProgram.h> // Arduino 0022
#endif
///////////////////////////////////////////////////////////////////////////////

#define RF69_COMPAT 0  /* Set this true to use the RF69 driver

If using the above with ATTiny84 hardware the sleep functions are more limited since 
the RFM69 only provides interrupt active high and ATTiny INT0 requires active low to 
exit a sleep power down. 
There are instances where using pin change interrupts (below) can bring the CPU out
of power down sleep. */

#define PINCHG_IRQ  0   // Set this true to use pin-change interrupts
                        // The above flags must be set similarly in RF12.cpp
// NOTE: The following does not apply to the ATTiny processors which uses USI
#define OPTIMIZE_SPI 1  // comment this out to write to the RFM69x @ 125Khz
                        // otherwise frequency is 8Mhz with 16Mhz processor
///////////////////////////////////////////////////////////////////////////////

// For pin change interrupts make sure you adjust the RFM_IRQ around line 130

// The interrupt routine (ISR) defined by rf12.cpp routine may also set up
// determined by the RF69_COMPAT flag setting

#define IRQ_ENABLE      sei()

#if RF69_COMPAT
	#warning Building for RFM69xx       
#endif

volatile byte lastPCInt;

#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)

// Items below are options to match your shield/cabling
#define INT         INT1
#define INT_NUMBER  1
#define RFM_IRQ     3	// 2 for INT0 on PD2, 3 for INT1 on PD3
#define SS_DDR      DDRB
#define SS_PORT     PORTB
#define SS_BIT      PB4		// pin 23, Digital 10
// Items below fixed and determined by the ATMega hardware
#define SPI_MISO    PB3		// pin 22, Digital 50
#define SPI_MOSI    PB2		// pin 21, Digital 51
#define SPI_SCK     PB1		// pin 20, Digital 52
#define SPI_SS      PB0		// pin 19, Digital 53

static void spiConfigPins () {
    SS_PORT |= _BV(SS_BIT);
    SS_DDR |= _BV(SS_BIT);
    
    PORTB |= _BV(SPI_SS);	// PB0, Digital 53 required for SPI hardware to activate
    DDRB |= _BV(SPI_MOSI) | _BV(SPI_SCK);
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
#define RFM_IRQ     0	    // PD0, INT0, Digital3
#define SS_DDR      DDRB
#define SS_PORT     PORTB
#define SS_BIT      6	    // Dig10, PB6

#define SPI_SS      10    // PB6, pin 30, Digital10
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

#define INT          INT0   // INT0 or INT1 also used to select SYNC or RSSI
#define INT_NUMBER      0   // 0 for INT0 and 1 for INT1
#if PINCHG_IRQ
    #define RFM_IRQ    18	// 18 for pin change on PD2
#else
    #define RFM_IRQ     2	// 2 for INT0 on PD2, 3 for INT1 on PD3
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

#define SYNC_INTERRUPT  0
#define RSSI_INTERRUPT  1

#define INTERRUPT_HANDLER interrupt_compat(RSSI_INTERRUPT)

void interrupt_stub() {
        RF69::INTERRUPT_HANDLER;
}

#ifdef EIMSK    // Test for ATMega
    #if PINCHG_IRQ
        #define XXMSK PCICR
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
                    interrupt_stub();   //Process the RFM69x IRQ
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
                    interrupt_stub();   //Process the RFM69x IRQ
                    XXMSK |= (1 << INT_BIT);    //Restore IRQ function
                }
            }
            // PCINT15 is not available in ATMega328
        #else
            #define INT_BIT PCIE2
            ISR(PCINT2_vect) {// Create appropriate pin change interrupt handler
                volatile byte pinD = PIND;     // Read port data
                sleep_disable();  // What is this for?
                RF69::pcIntBits |=  pinD ^ lastPCInt;
                lastPCInt = pinD;  
                // Prevent more pcinterrupts(s) until pcIntBits bit is cleared
                PCMSK2 = ~(RF69::pcIntBits);
                PCMSK2 |= (1 << RFM_IRQ - 16);  //Except Radio IRQ
                RF69::pcIntCount++;
                                
                if (pinD & (1 << RFM_IRQ - 16)) {
                    XXMSK &= ~(1 << INT_BIT);   //Prevent nested IRQ 
                    interrupt_stub();   //Process the RFM69x IRQ
                    XXMSK |= (1 << INT_BIT);    //Restore IRQ function
                }
            }
        #endif
    #else
        #define XXMSK EIMSK
        #define INT_BIT INT 
    #endif
#endif
#ifdef GIMSK    // Test for ATTiny
    #if PINCHG_IRQ
        #define XXMSK GIMSK    
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
                    interrupt_stub();// Process the RFM69x interrupt
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
                    interrupt_stub();// Process the RFM69x interrupt
                    XXMSK |= (1 << INT_BIT);
                }                        
            }
        #endif
    #else
        #define XXMSK GIMSK    
        #define INT_BIT INT
    #endif
#endif

struct PreventInterrupt {
    PreventInterrupt () { XXMSK &= ~ _BV(INT_BIT); }
    ~PreventInterrupt () { XXMSK |= _BV(INT_BIT); }
};  // Semicolon is required

static void spiInit (void) {
    spiConfigPins();
    
#ifdef SPCR    
    SPCR = _BV(SPE) | _BV(MSTR);    

  #if OPTIMIZE_SPI == 0    
    SPCR |= _BV(SPR0);  // Divide SPI by 4
    SPCR |= _BV(SPR1);  // Divide SPI by 16
//    SPSR |= _BV(SPI2X);  // Double SPI
  #else    
    SPSR |= _BV(SPI2X);  // Double SPI
  #endif
  
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
                bitSet(PCMSK0, RFM_IRQ);      // pin-change
                bitSet(PCICR, PCIE0);         // enable
            } else
                bitClear(PCMSK0, RFM_IRQ);
                
        #elif RFM_IRQ < 16
            if (RF69::node != 0) {
                bitClear(DDRC, RFM_IRQ - 8);  // input
                bitSet(PCMSK1, RFM_IRQ - 8);  // pin-change
                bitSet(PCICR, PCIE1);         // enable
            } else
                bitClear(PCMSK1, RFM_IRQ - 8);
        // PCINT15 is not available in ATMega328

        #else
            if (RF69::node != 0) {
                bitClear(DDRD, RFM_IRQ - 16); // input
                bitSet(PCMSK2, RFM_IRQ - 16); // pin-change
                bitSet(PCICR, PCIE2);         // enable
                lastPCInt = PIND;             // Init pin change from value                      
            } else
                bitClear(PCMSK2, RFM_IRQ - 16);
        #endif
    #elif RF69_COMPAT
        if (RF69::node != 0) {
            XXMSK &= ~ _BV(INT_BIT);          // Mask radio interrupt
            attachInterrupt(INT_NUMBER, interrupt_stub, RISING);
            XXMSK |= _BV(INT_BIT);            // Enable radio interrupt
        } else {
            detachInterrupt(INT_NUMBER);
        }
    #endif
#endif

#ifdef GIMSK    // ATTiny
    #if PINCHG_IRQ && RF69_COMPAT
        GIMSK &= ~ (1 << INT);            // Disable INTx
        #if RFM_IRQ < 8   // Be aware of conflict with JNu serial input
            if (RF69::node != 0) {
                bitClear(DDRA, RFM_IRQ);      // input
                bitSet(PCMSK0, RFM_IRQ);      // pin-change
                bitSet(GIMSK, PCIE0);         // enable
            } else
                bitClear(PCMSK0, RFM_IRQ);                   

        #elif RFM_IRQ > 7
            if (RF69::node != 0) {
                bitClear(DDRB, RFM_IRQ - 8);  // input
                bitSet(PCMSK1, RFM_IRQ - 8);  // pin-change
                bitSet(GIMSK, PCIE1);         // enable
            } else
                bitClear(PCMSK1, RFM_IRQ - 8);
        #endif                   
    #elif RF69_COMPAT
        if (RF69::node != 0) {
            XXMSK &= ~ _BV(INT_BIT);          // Mask radio interrupt
            attachInterrupt(INT_NUMBER, interrupt_stub, RISING);
            XXMSK |= _BV(INT_BIT);            // Enable radio interrupt
        } else {
            detachInterrupt(INT_NUMBER);
        }
    #endif
#endif

}

static byte* SPI_Pins(void) {
    static byte pins[] = {OPTIMIZE_SPI, PINCHG_IRQ, RF69_COMPAT, SPI_SS,   
                          SPI_MOSI,SPI_MISO, SPI_SCK, RFM_IRQ, INT_NUMBER };                              
    return &pins[0];
}

