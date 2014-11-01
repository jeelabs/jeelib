#include <avr/interrupt.h>
#include <util/crc16.h>
#if ARDUINO >= 100
#include <Arduino.h>  // Arduino 1.0
#else
#include <WProgram.h> // Arduino 0022
#endif

// pin change interrupts are currently only supported on ATmega328's
// 
#define PINCHG_IRQ 1    // uncomment this to use pin-change interrupts

// For pin change interrupts make sure you adjust the RFM_IRQ around line 130

// prog_uint8_t appears to be deprecated in avr libc, this resolves it for now
#define __PROG_TYPES_COMPAT__
#include <avr/pgmspace.h>

#define ROM_UINT8       const prog_uint8_t
#define ROM_READ_UINT8  pgm_read_byte
#define ROM_DATA        PROGMEM

#define IRQ_ENABLE      sei()
#define IRQ_NUMBER      INT0
#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)

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

#define RFM_IRQ     2
#undef  IRQ_NUMBER 
#define IRQ_NUMBER  INT2
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

#define RFM_IRQ     2     // 2 for pin change on JeeNode
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

#define RFM_IRQ     2     // 2 for pin change on JeeNode 
#define SS_DDR      DDRB
#define SS_PORT     PORTB
#define SS_BIT      2     // for PORTB: 2 = d.10, 1 = d.9, 0 = d.8

#define SPI_SS      2     // PB0
#define SPI_MOSI    3     // PB1
#define SPI_MISO    4     // PB2
#define SPI_SCK     5     // PB3

static void spiConfigPins () {
    SS_PORT |= _BV(SS_BIT);
    SS_DDR |= _BV(SS_BIT);
    PORTB |= _BV(SPI_SS);
    DDRB |= _BV(SPI_SS) | _BV(SPI_MOSI) | _BV(SPI_SCK);
}

#endif

#ifdef EIMSK    // ATMega
    #define XXMSK EIMSK
    #if PINCHG_IRQ
        #if RFM_IRQ < 8
            #define INT_BIT PCIE2
            ISR(PCINT2_vect) {// Create appropriate pin change interrupt handler
                while (!bitRead(PIND, RFM_IRQ))
                    RF69::interrupt_compat();
            }
        #elif RFM_IRQ < 14
            #define INT_BIT PCIE0
            ISR(PCINT0_vect) {// Create appropriate pin change interrupt handler 
                while (!bitRead(PINB, RFM_IRQ - 8))
                    RF69::interrupt_compat();
            }
        #else
            #define INT_BIT PCIE1
            ISR(PCINT1_vect) {// Create appropriate pin change interrupt handler
                while (!bitRead(PINC, RFM_IRQ - 14))
                    RF69::interrupt_compat();
            }
        #endif
    #endif
#endif
#ifdef GIMSK    // ATTiny
    #define XXMSK GIMSK    
    #if PINCHG_IRQ            
        #define INT_BIT PCIE1
        ISR(PCINT1_vect) {// Create appropriate pin change interrupt handler
            while (!bitRead(PINB, RFM_IRQ)) // PCINT10        //TODO CHECK IRQ
                RF69::interrupt_compat();
            }
    #endif
#endif

struct PreventInterrupt {
    PreventInterrupt () { XXMSK &= ~ _BV(INT_BIT); }
    ~PreventInterrupt () { XXMSK |= _BV(INT_BIT); }
};

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
setPrescaler(2);  // div 4, i.e. 2 MHz
    USIDR = out; // ATtiny
    uint8_t v1 = _BV(USIWM0) | _BV(USITC);
    uint8_t v2 = _BV(USIWM0) | _BV(USITC) | _BV(USICLK);
    for (uint8_t i = 0; i < 8; ++i) {
        USICR = v1;
        USICR = v2;
    }
    
setPrescaler(0);  // div 1, i.e. 8 MHz
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
