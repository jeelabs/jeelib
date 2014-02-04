#include <Arduino.h>
#include <avr/pgmspace.h>
#include <util/crc16.h>

#define ROM_UINT8       prog_uint8_t
#define ROM_READ_UINT8  pgm_read_byte

// ATmega168, ATmega328, etc.
#define RFM_IRQ     2
#define SS_DDR      DDRB
#define SS_PORT     PORTB
#define SS_BIT      2     // for PORTB: 2 = d.10, 1 = d.9, 0 = d.8

#define SPI_SS      10    // PB2, pin 16
#define SPI_MOSI    11    // PB3, pin 17
#define SPI_MISO    12    // PB4, pin 18
#define SPI_SCK     13    // PB5, pin 19

#define cs_pin SS_BIT

static void spiInit (void) {
    bitSet(SS_PORT, cs_pin);
    bitSet(SS_DDR, cs_pin);
    digitalWrite(SPI_SS, 1);
    pinMode(SPI_SS, OUTPUT);
    pinMode(SPI_MOSI, OUTPUT);
    pinMode(SPI_MISO, INPUT);
    pinMode(SPI_SCK, OUTPUT);
    SPCR = _BV(SPE) | _BV(MSTR);
#if F_CPU > 10000000
    // use clk/2 (2x 1/4th) for sending (and clk/8 for recv, see rf12_xferSlow)
    SPSR |= _BV(SPI2X);
#endif
    pinMode(RFM_IRQ, INPUT);
    // digitalWrite(RFM_IRQ, 1); // pull-up
}

static uint8_t spiTransfer (uint8_t cmd, uint8_t val) {
    bitClear(SS_PORT, cs_pin);
    SPDR = cmd;
    while (!(SPSR & _BV(SPIF)))
        ;
    SPDR = val;
    while (!(SPSR & _BV(SPIF)))
        ;
    bitSet(SS_PORT, cs_pin);
    return SPDR;
}
