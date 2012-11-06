/// @dir ledNode
/// Programmable color ramps for the LED drivers on the LED Node.
// 2011-10-26 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>
#include <EEPROM.h>
#include <avr/sleep.h>

#define EEPROM_BASE 0x100 // store ramps starting at this offset
#define RAMP_LIMIT 100    // room for ramps 0..99, stored in EEPROM

#define LED_R 6   // the PWM pin which drives the red LED
#define LED_G 9   // the PWM pin which drives the green LED
#define LED_B 5   // the PWM pin which drives the blue LED

/// @struct Ramp
/// A "Ramp" is a target RGB color and the time in seconds to reach that color.
/// Ramps can be chained together with a non-zero ramp index in the chain field.
typedef struct {
  byte colors[3]; // red, green, blue, 0..255
  byte steps;     // number of seconds used to reach given RGB colors
  byte chain;     // next ramp to use when done, or 0 to stay as is
} Ramp;

long now[3];      // current PWM values, as 9+23 bit fractional int
long delta[3];    // current PWM deltas, as 9+23 bit fractional int
word duration;    // number of 0.01s steps remaining in this ramp
byte nextRamp;    // which saved ramp to use next (or none if zero)
MilliTimer timer; // used as timer for the 0.01s steps

static Ramp stdRamps[] = {
  {   0,   0,   0, 0, 0 }, // 0: instant off
  { 255,  85,  30, 0, 0 }, // 1: instant warm white
  { 255, 150,  75, 0, 0 }, // 2: instant cold white
  {   0,   0,   0, 5, 0 }, // 3: 5s off
  { 255,  85,  30, 5, 0 }, // 4: 5s warm white
  { 255, 150,  75, 5, 0 }, // 5: 5s cold white
  { 255,   0,   0, 5, 7 }, // 6: 5s red -> green -> blue
  {   0, 255,   0, 5, 8 }, // 7: 5s green -> blue -> red
  {   0,   0, 255, 5, 6 }, // 8: 5s blue -> red -> green
  {   7,   1,   0, 0, 0 }, // 9: instant faint red'ish yellow
};

static void setLeds () {
  // set to bits 30..23, but rounded by one extra bit (i.e. bit 22)
  analogWrite(LED_R, (byte) (((word) (now[0] >> 22) + 1) >> 1));
  analogWrite(LED_G, (byte) (((word) (now[1] >> 22) + 1) >> 1));
  analogWrite(LED_B, (byte) (((word) (now[2] >> 22) + 1) >> 1));
}

static void useRamp (const void* ptr) {
  const Ramp* ramp = (const Ramp*) ptr;
  nextRamp = ramp->chain;
  duration = ramp->steps * 100;
  for (byte i = 0; i < 3; ++i) {
    long target = (long) ramp->colors[i] << 23;
    if (duration > 0)
      delta[i] = (target - now[i]) / duration;
    else
      now[i] = target;
  }
  setLeds();
}

static void loadRamp (byte pos) {
  if (pos < RAMP_LIMIT) {
    word addr = EEPROM_BASE + pos * sizeof (Ramp);
    Ramp ramp;
    for (byte i = 0; i < sizeof (Ramp); ++i)
      ((byte*) &ramp)[i] = EEPROM.read(addr+i);
    useRamp(&ramp);
  }
}

static void saveRamp (byte pos, const void* data) {
  if (pos < RAMP_LIMIT) {
    word addr = EEPROM_BASE + pos * sizeof (Ramp);
    for (byte i = 0; i < sizeof (Ramp); ++i)
      EEPROM.write(addr+i, ((const byte*) data)[i]);
  }
}

void setup () {
  // fix timer 1 so it also runs in fast PWM mode, to match timer 0
  bitSet(TCCR1B, WGM12);
  // set up the default ramps
  for (byte i = 0; i < sizeof stdRamps / sizeof *stdRamps; ++i)
    saveRamp(i, stdRamps + i);
  // intialize wireless
  rf12_initialize(1, RF12_868MHZ, 19);
  // test code: start up with ramp #1
  //loadRamp(1);
}

void loop () {
  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_mode();

  if (timer.poll(10)) {
    if (duration > 0) {
      --duration;
      for (byte i = 0; i < 3; ++i)
        now[i] += delta[i];
      setLeds();
    } else if (nextRamp != 0)
      loadRamp(nextRamp);
  }
  
  if (rf12_recvDone() && rf12_crc == 0) {
    const byte* p = (const byte*) rf12_data;
    if (rf12_len == 1) {
      // a single byte loads the ramp from EEPROM
      loadRamp(rf12_data[0]);
    } else if (rf12_len == 1 + sizeof (Ramp)) {
      // 6 bytes, either a save or an immediate command
      // make sure that slot zero, i.e. "all-off", is never overwritten
      if (rf12_data[0] > 0) // save the date to EEPROM, if slot is not zero
        saveRamp(rf12_data[0], (const Ramp*) (rf12_data + 1));
      else // use the ramp as is without saving, if slot is zero
        useRamp((const Ramp*) (rf12_data + 1));
    }
  }
}
