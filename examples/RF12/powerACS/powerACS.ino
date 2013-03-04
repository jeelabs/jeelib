/// @dir powerACS
/// Measure power from an ACS714.
// 2011-09-13 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php
// $Id: $

// ACS714 output pin is tied to AIO3. Power is 5V, so it swings around 2.5V.
// Basic approach is to track a moving average, and to calculate the average
// *absolute* difference from this value. Result is in microvolts.

#include <JeeLib.h>
#define DEBUG 0

Port measure (3);
MilliTimer report;
word count;
uint32_t total, avg = 2500000; // best guess for startup

void setup () {
#if DEBUG
  Serial.begin(57600);
  Serial.println("\n[powerACS]");
#endif
  rf12_initialize(17, RF12_868MHZ, 5);
}

void loop () {
  // convert ADC reading to microvolts 0..3300000
  uint32_t value = measure.anaRead() * (3300000L / 1023);
  
  // keep track of the (slow-) moving average
  avg = (499L * avg + value + 250) / 500;

  // accumulate the absolute differences from the average
  if (value > avg)
    total += value - avg;
  else if (value < avg)
    total += avg - value;
  ++count;
  
  if (report.poll(1000)) {
    // the reported value is the average absolute value
    uint32_t range = total / count;
#if DEBUG    
    Serial.print(value);
    Serial.print(' ');
    Serial.print(avg);
    Serial.print(' ');
    Serial.print(count);
    Serial.print(' ');
    Serial.print(total);
    Serial.print(' ');
    Serial.println(range);
#else
    rf12_sendNow(0, &range, sizeof range);
#endif    
    total = count = 0;
  }
}
