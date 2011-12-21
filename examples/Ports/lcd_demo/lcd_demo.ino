// Demo sketch for an LCD connected to I2C port via MCP23008 I/O expander
// 2009-10-31 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <PortsLCD.h>

PortI2C myI2C (1);
LiquidCrystalI2C lcd (myI2C);

void setup() {
  // set up the LCD's number of rows and columns: 
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.print("Hello, world!");
}

void loop() {
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  lcd.setCursor(0, 1);
  // print the number of seconds since reset:
  lcd.print(millis()/1000);
  // turn the backlight off, briefly
  delay(500);
  lcd.noBacklight();
  delay(50);
  lcd.backlight();
  delay(500);
}
