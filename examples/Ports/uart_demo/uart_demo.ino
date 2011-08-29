// Demo for the SC16IS740 UART, connected via I2C
// 2009-10-01 <jcw@equi4.com> http://opensource.org/licenses/mit-license.php
// $Id: uart_demo.pde 4747 2009-12-18 12:53:27Z jcw $

#include <Ports.h>
#include <RF12.h> // needed to avoid a linker error :(

PortI2C i2cBus (1);
UartPlug uart (i2cBus, 0x4D);

void setup () {
  Serial.begin(57600);
  Serial.println("\n[uart_demo]");
  uart.begin(57600);
}

void loop () {
    int u = uart.read();
    if (u >= 0)
        Serial.print((char) u);
        
    int s = Serial.read();
    if (s >= 0)
        uart.print((char) s);
}
