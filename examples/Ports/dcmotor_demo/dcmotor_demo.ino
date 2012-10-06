/// @dir dcmotor_demo
/// Demo for the DC Motor Plug.
/// @see http://jeelabs.org/2010/11/30/meet-the-dc-motor-plug/
// 2010-11-18 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

PortI2C myport (1 /*, PortI2C::KHZ400 */);
DeviceI2C expander (myport, 0x26);

enum {
  MCP_IODIR, MCP_IPOL, MCP_GPINTEN, MCP_DEFVAL, MCP_INTCON, 
  MCP_IOCON, MCP_GPPU, MCP_INTF, MCP_INTCAP, MCP_GPIO, MCP_OLAT
};

static void exp_setup () {
    expander.send();
    expander.write(MCP_IODIR);
    expander.write(0); // all outputs
    expander.stop();
}

static void exp_write (byte value) {
    expander.send();
    expander.write(MCP_GPIO);
    expander.write(value);
    expander.stop();
}


void setup () {
    Serial.begin(57600);
    Serial.println("\n[dcmotor_demo]");
    Serial.println((int) expander.isPresent());
    exp_setup();
}

void loop () {
    // turn each motor in either direction for one second
    for (byte i = 0; i < 4; ++i) {
        exp_write(1 << i);
        delay(1000);
        exp_write(0x0);
        delay(1000);
    }
}
