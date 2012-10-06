/// @dir output_stepper
/// Use Output Plug as a unipolar stepper motor driver.
// 2009-11-12 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php
// see http://jeelabs.org/2009/11/17/meet-the-output-plug/

// this code was inspired by Chad Phillips' example code at:
//  http://windmeadowdotcom.googlecode.com/svn/trunk/examples/stepper.c

#include <JeeLib.h>

enum {
  MCP_IODIR, MCP_IPOL, MCP_GPINTEN, MCP_DEFVAL, MCP_INTCON, MCP_IOCON,
  MCP_GPPU, MCP_INTF, MCP_INTCAP, MCP_GPIO, MCP_OLAT
};

PortI2C myport (1);
DeviceI2C stepper (myport, 0x26);

static void exp_setup () {
    stepper.send();
    stepper.write(MCP_IODIR);
    stepper.write(0); // all outputs
    stepper.stop();
}

static void exp_write (byte value) {
    stepper.send();
    stepper.write(MCP_GPIO);
    stepper.write(value);
    stepper.stop();
}

static void wait () {
    delay(2);
}

static void forward (byte steps) {
    for (byte i = 0; i < steps; ++i) {
        wait(); exp_write(0x05);
        wait(); exp_write(0x09);
        wait(); exp_write(0x0A);
        wait(); exp_write(0x06);
    }
}

static void reverse (byte steps) {
    for (byte i = 0; i < steps; ++i) {
        wait(); exp_write(0x06);
        wait(); exp_write(0x0A);
        wait(); exp_write(0x09);
        wait(); exp_write(0x05);
    }
}

void setup() {
	Serial.begin(57600);
	Serial.println("\n[output_stepper]");
    exp_setup();
}

void loop() {	
    forward(250);
    reverse(250);
}
