/// @dir indoor
/// Example indoor temp + humidty + barometer, mounted on a Graphics Board.
// 2010-11-17 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <ST7565.h>
#include <JeeLib.h>
#include <PortsBMP085.h>
#include <PortsSHT11.h>

ST7565 glcd(14, 4, 17, 7);
SHT11 th_sensor (3);        // port 3
PortI2C two (2);            // port 2
BMP085 p_sensor (two, 3);   // ultra high resolution
MilliTimer timer;
char outBuf [25];

// this has to be added since we're using the watchdog for low-power waiting
ISR(WDT_vect) { Sleepy::watchdogEvent(); }

// spend a little time in power down mode while the SHT11 does a measurement
static void shtDelay () {
    Sleepy::loseSomeTime(32); // must wait at least 20 ms
}

void setup () {
    rf12_initialize(1, RF12_868MHZ);
    rf12_sleep(RF12_SLEEP); // power down

    p_sensor.getCalibData();

    glcd.st7565_init();
    glcd.st7565_command(CMD_DISPLAY_ON);
    glcd.st7565_command(CMD_SET_ALLPTS_NORMAL);
    glcd.st7565_set_brightness(0x15);
}

void loop () {
    glcd.clear();

    float h, t, d;
    th_sensor.measure(SHT11::HUMI, shtDelay);        
    th_sensor.measure(SHT11::TEMP, shtDelay);
    th_sensor.calculate(h, t);
    d = th_sensor.dewpoint(h, t);
    // convert back to ints
    int temp = t * 10 + 0.5;
    int humi = h + 0.5;
    int dewp = d * 10 + 0.5;
    
    // sensor readout takes some time, so go into power down while waiting
    int temp2;
    long pres;
    p_sensor.startMeas(BMP085::TEMP);
    Sleepy::loseSomeTime(16);
    p_sensor.getResult(BMP085::TEMP);
    p_sensor.startMeas(BMP085::PRES);
    Sleepy::loseSomeTime(32);
    p_sensor.getResult(BMP085::PRES);
    p_sensor.calculate(temp2, pres);
    
    glcd.drawstring(0, 0, "SHT11:");
    sprintf(outBuf, "       Temp %4d.%d C", temp/10, temp%10);
    glcd.drawstring(0, 1, outBuf);
    sprintf(outBuf, "   Dewpoint %4d.%d C", dewp/10, dewp%10);
    glcd.drawstring(0, 2, outBuf);
    sprintf(outBuf, "   Humidity %4d   %%", humi);
    glcd.drawstring(0, 3, outBuf);

    glcd.drawstring(0, 5, "BMP085:");
    sprintf(outBuf, "       Temp %4d.%d C", temp2/10, temp2%10);
    glcd.drawstring(0, 6, outBuf);
    sprintf(outBuf, "   Pressure %4d.%d mb", (int) (pres/100),
                                             (int) (pres%100) / 10);
    glcd.drawstring(0, 7, outBuf);

    glcd.display();
    Sleepy::loseSomeTime(1000);
}
