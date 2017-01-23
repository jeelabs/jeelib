This turns a JeeNode (or other 3.3V Arduino clone) into a programmer for the
JeeNode Zero, loading a copy of Mecrisp Forth on it via the ROM boot loader.

Required wiring connections are just the 6 FTDI pins, including DTR and RTS:

```
#define RX_PIN      7   // P4D: digital pin on JeePort #4
#define TX_PIN      17  // P4A: analog  pin on JeePort #4
#define RESET_PIN   4   // P1D: digital pin on JeePort #1
#define BOOT0_PIN   14  // P1A: analog  pin on JeePort #1
```

Plus of course ground and +5V.

Example output on the Arduino IDE's serial console:

```
[uploadJNZ] l052-mecrisp.h 20500

  Connecting: .. OK
Boot version: 0x31
   Chip type: 0x417
 Read unprot: OK
    Resuming: .. OK
Write unprot: OK
    Resuming: .. OK
     Erasing: OK
     Writing: ................ [etc] ................ OK
        Done: 20500 bytes uploaded.
Mecrisp-Stellaris RA 2.3.3 with M0 core for STM32L053C8 by Matthias Koch
```

Note: 115200 baud reception via Software Serial is tricky, some input data
corruption may occur, for example when timer interrupts interfere with timing.
As a result, some bytes received after upload + reset may be incorrect. The
flashing itself runs at 57600 baud to avoid such timing problems.
