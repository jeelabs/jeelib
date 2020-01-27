int incomingByte = 0; // for incoming serial data

void setup() {
  Serial.begin(115200); // opens serial port, sets data rate to 115200 bps
  Serial.println("\nHello world, enter characters at the keyboard");
Serial.println(INT0);
Serial.println(INT1);
Serial.print("EIMSK:");
Serial.println(EIMSK, BIN);
Serial.println(_BV(INT0));
Serial.println(_BV(INT1));
Serial.println( (_BV(INT0) | _BV(INT1)));
//EIMSK |= ( _BV(INT1) | _BV(INT0) );
Serial.print("EIMSK:");
Serial.println(EIMSK, BIN);
attachInterrupt(INT0, interrupt_stub0, RISING);           
Serial.print("EIMSK:");
Serial.println(EIMSK, BIN);

}

void loop() {
//  Serial.println("\nHello world, enter characters at the keyboard");
  // send data only when you receive data:
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();

    // say what you got:
    Serial.println("I received:");
    Serial.println((char)incomingByte);
    Serial.println(incomingByte, BIN);
    Serial.println(incomingByte, DEC);
    Serial.println(incomingByte, HEX);
  }
}
void interrupt_stub0 () {

}