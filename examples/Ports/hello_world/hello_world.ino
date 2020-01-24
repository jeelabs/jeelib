int incomingByte = 0; // for incoming serial data

void setup() {
  Serial.begin(115200); // opens serial port, sets data rate to 115200 bps
  Serial.println("\nHello world, enter characters at the keyboard");
}

void loop() {
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
