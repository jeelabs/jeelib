void setup() {
  delay(10000);
  ookPulse(375, 1125);
  Serial.setTxBit(PIN3);	// https://github.com/SpenceKonde/ATTinyCore/search?q=T84+tx
  Serial.begin(38400);
  Serial.println("raw text");
  Serial.println(F("F() macro"));
  Serial.printHex(EEDR);
  Serial.println();
  byte one = 1;
  Serial.println(one);
  Serial.println(!one);
  Serial.println(!!one);
}
void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()) {
    byte c = Serial.read();
    delay (60);
    Serial.print(c, HEX);
  }
}
static void ookPulse(int on, int off) {//   delayMicroseconds(on + 150);
//    delayMicroseconds(on + 150); 	// This will compile
   delayMicroseconds(off - 200);	// This will not compile
}
