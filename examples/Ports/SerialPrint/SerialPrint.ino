void setup() {
  delay(5000);
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
  uint8_t count = Serial.available();
  if (count) {
  	Serial.print("Buffer size is: "); Serial.println(count);
    byte c = Serial.read();
    delay (1);
    Serial.println(c, HEX);
    Serial.println(c, BIN);
    Serial.println((char)c);
  }
}
