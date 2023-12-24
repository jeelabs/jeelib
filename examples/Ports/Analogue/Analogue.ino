// the setup routine runs once when you press reset:
int value = 0;
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:
  int sensorValue;
  for (byte c = 0; c < 4; ++c) {
  	sensorValue = sensorValue + analogRead(A0);
  }
//  sensorValue = (sensorValue + 1)>>2;
  // print out the value you read:
  if (value != sensorValue) {
  	Serial.println(sensorValue);
  	value = sensorValue;
  	delay(1000);        // delay in between reads for stability
  }
}