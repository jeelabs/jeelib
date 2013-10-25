
#include <ttyIn.h>
#include <jeelib.h>

ttyIn tty =  ttyIn(10);

void setup()  {
  Serial.begin(9600);
  Serial.println("Goodnight moon!");
  delay(1000);
  // set the data rate for the SoftwareSerial port
  tty.begin(19200); // Because code is set up for 16MHz processor and we are 8MHz
  delay(1000);
}

void loop()                     // run over and over again
{
      byte c = tty.read();
      if ((c > 31) && (c < 123)) {
        Serial.print((char)c);
      }
}

