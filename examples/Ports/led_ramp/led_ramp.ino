// Gradually increasing and decreasing the brightness of an attached LED
// 2010-08-27 <jcw@equi4.com> http://opensource.org/licenses/mit-license.php
// $Id: $

word level;

void setup () {}

void loop () {
    int direction = - bitRead(++level, 8)
    analogWrite(6, level ^ direction);
    delay(10);
}
