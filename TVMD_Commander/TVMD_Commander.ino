#include <SoftwareSerial.h>


#define PIN_SW 8
#define PIN_LED 13

SoftwareSerial hc05(10,11);  //re tx
bool armed = false;

void setup()
{
   // define pin modes for tx, rx pins:

   hc05.begin(57600);
   Serial.begin(9600);

   pinMode(PIN_SW, INPUT_PULLUP);
   pinMode(PIN_LED, OUTPUT);
  }
void loop()
{
  if (digitalRead(PIN_SW) == LOW)
  {
    if (armed) {
      hc05.println("SET_ARM disarm");
      digitalWrite(PIN_LED, LOW);
      armed = false;
    }
    else {
      hc05.println("SET_ARM arm");
      digitalWrite(PIN_LED, HIGH);
      armed = true;
    }
    delay(500);
  }
}
