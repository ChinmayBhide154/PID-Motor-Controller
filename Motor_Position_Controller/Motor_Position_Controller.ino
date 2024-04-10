
#include <PID_v1.h>


//const int frw = 7;
//const int rev = 6;
const int clk = 7;
const int pulse = 6;

void setup() {
  // Start serial communication for debugging
  Serial.begin(9600);
  //pinMode(frw, OUTPUT);
  //pinMode(rev, OUTPUT);

}

void loop() {
  digitalWrite(frw, HIGH);
  delay(1);
  digitalWrite(rev, LOW);

  //delay(100); // Just to make the Serial output less frequent
}

