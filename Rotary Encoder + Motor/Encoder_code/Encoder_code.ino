#include <math.h>

volatile float temp, counter = 0;// This variable will increase or decrease depending on the rotation of encoder

const int EA = 2;
const int EB = 3;

void setup() {
  Serial.begin(115200);

  pinMode(EA, INPUT_PULLUP); // Internal pullup input pin 2 
  pinMode(EB, INPUT_PULLUP); // Internal pullup input pin 3

  // Setting up interrupts
  // A rising pulse from encoder activates ai0().
  attachInterrupt(0, ai0, RISING);
  
  // B rising pulse from encoder activates ai1().
  attachInterrupt(1, ai1, RISING);
}

void loop() {
  // Send the value of counter
  if (counter != temp) {
    float rot_angle = counter*(360.0/600.0);
    float post = fmod(rot_angle,360.0);
    Serial.println("Position = " + String(post));
    Serial.println("Angle Rotated = " + String(rot_angle));  
    temp = counter;
  }
}

void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if (digitalRead(EB) == LOW) {
    counter++;
  } else {
    counter--;
  }
}

void ai1() {
  // ai1 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check pin 2 to determine the direction
  if (digitalRead(EA) == LOW) {
    counter--;
  } else {
    counter++;
  }
}
