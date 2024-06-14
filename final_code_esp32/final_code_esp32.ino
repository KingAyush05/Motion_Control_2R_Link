#include <math.h>

volatile float temp, counter = 0; // This variable will increase or decrease depending on the rotation of encoder
const int pwmPin 13  // Onboard LED

// Setting PWM properties
const int freq = 5;
const int ledChannel = 0;
const int resolution = 10; //Resolution can be 8, 10, 12, 15

void setup() {
  Serial.begin(115200);
  pinMode(LED,OUTPUT);

  pinMode(12,OUTPUT);
  pinMode(14,OUTPUT);

  digitalWrite(12,HIGH);
  digitalWrite(14,LOW); 

  ledcSetup(ledChannel, freq, resolution);

  ledcAttachPin(LED, ledChannel);

  pinMode(2, INPUT_PULLUP); // Internal pullup input pin 2 
  pinMode(4, INPUT_PULLUP); // Internal pullup input pin 3

  // Setting up interrupts
  // A rising pulse from encoder activates ai0().
  attachInterrupt(digitalPinToInterrupt(2), ai0, RISING);
  
  // B rising pulse from encoder activates ai1().
  attachInterrupt(digitalPinToInterrupt(4), ai1, RISING);
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

  // Check if there's any user input available
  if (Serial.available() > 0) {
    // Read the input as a string
    String input = Serial.readStringUntil('\n');

    // Convert the input to an integer (duty cycle percentage)
    int dutyCycle = input.toInt();

    // Ensure the duty cycle is within 0-100 range
    if (dutyCycle < 0) dutyCycle = 0;
    if (dutyCycle > 100) dutyCycle = 100;

    // Convert the duty cycle percentage to a value between 0 and 255
    int pwmValue = map(dutyCycle, 0, 100, 0, 1023);

    // Set the PWM value
    ledcWrite(0, pwmValue);

    // Print the set duty cycle and PWM value to the Serial monitor
    Serial.print("Duty Cycle: ");
    Serial.print(dutyCycle);
    Serial.print("%, PWM Value: ");
    Serial.println(pwmValue);
  }
}

void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if (digitalRead(3) == LOW) {
    counter++;
  } else {
    counter--;
  }
}

void ai1() {
  // ai1 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check pin 2 to determine the direction
  if (digitalRead(2) == LOW) {
    counter--;
  } else {
    counter++;
  }
}
