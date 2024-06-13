const int pwmPin = 2; 

// Setting PWM properties
const int freq = 5;
const int ledChannel = 0;
const int resolution = 10; //Resolution can be 8, 10, 12, 15

void setup(){

  // Start the Serial communication
  Serial.begin(115200);
  pinMode(pwmPin,OUTPUT);

  pinMode(12,OUTPUT);
  pinMode(14,OUTPUT);

  digitalWrite(12,HIGH);
  digitalWrite(14,LOW); 

  ledcSetup(ledChannel, freq, resolution);

  ledcAttachPin(pwmPin, ledChannel);
}

void loop() {
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
