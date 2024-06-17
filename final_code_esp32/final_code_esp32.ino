/*This code gives custom PWMs using an ESP32 and reads angle from an encoder.
 * Errors facing initially:
 * -For high speeds, the ESP32 was rebooting due to access of illegal memory
 * -Also for low speeds of motor, there was still the same issue but after some time interval.
 * 
 * Due to these errors the angle rotated by the motor was always going back to zero.
 * 
 * Changes done to fix the issues:
 * -Added portMUX_TYPE mux which is used to ensure only one task or interrupt can access the resource at a time.
 * -Used portENTER_CRITICAl & portEXIT_CRITICAL so that the critcal code is note interrupted.
 * -Used a local copy of counter in the main loop to minimize the time spent in critical sections.
 * -Used IRAM_ATTR for the ISR function. This tells the compiler to place the specific function into the Internal RAM(IRAM) instead of flash memory
 * 
 * For more details of these changes refer to markdown file in this folder.
 */


#include <math.h>

volatile int counter,SP, error = 0; // This variable will increase or decrease depending on the rotation of encoder
volatile int temp = 360;
// Define a portMUX_TYPE variable
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

const int pwmPin = 13;  // Onboard LED

// Setting PWM properties
const int freq = 10000;
const int resolution = 10; //Resolution can be 8, 10, 12, 15

const int IN1 = 25;
const int IN2 = 33;

const int EA = 26;
const int EB = 27;
float rot_angle, post = 0;
float p = 2;
void IRAM_ATTR enc_isr0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  portENTER_CRITICAL_ISR(&mux);
  if (digitalRead(27) == LOW) {
    counter++;
  } else {
    counter--;
  }
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR enc_isr1() {
  // ai1 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check pin 2 to determine the direction
  portENTER_CRITICAL_ISR(&mux);
  if (digitalRead(26) == LOW) {
    counter--;
  } else {
    counter++;
  }
  portEXIT_CRITICAL_ISR(&mux);
}
void IRAM_ATTR pid_sig(){
  portENTER_CRITICAL_ISR(&mux);
      float sig = error*p;
    if(sig>100)
      analogWrite(pwmPin,100);
    else if(sig>0)
      analogWrite(pwmPin, sig);
    else
      analogWrite(pwmPin, 0);
  portEXIT_CRITICAL_ISR(&mux);
  }
void setup() {
  Serial.begin(115200);
  pinMode(pwmPin,OUTPUT);

  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);

  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH); 
  analogWriteResolution(pwmPin, resolution);
  analogWriteFrequency(pwmPin, freq);

  pinMode(EA, INPUT_PULLUP); // Internal pullup input pin 2 
  pinMode(EB, INPUT_PULLUP); // Internal pullup input pin 3

  // Setting up interrupts
  // A rising pulse from encoder activates ai0().
  attachInterrupt(digitalPinToInterrupt(EA), enc_isr0, RISING);
  
  // B rising pulse from encoder activates ai1().
  attachInterrupt(digitalPinToInterrupt(EB), enc_isr1, RISING);
}

void loop() {
  int localCounter;
  portENTER_CRITICAL(&mux);
  localCounter = counter;
  portEXIT_CRITICAL(&mux);

  if (localCounter != temp) {
    rot_angle = localCounter * (360.0 / 1200.0);
    int rev = (rot_angle >= 0) ? (rot_angle / 360) : ((rot_angle-360) / 360);
    post = rot_angle - 360*rev;
    Serial.println("Position = " + String(post));
    Serial.println("Angle Rotated = " + String(rot_angle));
    Serial.println("rev = " + String(rev));
    temp = localCounter;
  }
  // Check if there's any user input available
  if (Serial.available() > 0) {
    // Read the input as a string
    String input = Serial.readStringUntil('\n');
    if(input == "print"){
      Serial.println("Position global = " + String(post));
      Serial.println("error = " + String(error));
    }
    // Convert the input to an integer (duty cycle percentage)
    else
      SP = input.toInt();
  }
  if((SP-post<=180 && SP-post>0) ){       //clockwise if it does not cross 0 degree mark
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH); 
    error = SP-post;
    pid_sig();
  }
  else if ((post-SP<=180 && post-SP>0) ){   //anti-clockwise if it does not cross 0 degree mark
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2,LOW); 
    error = post - SP;
    pid_sig();
  }
  else if((post>=270 && SP+270-post<=90 && SP+360-post>0 && post!=SP)){       //clockwise if it does cross 0 degree mark
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH); 
    error = SP +360 -post;
    pid_sig();

  }
  else if ((SP>=270 && post+270-SP<=90 && post+360-SP>0 && SP!=post )){   //anti-clockwise if it does cross 0 degree mark
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2,LOW); 
    error = post +360 - SP; 
    pid_sig();
  }
}



