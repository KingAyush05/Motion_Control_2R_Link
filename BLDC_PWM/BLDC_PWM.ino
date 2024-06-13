int pwmChannel = 0; 
int frequency = 50; 
int resolution = 10;  
int pwmPin = 26;

void setup() {

    ledcSetup(pwmChannel, frequency, resolution);

    ledcAttachPin(pwmPin, pwmChannel);
    // Configuration of channel 0 with the chosen frequency and resolution
//    ledcAttach(pwmChannel, frequency, resolution);

    // Assigns the PWM channel to pin 26
    // ledcWrite(pwmChannel, pwmPin);

}

void loop(){
    ledcWrite(pwmChannel, 250); // Set to maximum throttle
    delay(2000);
    
    ledcWrite(pwmChannel, 0); // Set to minimum throttle
    delay(2000);
    
    ledcWrite(pwmChannel, 100); // Set to neutral throttle
    delay(2000);
}
