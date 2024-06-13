// PWM signals using an ESP32

#define LED 13  // Onboard LED

int brightness = 0;
int fade = 5;

// Setting PWM properties
const int freq = 5;
const int ledChannel = 0;
const int resolution = 10; //Resolution can be 8, 10, 12, 15

void setup(){
  Serial.begin(115200);
  pinMode(LED,OUTPUT);

  pinMode(12,OUTPUT);
  pinMode(14,OUTPUT);

  digitalWrite(12,HIGH);
  digitalWrite(14,LOW); 

  ledcSetup(ledChannel, freq, resolution);

  ledcAttachPin(LED, ledChannel);
}

void loop(){
  // PWM Values varies from 0 to 1023 since we are using 10 bit resolution
  Serial.println("10 % PWM");
  ledcWrite(ledChannel, 102);
  delay(1000);

  Serial.println("20 % PWM");
  ledcWrite(ledChannel, 205);
  delay(1000);
  
  Serial.println("40 % PWM");
  ledcWrite(ledChannel, 410);
  delay(1000);

  Serial.println("80 % PWM");
  ledcWrite(ledChannel, 818);
  delay(1000);

  Serial.println("100 % PWM");
  ledcWrite(ledChannel, 1024);
  delay(1000);
}
