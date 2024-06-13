#define ENC_A 32 // Pin for Encoder A
#define ENC_B 33 // Pin for Encoder B

int A,B;

void encoder_isr() {
  // Reading the current state of encoder A and B
  A = digitalRead(ENC_A);
  B = digitalRead(ENC_B);

}

void setup() {

    Serial.begin(115200);
    pinMode(ENC_A, INPUT_PULLUP);
    pinMode(ENC_B, INPUT_PULLUP);
    
    // Attaching the ISR to encoder A
    attachInterrupt(digitalPinToInterrupt(ENC_A), encoder_isr, CHANGE);

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(String(A) + " " + String(B));
  delay(100);
}
