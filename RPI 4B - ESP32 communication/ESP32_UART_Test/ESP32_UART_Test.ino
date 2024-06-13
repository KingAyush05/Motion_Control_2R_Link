// ESP32 serial2 hardware loop back test - jumper GPIO16 (Rx) and GPIO17 (Tx)

// see https://circuits4you.com/2018/12/31/esp32-hardware-serial2-example/
/* There are three serial ports on the ESP known as U0UXD, U1UXD and U2UXD.
 * 
 * U0UXD is used to communicate with the ESP32 for programming and during reset/boot.
 * U1UXD is unused and can be used for your projects. Some boards use this port for SPI Flash access though
 * U2UXD is unused and can be used for your projects.
*/

#define RXD2 16
#define TXD2 17

void setup() {
  // Note the format for setting a serial port is as follows: Serial2.begin(baud-rate, protocol, RX pin, TX pin);
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  Serial.println("ESP32 hardware serial test on Serial2");
  Serial.println("Serial2 Txd is on pin: "+String(TXD2));
  Serial.println("Serial2 Rxd is on pin: "+String(RXD2));
}

void loop() { //Choose Serial1 or Serial2 as required
   if (Serial.available() > 0) {
    // Read the input as a string
    String input = Serial.readStringUntil('\n');

    // Convert the input to an integer (duty cycle percentage)
//    int data = input.toInt();

    //Send that data over UART
    Serial.println("Data Sending over UART");
    Serial2.println(input);
    Serial.println("Data Sent");
    delay(2000);
   }

//  while (Serial2.available()) {
//    Serial.print(char(Serial2.read()));
//  }
  
//  while (Serial.available()) {
//    Serial2.print(char(Serial.read()));
//  }
}
