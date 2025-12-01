// Teensy 4.1 XBee3 UART2 test
// Serial  = USB to PC (debug)
// Serial2 = UART to XBee3 (pins 7/8 on Teensy 4.1)
 
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 1000; // 1 second
unsigned long counter = 0;
 
void setup() {
  // USB serial for debug
  Serial.begin(115200);
  while (!Serial) {
    // Wait for USB Serial to be ready (Teensy-specific)
  }
 
  // XBee UART on Serial2 (pins 7 = RX2, 8 = TX2 on Teensy 4.1)
  // Baud must match XBee's ATBD
  Serial2.begin(9600);
 
  Serial.println("Teensy 4.1 <-> XBee3 UART2 Test");
  Serial.println("Sending a message to XBee every second...");
  Serial.println("Type into this Serial Monitor to send data to the XBee.");
}
 
void loop() {
  unsigned long now = millis();
 
  // 1) Periodically send a test message to the XBee over Serial2
  if (now - lastSendTime >= sendInterval) {
    lastSendTime = now;
 
    Serial.print("Sending to XBee: ");
    Serial.print("Hello from Teensy, count = ");
    Serial.println(counter);
 
    Serial2.print("Hello from Teensy, count = ");
    Serial2.print(counter);
    Serial2.print("\r\n"); // for readability on the receiving side
 
    counter++;
  }
 
  // 2) Forward anything from PC (USB Serial) → XBee (Serial2)
  while (Serial.available() > 0) {
    char c = Serial.read();
    Serial2.write(c);
  }
 
  // 3) Forward anything from XBee (Serial2) → PC (USB Serial)
  while (Serial2.available() > 0) {
    char c = Serial2.read();
    Serial.write(c);
  }
}