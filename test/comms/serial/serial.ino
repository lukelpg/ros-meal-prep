void setup() {
  Serial.begin(115200);
  while (!Serial);  // Wait for serial port to initialize
}

void loop() {
  Serial.println("Hello from ESP32!");  // Just send a message from ESP32
  delay(1000);  // Wait for 1 second

  // No need to echo received data back to Python anymore
}
