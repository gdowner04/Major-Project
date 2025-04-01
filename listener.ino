#include "BluetoothSerial.h"

BluetoothSerial SerialBT;
const int buttonPin = 4;  // Set the GPIO pin for the button
bool listening = false;
unsigned long startTime = 0;
const unsigned long listenDuration = 30000; // 30 seconds

void setup() {
  Serial.begin(115200);
  pinMode(buttonPin, INPUT_PULLUP); // Set button as input with pull-up resistor
  SerialBT.begin("ESP Receiver");  // Start Bluetooth
  Serial.println("Bluetooth Receiver Ready. Press button to start listening...");
}

void loop() {
  // Check if button is pressed
  if (digitalRead(buttonPin) == LOW && !listening) {
    Serial.println("Button pressed! Listening for incoming data...");
    listening = true;
    startTime = millis();  // Record start time
  }

 // If listening mode is active
  if (listening) {
    if (SerialBT.available()) {  // Check if data is received
      String receivedData = SerialBT.readString();  // Read data
      Serial.print("Received: ");
      Serial.println(receivedData);
    }

    // Stop listening after 30 seconds
    if (millis() - startTime >= listenDuration) {
      Serial.println("Listening time over. Waiting for button press...");
      listening = false;
    }
  }

  delay(100);  // Small delay to reduce CPU usage
}