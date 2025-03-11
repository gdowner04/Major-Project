#include <Preferences.h>

Preferences preferences;

void setup() {
    Serial.begin(115200);
    
    preferences.begin("ESP_Config", false); // Open storage namespace
    preferences.putString("device_name", "ESP1"); // Change "ESP1" for each device
    
    Serial.println("Device name stored!");
    preferences.end(); // Close storage
}

void loop() {
}
