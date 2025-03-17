#include <TinyGPS++.h>
#include <esp_now.h> 
#include <WiFi.h> 
#include <Preferences.h>

// Define the RX and TX pins for Serial 2
#define RXD2 16
#define TXD2 17
#define GPS_BAUD 9600
#define MAX_ESPS 3  // Maximum ESPs we track

// TinyGPS++ object
TinyGPSPlus gps;
// Used for pulling unique ESP Name from storage
Preferences preferences;
String deviceName;

// HardwareSerial instance for Serial 2
HardwareSerial gpsSerial(2);

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Broadcast to all ESP32s

// GPS data structure
struct GPSData {
    float latitude;
    float longitude;
    float speed_kmh;
    float altitude_m;
    float hdop;
    int satellites;
    long timestamp; // UNIX timestamp (seconds since 1970)
};

// Send GPS data with ESP name
struct TransmittedData {
    char name[10];  // ESP name
    GPSData gpsData;
};

// Structure for storing ESP data including its name
struct ESPNodeData {
    char name[10];  // Name of the ESP (e.g., "ESP1")
    GPSData gpsData; // GPS and sensor readings
    unsigned long lastUpdateTime; // Last received timestamp
};

ESPNodeData espNodes[MAX_ESPS];
int espNodeCount = 0;  // Number of ESPs stored

GPSData gpsDataToSend;
GPSData receivedGPSData;

// Function to update and add data for each ESP
void updateESPData(const char* espName, GPSData receivedData) { 
    for (int i = 0; i < espNodeCount; i++) {
        if (strcmp(espNodes[i].name, espName) == 0) {  // Check if the ESP already exists
            espNodes[i].gpsData = receivedData;
            espNodes[i].lastUpdateTime = receivedData.timestamp;
            return;
        }
    }

    // If it's new and we have space
    if (espNodeCount < MAX_ESPS) {
        strncpy(espNodes[espNodeCount].name, espName, sizeof(espNodes[espNodeCount].name));
        espNodes[espNodeCount].gpsData = receivedData;
        espNodes[espNodeCount].lastUpdateTime = receivedData.timestamp;
        espNodeCount++;
    } else {
        Serial.println("ESP storage full! - Increase MAX_ESPs.");
    }
}

// Callback when data is received
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
    TransmittedData receivedData;
    memcpy(&receivedData, incomingData, sizeof(receivedData));

    updateESPData(receivedData.name, receivedData.gpsData);
}

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("Last message send status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

// Function to read GPS data
GPSData readGPS() {
    GPSData gpsData = {};  // Initialize with default values

    // Read GPS data for a short time window
    unsigned long start = millis();
    while (millis() - start < 1000) {  
        while (gpsSerial.available() > 0) {
            gps.encode(gpsSerial.read());
        }
    }

    if (gps.location.isValid()) {
        gpsData.latitude = gps.location.lat();
        gpsData.longitude = gps.location.lng();
        gpsData.speed_kmh = gps.speed.kmph();
        gpsData.altitude_m = gps.altitude.meters();
        gpsData.hdop = gps.hdop.value() / 100.0;
        gpsData.satellites = gps.satellites.value();

        // Convert GPS time to UNIX time 
        if (gps.date.isValid() && gps.time.isValid()) {
            struct tm timeinfo;
            timeinfo.tm_year = gps.date.year() - 1900;
            timeinfo.tm_mon = gps.date.month() - 1;
            timeinfo.tm_mday = gps.date.day();
            timeinfo.tm_hour = gps.time.hour();
            timeinfo.tm_min = gps.time.minute();
            timeinfo.tm_sec = gps.time.second();

            gpsData.timestamp = mktime(&timeinfo); // Convert to UNIX time 
        }
    }

    return gpsData;
}

void setup() {
    Serial.begin(115200); 
    preferences.begin("ESP_Config", true); // Open storage in read mode
    deviceName = preferences.getString("device_name", "Unknown"); // Default "Unknown"
    preferences.end(); // Close storage
    
    WiFi.mode(WIFI_STA); // ESP-NOW works best in station mode
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW initialization failed!");
        return;
    }

    esp_now_register_send_cb(OnDataSent); // Register send callback
    esp_now_register_recv_cb(OnDataRecv); // Register receive callback
    
    // Register peer (broadcasting to all)
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }

    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
    Serial.println("GPS initialized.");
}

void loop() {
    gpsDataToSend = readGPS(); 

    // Ensure the data is valid before sending
    if (gpsDataToSend.latitude != 0 && gpsDataToSend.longitude != 0) {
        TransmittedData dataToSend;
        strncpy(dataToSend.name, deviceName.c_str(), sizeof(dataToSend.name));
        dataToSend.gpsData = gpsDataToSend;

        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &dataToSend, sizeof(dataToSend));

        Serial.print("Sending data from: ");
        Serial.println(dataToSend.name);
    } else {
        Serial.println("Waiting for valid GPS signal...");
    }

    delay(3000);
    printESPData();
}

// Function to print tracked ESP data
void printESPData() {
    Serial.println("Tracked ESP Nodes:");
    for (int i = 0; i < espNodeCount; i++) {
        Serial.print("ESP Name: ");
        Serial.println(espNodes[i].name);
        Serial.print("Latitude: ");
        Serial.println(espNodes[i].gpsData.latitude, 6);
        Serial.print("Longitude: ");
        Serial.println(espNodes[i].gpsData.longitude, 6);
        Serial.print("Last Updated: ");
        time_t timestamp = espNodes[i].lastUpdateTime;
        Serial.println(ctime(&timestamp));
        Serial.println("----------------------");
    }
}
