#include <TinyGPS++.h>
#include <esp_now.h> 
#include <WiFi.h> 

// Define the RX and TX pins for Serial 2
#define RXD2 16
#define TXD2 17
#define GPS_BAUD 9600

// TinyGPS++ object
TinyGPSPlus gps;
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
    char timestamp[20];  // Format: YYYY/MM/DD HH:MM:SS
};

GPSData gpsDataToSend;
GPSData receivedGPSData;

// Callback when data is recieved 
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
    memcpy(&receivedGPSData, incomingData, sizeof(receivedGPSData));

    Serial.println("Received GPS Data:");
    Serial.print("Latitude: ");
    Serial.println(receivedGPSData.latitude, 6);
    Serial.print("Longitude: ");
    Serial.println(receivedGPSData.longitude, 6);
    Serial.print("Speed (km/h): ");
    Serial.println(receivedGPSData.speed_kmh);
    Serial.print("Altitude (m): ");
    Serial.println(receivedGPSData.altitude_m);
    Serial.print("HDOP: ");
    Serial.println(receivedGPSData.hdop);
    Serial.print("Satellites: ");
    Serial.println(receivedGPSData.satellites);
    Serial.print("Timestamp: ");
    Serial.println(receivedGPSData.timestamp);
    Serial.println("");
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
        snprintf(gpsData.timestamp, sizeof(gpsData.timestamp), "%04d/%02d/%02d %02d:%02d:%02d",
                 gps.date.year(), gps.date.month(), gps.date.day(),
                 gps.time.hour(), gps.time.minute(), gps.time.second());
    }

    return gpsData;
}

void setup() {
    Serial.begin(115200);
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
    // Get the latest GPS data
    gpsDataToSend = readGPS();

    // Ensure the data is valid before sending
    if (gpsDataToSend.latitude != 0 && gpsDataToSend.longitude != 0) { 
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &gpsDataToSend, sizeof(gpsDataToSend));

        if (result == ESP_OK) {
            Serial.println("GPS Data sent successfully!");
        } else {
            Serial.println("Failed to send GPS Data!");
        }
    } else {
        Serial.println("Waiting for valid GPS signal...");
    }

    delay(2000);
}
