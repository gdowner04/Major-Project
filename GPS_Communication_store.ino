#include <TinyGPS++.h>
#include <esp_now.h> 
#include <WiFi.h> 

// Define the RX and TX pins for Serial 2
#define RXD2 16
#define TXD2 17
#define GPS_BAUD 9600
#define MAX_ESPS 3  // Maximum ESPs we track

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
    long timestamp; // UNIX timestamp (seconds since 1970)
};

struct ESPNodeData {
    uint8_t mac[6];  // Unique identifier
    GPSData gpsData; // GPS and sensor readings
    unsigned long lastUpdateTime; // Last received timestamp
};

ESPNodeData espNodes[MAX_ESPS];
int espNodeCount = 0;  // Number of ESPs stored

GPSData gpsDataToSend;
GPSData receivedGPSData;

//function to update and add data for each ESP
void updateESPData(uint8_t* senderMac, GPSData recievedData){ 
  for (int i = 0; i< espNodeCount; i++){
    if (memcmp(espNodes[i].mac, senderMac,6)== 0){ //if esp already exists
      espNodes[i].gpsData = recievedData;
      espNodes[i].lastUpdateTime = recievedData.timestamp;
      return;
    }
  }

  //if its new and we have space
  if(espNodeCount < MAX_ESPS){
    memcpy(espNodes[espNodeCount].mac, senderMac, 6);
    espNodes[espNodeCount].gpsData = recievedData;
    espNodes[espNodeCount].lastUpdateTime = recievedData.timestamp;
    espNodeCount++;
  } else {
    Serial.println("ESP storage full! - Increase MAX_ESPs.");
  }
}

// Callback when data is recieved 
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  GPSData receivedGPSData;
  memcpy(&receivedGPSData, incomingData, sizeof(receivedGPSData));

  updateESPData(info->src_addr, receivedGPSData);
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

        //convert GPS time to unix time 
        if (gps.date.isValid() && gps.time.isValid()){
          struct tm timeinfo;
          timeinfo.tm_year = gps.date.year() - 1900;
          timeinfo.tm_mon = gps.date.month() - 1;
          timeinfo.tm_mday = gps.date.day();
          timeinfo.tm_hour = gps.time.hour();
          timeinfo.tm_min = gps.time.minute();
          timeinfo.tm_sec = gps.time.second();

         gpsData.timestamp = mktime(&timeinfo); // convert to unix time 
        }
    }

    return gpsData;
}

String formatMacAddress(const uint8_t *mac) {
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return String(macStr);
}

void printESPData() {
    Serial.println("Tracked ESP Nodes:");
    for (int i = 0; i < espNodeCount; i++) {
        Serial.print("ESP MAC: ");
        Serial.println(formatMacAddress(espNodes[i].mac));
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

    delay(3000);

    printESPData();

  
}
