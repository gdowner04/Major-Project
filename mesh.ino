#include <TinyGPS++.h>
#include <WiFi.h>
#include <Preferences.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "painlessMesh.h"
#include <ArduinoJson.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
int currentPage = 0;
unsigned long lastPageSwitch = 0;
const int pageSwitchInterval = 3000; // Switch every 3 seconds

#define RXD2 16
#define TXD2 17
#define GPS_BAUD 9600
#define MAX_ESPS 3  // Maximum ESPs we track

#define SOS_BUTTON_PIN 4
#define MAX_SOS_ALERTS 10  // Max number of SOS alerts to store

#define MESH_PREFIX "whateverYouLike"
#define MESH_PASSWORD "somethingSneaky"
#define MESH_PORT 5555

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
TinyGPSPlus gps;
Preferences preferences;
String deviceName;
HardwareSerial gpsSerial(2);

Scheduler userScheduler;
painlessMesh mesh;

struct GPSData {
    float latitude;
    float longitude;
    float speed_kmh;
    float altitude_m;
    float hdop;
    int satellites;
    long timestamp;
};

struct TransmittedData {
    char name[10];
    GPSData gpsData;
    bool isSOS;
};

struct ESPNodeData {
    char name[10];
    GPSData gpsData;
    unsigned long lastUpdateTime;
};

struct SOSAlert {
    String espName;
    long timestamp;
};

SOSAlert sosAlerts[MAX_SOS_ALERTS];
int sosAlertCount = 0;
bool sosActive;
SOSAlert currentSOS;
long sosStartTime = millis();

ESPNodeData espNodes[MAX_ESPS];
int espNodeCount = 0;

GPSData gpsDataToSend;
GPSData receivedGPSData;

//function to prepare message 
String createJsonMessage(const TransmittedData& data){
  StaticJsonDocument<256> doc;
  doc["name"] = data.name;
  doc["latitude"] = data.gpsData.latitude;
  doc["longitude"] = data.gpsData.longitude;
  doc["timestamp"] = data.gpsData.timestamp;
  doc["isSOS"] = data.isSOS;

  String jsonString;
  serializeJson(doc, jsonString);
  return jsonString;
}

bool deserialiseJsonMessage(const String &msg, TransmittedData &data){
  StaticJsonDocument<256> doc; 
  DeserializationError error = deserializeJson(doc, msg);
  if (error){
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return false;
  }
  strlcpy(data.name,doc["name"] | "",sizeof(data.name));
  data.gpsData.latitude = doc["latitude"] | 0.0;
  data.gpsData.longitude = doc["longitude"] | 0.0;
  data.gpsData.timestamp = doc["timestamp"] | 0;
  data.isSOS = doc["isSOS"] | false; 

  return true;
}

void updateESPData(const char* espName, GPSData receivedData) {
    for (int i = 0; i < espNodeCount; i++) {
        if (strcmp(espNodes[i].name, espName) == 0) {
            espNodes[i].gpsData = receivedData;
            espNodes[i].lastUpdateTime = receivedData.timestamp;
            return;
        }
    }

    if (espNodeCount < MAX_ESPS) {
        strncpy(espNodes[espNodeCount].name, espName, sizeof(espNodes[espNodeCount].name));
        espNodes[espNodeCount].gpsData = receivedData;
        espNodes[espNodeCount].lastUpdateTime = receivedData.timestamp;
        espNodeCount++;
    } else {
        Serial.println("ESP storage full! - Increase MAX_ESPs.");
    }
}

void receivedCallback(uint32_t from, String &msg) {
    Serial.printf("Received from %u msg=%s\n", from, msg.c_str());
    TransmittedData receivedData; 
    if (deserialiseJsonMessage(msg, receivedData)){
      if (receivedData.isSOS){
        currentSOS.espName = String(receivedData.name);
        currentSOS.timestamp = receivedData.gpsData.timestamp;
        sosStartTime = millis();
        //store SOS in the array
        if(sosAlertCount < MAX_SOS_ALERTS){
          sosAlerts[sosAlertCount] = currentSOS;
          sosAlertCount++;
        }
        sosActive = true; // new sos has been received
      }
      updateESPData(receivedData.name,receivedData.gpsData);
    }



}

void newConnectionCallback(uint32_t nodeId) {
    Serial.printf("New Connection, nodeId = %u\n", nodeId);
}

void changedConnectionCallback() {
    Serial.printf("Changed connections\n");
}

void nodeTimeAdjustedCallback(int32_t offset) {
    Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(), offset);
}

void sendSOS() {
    TransmittedData dataToSend;
    strncpy(dataToSend.name, deviceName.c_str(), sizeof(dataToSend.name));
    dataToSend.gpsData = gpsDataToSend;
    dataToSend.isSOS = true;
    String jsonMessage = createJsonMessage(dataToSend);
    mesh.sendBroadcast(jsonMessage);
    Serial.println("SOS message sent!");
}

GPSData readGPS() {
    GPSData gpsData = {};

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

        if (gps.date.isValid() && gps.time.isValid()) {
            struct tm timeinfo;
            timeinfo.tm_year = gps.date.year() - 1900;
            timeinfo.tm_mon = gps.date.month() - 1;
            timeinfo.tm_mday = gps.date.day();
            timeinfo.tm_hour = gps.time.hour();
            timeinfo.tm_min = gps.time.minute();
            timeinfo.tm_sec = gps.time.second();

            gpsData.timestamp = mktime(&timeinfo);
        }
    }

    return gpsData;
}

void setup() {
    Serial.begin(115200);
    preferences.begin("ESP_Config", true);
    deviceName = preferences.getString("device_name", "Unknown");
    preferences.end();
    pinMode(SOS_BUTTON_PIN, INPUT_PULLUP);
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;);
    }

    mesh.setDebugMsgTypes(ERROR | STARTUP);
    mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
    mesh.onReceive(&receivedCallback);
    mesh.onNewConnection(&newConnectionCallback);
    mesh.onChangedConnections(&changedConnectionCallback);
    mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);

    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
    Serial.println("GPS initialized.");
}

void loop() {
    mesh.update();
    gpsDataToSend = readGPS();

    if (gpsDataToSend.latitude != 0 && gpsDataToSend.longitude != 0) {
        TransmittedData dataToSend;
        strncpy(dataToSend.name, deviceName.c_str(), sizeof(dataToSend.name));
        dataToSend.gpsData = gpsDataToSend;
        String jsonMessage = createJsonMessage(dataToSend);
        mesh.sendBroadcast(jsonMessage);

        Serial.print("Sending data from: ");
        Serial.println(dataToSend.name);
    } else {
        Serial.println("Waiting for valid GPS signal...");
    }

    if (digitalRead(SOS_BUTTON_PIN) == LOW) {
        sendSOS();
        delay(500);
    }

    printESPData();
    if (sosActive) {
        if (millis() - sosStartTime < 5000) {
            displaySOSAlert(currentSOS);
        } else {
            sosActive = false;
            display.clearDisplay();
        }
    } else {
        outputESPData(gpsDataToSend.latitude, gpsDataToSend.longitude, gpsDataToSend.timestamp);
    }

    mesh.update();
}

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

void outputESPData(float lat, float lon, long timestamp) {
    unsigned long currentMillis = millis();

    if (currentMillis - lastPageSwitch >= pageSwitchInterval) {
        currentPage = (currentPage + 1) % 3;
        lastPageSwitch = currentMillis;
    }

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 10);

    if (currentPage == 0) {
        display.print("ESP Name: ");
        display.println(deviceName);
        time_t new_timestamp = timestamp;
        struct tm *timeInfo = localtime(&new_timestamp);
        display.print("Time: ");
        display.printf("%02d:%02d:%02d", timeInfo->tm_hour, timeInfo->tm_min, timeInfo->tm_sec);
    } else {
        display.print("Lat: ");
        display.println(lat, 6);
        display.print("Lon: ");
        display.println(lon, 6);
    }

    display.display();
}

void displaySOSAlert(SOSAlert alert) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 10);
    display.print("SOS ALERT RECEIVED! - ");
    display.print("From: ");
    display.println(alert.espName);
    display.print("Time: ");
    time_t sosTime = alert.timestamp;
    struct tm *timeInfo = localtime(&sosTime);
    display.printf("%02d:%02d:%02d", timeInfo->tm_hour, timeInfo->tm_min, timeInfo->tm_sec);
    display.display();
}
