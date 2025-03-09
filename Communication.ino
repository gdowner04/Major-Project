#include <esp_now.h>
#include <WiFi.h>

// Define a structure to hold the message
typedef struct struct_message {
    char msg[32]; // Message content (up to 31 chars + null terminator)
} struct_message;

struct_message dataToSend;
struct_message receivedData;

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Broadcast to all ESP32s

// Callback when data is received
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
    memcpy(&receivedData, incomingData, sizeof(receivedData));
    Serial.print("Received message: ");
    Serial.println(receivedData.msg);
}

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("Last message send status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
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
    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }
}

void loop() {
    // Prepare and send message every 2 seconds
    strcpy(dataToSend.msg, "Hello from ESP32 - Power Bank");

    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &dataToSend, sizeof(dataToSend));

    /*if (result == ESP_OK) {
        Serial.println("Message sent successfully");
    } else {
        Serial.println("Error sending message");
    }*/

    delay(2000); // Wait before sending again
}

