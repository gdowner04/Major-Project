ESP32 GPS & DHT11 Data Transmission via ESP-NOW
This project uses ESP32 to collect GPS coordinates and temperature/humidity data, then sends it wirelessly using ESP-NOW.

Hardware Used
ESP32
NEO-6M GPS Module
DHT11 Temperature & Humidity Sensor
How It Works
The Sender ESP32 reads data from the GPS module and DHT11 sensor.
It transmits the data wirelessly using ESP-NOW.
The Receiver ESP32 receives and displays the data.
Wiring
Component	ESP32 Pin
GPS TX	GPIO 16
GPS RX	GPIO 17
DHT11 Data	GPIO 4
Libraries Used
TinyGPS++ (for GPS)
ESP-NOW (for wireless communication)
DHT Sensor Library (for temperature & humidity)
Next Steps
Store received data
Send data to a web server
