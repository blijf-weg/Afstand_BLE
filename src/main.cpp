#include "Arduino.h"
#include <BLEDevice.h>
#include <BLECast.h>
#include <String>

#include <WiFi.h>
#include <esp_now.h>
#include <String>
#include <math.h>

    BLEScan *pBLEScan;

    const int scanTimeSeconds = 1;
    int RSSI;

    class AdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
    {
        void onResult(BLEAdvertisedDevice advertisedDevice)

        {
            String manufacturer = advertisedDevice.getManufacturerData().c_str();
            String testString = "ZW";
            int lengte = manufacturer.length();
            String vergelijken = manufacturer.substring(lengte-2, lengte);
            if (testString.equals(vergelijken))
            //if (strcmp(advertisedDevice.getName().c_str(), "test") == 0)
            {
                //Serial.print(advertisedDevice.getName().c_str());
                //Serial.print(advertisedDevice.getServiceUUID());
                RSSI = advertisedDevice.getRSSI();
                //Serial.printf(": %d \n", RSSI);
                //Serial.println(advertisedDevice.getManufacturerData().c_str());
            }
        }
    };

    void updateDisplay();
    double getDistance();

// REPLACE WITH THE MAC Address of your receiver 7C:9E:BD:F4:36:74
uint8_t broadcastAddress[] = {0x7C, 0x9E, 0xBD, 0xED, 0x48, 0xA8};

// Define variables to store BME280 readings to be sent
double getal;

// Define variables to store incoming readings
double incomingGetal;

// Variable to store if sending data was successful
String success;

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    double getal;
} struct_message;

// Create a struct_message called BME280Readings to hold sensor readings
struct_message BME280Readings;

// Create a struct_message to hold incoming sensor readings
struct_message incomingReadings;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  //Serial.print("Bytes received: ");
  //Serial.println(len);
  incomingGetal = incomingReadings.getal;
}

    void setup()
    {
        WiFi.mode(WIFI_STA);
        Serial.begin(115200);
        Serial.println("Scanning...");

        BLEDevice::init("Radiation SCAN");
        pBLEScan = BLEDevice::getScan(); // create new scan
        pBLEScan->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks());
        pBLEScan->setActiveScan(false); // active scan (true) uses more power, but get results faster
        pBLEScan->setInterval(100);
        pBLEScan->setWindow(99); // less or equal setInterval value

        // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
    }

    void loop()
    {
        BLEScanResults foundDevices = pBLEScan->start(scanTimeSeconds, false);
        pBLEScan->clearResults();
        // Set values to send
  BME280Readings.getal = getDistance();

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &BME280Readings, sizeof(BME280Readings));
   
  if (result == ESP_OK) {
    //Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  updateDisplay();
  //delay(1000);
    }

void updateDisplay(){
  // Display Readings in Serial Monitor
  //Serial.println("INCOMING READINGS");
  Serial.print(BME280Readings.getal);
  Serial.print("_");
  Serial.print(incomingReadings.getal);
  Serial.println();
}

double getDistance(){
  double MeasuredPower = -69;
  double N = 2;
  double RSSIDouble = RSSI;
  double distance = pow(10, (MeasuredPower-RSSIDouble)/(10.0*N));
  return distance;
}
