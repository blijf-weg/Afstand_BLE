#include "Arduino.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLECast.h>
#include <WiFi.h>

#ifndef BEACON

    BLEScan *pBLEScan;

    const uint32_t scanTimeSeconds = 0.1;

    const char* ssid = "hot";
    const char* password =  "hothothot";

    void connectToNetwork() {
     WiFi.begin(ssid, password);
 
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Establishing connection to WiFi..");
  }
 
  Serial.println("Connected to network");
 
}

    class AdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
    {
        void onResult(BLEAdvertisedDevice advertisedDevice)
        {
            if (strcmp(advertisedDevice.getName().c_str(), "0_RSSI") == 0)
            {
                Serial.print(advertisedDevice.getName().c_str());
                //Serial.printf(": %d \n", advertisedDevice.getRSSI());
                Serial.printf(": %s \n", advertisedDevice.getManufacturerData().c_str());
            }
            if (strcmp(advertisedDevice.getName().c_str(), "1_RSSI") == 0)
            {
                Serial.print(advertisedDevice.getName().c_str());
                //Serial.printf(": %d \n", advertisedDevice.getRSSI());
                Serial.printf(": %s \n", advertisedDevice.getManufacturerData().c_str());
            }
            if (strcmp(advertisedDevice.getName().c_str(), "2_RSSI") == 0)
            {
                Serial.print(advertisedDevice.getName().c_str());
                //Serial.printf(": %d \n", advertisedDevice.getRSSI());
                Serial.printf(": %s \n", advertisedDevice.getManufacturerData().c_str());
            }
        }
    };

    void setup()
    {
        Serial.begin(115200);
        Serial.println("Scanning...");
        esp_err_t esp_wifi_get_max_tx_power(78);

        connectToNetwork();
        Serial.println(WiFi.macAddress());
        Serial.println(WiFi.localIP());
        BLEDevice::init("Radiation SCAN");
        pBLEScan = BLEDevice::getScan(); // create new scan
        pBLEScan->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks());
        pBLEScan->setActiveScan(false); // active scan (true) uses more power, but get results faster
        pBLEScan->setInterval(100);
        pBLEScan->setWindow(99); // less or equal setInterval value
    }

    void loop()
    {
        BLEScanResults foundDevices = pBLEScan->start(scanTimeSeconds, false);
        pBLEScan->clearResults();
        long rssi = WiFi.RSSI();
        Serial.print("eigen meting: ");
        Serial.println(WiFi.RSSI());
    }
#else
    // define BTLE name
    // CAREFUL: each character eats into your usable adv packet length
    BLECast bleCast("RadiationBeacon");
    
    uint8_t cnt = 0;
    char data[5];

    void setup()
    {
        Serial.begin(115200);
        Serial.println("Starting BLE Beacon");

        bleCast.begin();
    }

    void loop()
    {
        // note -- if you have too much data, it will not be added to the adv payload

        if (cnt == 20){
            // reset
            cnt = 0;
        }

        if (cnt == 0){
            // regenerate "random" data
            int red = random(20, 50);
            int orange = random(50, 70);
            sprintf(data, "%02d&%02d", red, orange);
        }
        cnt += 1;
        
        std::string s = bleCast.setManufacturerData(data, sizeof(data));
        Serial.println(s.c_str());
        delay(1000);
    }
#endif