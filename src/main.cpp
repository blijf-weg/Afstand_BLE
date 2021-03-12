#include "Arduino.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLECast.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <iostream>
//#include <cstring>

#ifndef BEACON

using namespace std;

    BLEScan *pBLEScan;

    const int scanTimeSeconds = 1;

    const char* mqtt_server = "192.168.0.137";

    const char* ssid = "telenet-648FE13";
    const char* password = "YF74spyvpdkp";

    WiFiClient espClient;
    PubSubClient client(espClient);
    long lastMsg = 0;
    char msg[50];
    int value = 0;


    class AdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
    {
        void onResult(BLEAdvertisedDevice advertisedDevice)
        {
            if (strcmp(advertisedDevice.getName().c_str(), "RadiationNathan") == 0)
            {
                //Serial.print(advertisedDevice.getName().c_str());
                //Serial.printf(": %d \n", advertisedDevice.getRSSI());
                //Serial.printf(": %s \n", advertisedDevice.getManufacturerData().c_str());
                int rssi = advertisedDevice.getRSSI();
                char cstr[16];
                char* karakters = itoa(rssi, cstr, 10);
                String rssistring= String(rssi) + "_1";
                client.publish("esp32/rssi/nathan",rssistring.c_str());
                //client.publish("esp32/rssi/nathan", karakters);
                Serial.println(rssistring.c_str());
            }
        }
    };

    void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8269Client1")) {
      Serial.println("connected");
      // Subscribe
      //client.subscribe("esp32/rssi/nathan");
      client.subscribe("esp32/humidity");
      client.subscribe("esp32/RSSI");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

    void setup()
    {
        Serial.begin(115200);
        Serial.println("Scanning...");

        setup_wifi();
        client.setServer(mqtt_server, 1883);
        //client.setCallback(callback);
        BLEDevice::init("Radiation SCAN");
        pBLEScan = BLEDevice::getScan(); // create new scan
        pBLEScan->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks());
        pBLEScan->setActiveScan(false); // active scan (true) uses more power, but get results faster
        pBLEScan->setInterval(100);
        pBLEScan->setWindow(99); // less or equal setInterval value

        BLEDevice::init("Radiation SCAN");
        pBLEScan = BLEDevice::getScan(); // create new scan
        pBLEScan->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks());
        pBLEScan->setActiveScan(false); // active scan (true) uses more power, but get results faster
        pBLEScan->setInterval(100);
        pBLEScan->setWindow(99); // less or equal setInterval value
    }

    void loop()
    {
        if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 1000) {
    lastMsg = now;
        
    // Convert the value to a char array
    char tempString[8];
    dtostrf(69, 1, 2, tempString);
    //Serial.print("Temperature: ");
    //Serial.println(tempString);
    //client.publish("esp32/temperature", tempString);

    BLEScanResults foundDevices = pBLEScan->start(scanTimeSeconds, false);
        pBLEScan->clearResults();
    }
  }
#else
    // define BTLE name
    // CAREFUL: each character eats into your usable adv packet length
    BLECast bleCast("RadiationNathan");
    
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
        
        
        std::string s = bleCast.setManufacturerData(data, sizeof(data));
        Serial.println(s.c_str());
        delay(1000);
    }
#endif