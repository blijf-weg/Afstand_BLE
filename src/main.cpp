#include "Arduino.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLECast.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <iostream>
#include <esp_bt.h>

using namespace std;

volatile int interruptCounter;

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter++;
  Serial.println("Timer interrupt");
  portEXIT_CRITICAL_ISR(&timerMux);
}

const uint32_t scanTimeSeconds = 0;
//const int scanTimeSeconds = 1;
const char* mqtt_server = "192.168.137.1";
//"192.168.1.2"; 
//"192.168.43.101";
//192.168.0.137
//const char* ssid = "telenet-648FE13";
//const char* password = "YF74spyvpdkp";
const char* ssid = "D84Z82H2 9418";
const char* password = "73U-229k";
const char* esp_naam = "Afstand_3";

WiFiClient Afstand_3;
PubSubClient client(Afstand_3);

BLEScan *pBLEScan;
BLECast bleCast(esp_naam);

long lastMsg = 0;
char msg[50];
int value = 0;

uint8_t cnt = 0;
char data[5];

bool send_to_broker = true;

class AdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
    void onResult(BLEAdvertisedDevice advertisedDevice)
    {
        if (strcmp(advertisedDevice.getName().c_str(), "Afstand_0") == 0)
        {
            int rssi = advertisedDevice.getRSSI();
            String rssistring= String(rssi) + "_0" + esp_naam[8];
            if(send_to_broker){
                client.publish("esp32/afstand/rssi",rssistring.c_str());
                //Serial.println(rssistring.c_str());
            }
        }
        else if (strcmp(advertisedDevice.getName().c_str(), "Afstand_1") == 0)
        {
            int rssi = advertisedDevice.getRSSI();
            String rssistring= String(rssi) + "_1" + esp_naam[8];
            if(send_to_broker){
                client.publish("esp32/afstand/rssi",rssistring.c_str());
                Serial.println(rssistring.c_str());
            }
        }
        else if (strcmp(advertisedDevice.getName().c_str(), "Afstand_2") == 0)
        {
            int rssi = advertisedDevice.getRSSI();
            String rssistring= String(rssi) + "_2" + esp_naam[8];
            if(send_to_broker){
                client.publish("esp32/afstand/rssi",rssistring.c_str());
                Serial.println(rssistring.c_str());
            }
        }
        else if (strcmp(advertisedDevice.getName().c_str(), "Afstand_3") == 0)
        {
            Serial.println(advertisedDevice.getRSSI());
            int rssi = advertisedDevice.getRSSI();
            String rssistring= String(rssi) + "_3" + esp_naam[8];
            if(send_to_broker){
                client.publish("esp32/afstand/rssi",rssistring.c_str());
                Serial.println(rssistring.c_str());
            }
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
    if (client.connect(esp_naam)) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("esp32/afstand/control");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void callback(char* topic, byte* message, unsigned int length) {
    Serial.print("Message arrived on topic: ");
    Serial.print(topic);
    Serial.print(". Message: ");
    String messageTemp;
    
    for (int i = 0; i < length; i++) {
        Serial.print((char)message[i]);
        messageTemp += (char)message[i];
    }
    Serial.println();
    Serial.println(char(message[0]));

    char test = (char)message[0];

    if (strcmp(topic, "esp32/afstand/control") == 0){
        if ('1' == test)
            ESP.restart();
        else if('2' == test)
            send_to_broker = false;
        else if ('3' == test)
            send_to_broker = true;
    }
}

void setup() {

    Serial.begin(115200);

    /*timer = timerBegin(0, 80000, true);
    //1 tick = 1ms 
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 60000, true);
    timerAlarmEnable(timer);*/  

    Serial.println("Scanning...");
    setup_wifi();
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);

    BLEDevice::init("Radiation Scan");
    pBLEScan = BLEDevice::getScan(); // create new scan
    pBLEScan->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks(),true);
    pBLEScan->setActiveScan(true); // active scan (true) uses more power, but get results faster, kheb dit zelf effe op true gezet
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99); // less or equal setInterval value

    bleCast.begin();

    if (esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV,ESP_PWR_LVL_P9) == OK) {
        Serial.println("Transmission power changed\n");
    }
    sprintf(data,"%d",9);
}

void loop() {
    //zorgen dat de connectie met de broker actief blijft
    if (!client.connected()) {
        reconnect();
    }
    client.loop();

    long now = millis();
    if (now - lastMsg > 1000) {
        lastMsg = now;
    }  

    
    /* Convert the value to a char array
    char tempString[8];
    dtostrf(69, 1, 2, tempString);*/


    Serial.println("Scanning");
    //BLEScanResults foundDevices = pBLEScan->start(scanTimeSeconds, false);
    BLEScanResults foundDevices = pBLEScan->start(5);
    pBLEScan->clearResults();
    

    if (interruptCounter > 0) {
        portENTER_CRITICAL(&timerMux);
        interruptCounter--;
        portEXIT_CRITICAL(&timerMux);
        Serial.print("An interrupt as occurred.");
    }
}