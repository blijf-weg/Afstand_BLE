#include "Arduino.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLECast.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <iostream>
#include <circ-buffer.h>

using namespace std;

volatile int interruptCounter;

double calibratie0 = -40.23;
double calibratie1 = -40.37;
double calibratie2 = -40.27;
double calibratie3 = -42.27;
double calibratie4 = -40.27;

double meterWaarde0 = -31;
//(log10(2.5) * 40) + (calibratie0);
double meterWaarde1 = -31;
//(log10(2.5) * 40) + (calibratie1); 
double meterWaarde2 = -31;
//(log10(2.5) * 40) + (calibratie2); 
double meterWaarde3 = -31;

int size = 20;
int teller0 = 0;
int teller1 = 0;
int teller2=0;
int teller3=0;

CircBuffer buffer0;
CircBuffer buffer1;
CircBuffer buffer2;
CircBuffer buffer3;

//pin om de buzzer aan te sturen
int buzzerPin = 15;

//variabele om een cooldown periode te implementeren na verzenden van het alarm
int wachttijd = 10000;
int cooldown = -wachttijd;
int cooldown2 = 0;
int maxTijdTussenAlarm = 20000;


//variabele om bij te houden of de buzzer actief is
bool piepActief = false;
//variabele om bij te houden sinds wanneer de buzzer al aan het piepen is
int beginTijdstip = 0;
//variabele die het startsein moet geven om te beginnen piepen
bool beginPiep = false;

CircBufferStatus_t initBuffers(uint8_t size){
    CircBufferStatus_t status = buffer0.init(size);
    status = buffer1.init(size);
    status = buffer2.init(size);
    status = buffer3.init(size);
    if(status != CB_SUCCESS){
		return status;
	}
    return CB_SUCCESS;
}


//const char* mqtt_server = "192.168.137.1";
const char* mqtt_server = "192.168.137.1";
//"192.168.1.2"; 
//"192.168.43.101";
//192.168.0.137
//const char* ssid = "telenet-648FE13";
//const char* password = "YF74spyvpdkp";
const char* ssid = "D84Z82H2 9418";
const char* password = "73U-229k";
const char* esp_naam = "Afstand_1";
const char* piepkanaal = "esp32/afstand/piep/1";

WiFiClient Afstand_1;
PubSubClient client(Afstand_1);

BLEScan *pBLEScan;
BLECast bleCast(esp_naam);

long lastMsg = 0;
char msg[50];
int value = 0;

int teller=0;
uint8_t cnt = 0;
char data[5];

bool send_to_broker = true;

void stuurAlarm();
void piep();
void piepNonBlocking();

class AdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
    void onResult(BLEAdvertisedDevice advertisedDevice)
    {
        beginPiep = false;
        if (strcmp(advertisedDevice.getName().c_str(), "Afstand_0") == 0){
            //int rssi = advertisedDevice.getRSSI();
            //String rssistring= String(rssi) + "_0" + esp_naam[8];
            //Serial.println(rssistring);
            buffer0.put(advertisedDevice.getRSSI());
            teller0++;
            if(teller0 == size){
                teller0 =0;
                String rssistring = (String) buffer0.getAverage() + "_0" + esp_naam[8];
                client.publish("esp32/afstand/rssi",rssistring.c_str());
                Serial.print("Buffer0 gemiddelde: ");
                Serial.println(rssistring.c_str());

                Serial.println(meterWaarde0);
                if (send_to_broker && buffer0.getAverage() > meterWaarde0){
                    Serial.println("buffer 0 alarm");
                    String s = esp_naam[8] + "0";
                    client.publish("esp32/ontsmetten/id", s.c_str());
                    client.publish("esp32/afstand/piep/0","1");
                    //stuurAlarm();
                    beginPiep = true;
                }
            }
        }
        else if (strcmp(advertisedDevice.getName().c_str(), "Afstand_1") == 0)
        {    
            buffer1.put(advertisedDevice.getRSSI());
            teller1++;
            if(teller1 == size){
                teller1 = 0;
                String rssistring= String(buffer1.getAverage()) + "_1" + esp_naam[8];
                client.publish("esp32/afstand/rssi",rssistring.c_str());
                Serial.print("Buffer1 gemiddelde: ");
                Serial.println(rssistring.c_str());

                Serial.println(meterWaarde1);
                if (send_to_broker && buffer0.getAverage() > meterWaarde1){
                    Serial.println("buffer 1 alarm");
                    String s = esp_naam[8] + "1";
                    client.publish("esp32/ontsmetten/id", s.c_str());
                    client.publish("esp32/afstand/piep/1","1");
                    //stuurAlarm();
                    beginPiep = true;
                }
            }
        }
        else if (strcmp(advertisedDevice.getName().c_str(), "Afstand_2") == 0)
        {
            int rssi = advertisedDevice.getRSSI();
            String rssistring= String(rssi) + "_2" + esp_naam[8];
            //Serial.println(rssistring.c_str());
            buffer2.put(advertisedDevice.getRSSI());
            teller2++;
            if( teller2 == size){
                teller2 =0;
                String rssistring = (String) buffer2.getAverage() + "_2" + esp_naam[8];
                client.publish("esp32/afstand/rssi",rssistring.c_str());
                client.publish("esp32/afstand/rssi",rssistring.c_str());
                Serial.print("Buffer2 gemiddelde: ");
                Serial.println(rssistring.c_str());   
               
                Serial.println(meterWaarde2);
                if (send_to_broker && buffer2.getAverage() > meterWaarde2){
                    Serial.println("buffer 2 alarm");
                    String s = esp_naam[8] + "2";
                    client.publish("esp32/ontsmetten/id", s.c_str());
                    client.publish("esp32/afstand/piep/2","1");
                    //stuurAlarm();
                    beginPiep = true;
                }                              
            }
        }
        else if(strcmp(advertisedDevice.getName().c_str(),"Afstand_3") == 0){
            buffer3.put(advertisedDevice.getRSSI());
            teller3++;
            //Serial.println(advertisedDevice.getRSSI());
            if( teller3  == size){
                teller3 =0;
                String rssistring = (String) buffer3.getAverage() + "_3" + esp_naam[8];
                //client.publish("esp32/afstand/rssi",rssistring.c_str());
                Serial.print("Buffer3 gemiddelde: ");
                Serial.println(rssistring.c_str());
                
                Serial.println(meterWaarde3);
                if (send_to_broker && buffer3.getAverage() > meterWaarde3){
                    Serial.println("buffer 3 alarm");
                    String s = esp_naam[8] + "3";
                    client.publish("esp32/ontsmetten/id", s.c_str());
                    client.publish("esp32/afstand/piep/3","1");
                    //stuurAlarm();
                    beginPiep = true;
                }
            }
        }
        piepNonBlocking();
    }
};

//functie om het alarmsignaal naar de broker te sturen
void stuurAlarm(){
    if (send_to_broker && (millis() - cooldown) > wachttijd){
        Serial.println("Alarm!!!!!!");
        client.publish("esp32/afstand/alarm", "1");
        String s =(String) esp_naam[8] + esp_naam[8];
        client.publish("esp32/ontsmetten/id",s.c_str());
    }
    else{
        if(send_to_broker && (millis() - cooldown2) > maxTijdTussenAlarm){
            Serial.println("Alarm!!!!!!");
            client.publish("esp32/afstand/alarm", "1");    
        }
    }
}

//non blocking functie om de buzzer te laten piepen
void piepNonBlocking(){
    if((millis() - cooldown2) > maxTijdTussenAlarm){
        beginPiep = true;
    }
    if (!piepActief && beginPiep && (millis() - cooldown) > wachttijd){
        beginTijdstip = millis();
        digitalWrite(buzzerPin, HIGH);
        Serial.println("beginnen met piepen");
        stuurAlarm();
        piepActief = true;
        send_to_broker = false;
    }
    else if (millis() - beginTijdstip > 10000 && piepActief){
        digitalWrite(buzzerPin, LOW);
        piepActief = false;
        send_to_broker = true;
        Serial.println("Stoppen met piepen");
        cooldown = millis();
        cooldown2 = millis();
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

    char test = (char)message[0];

    if (strcmp(topic, "esp32/afstand/control") == 0){
        if ('1' == test)
            ESP.restart();
        else if('2' == test)
            send_to_broker = false;
        else if ('3' == test)
            send_to_broker = true;
    }
    else {
        if (strcmp(topic, piepkanaal) == 0){
            if(strcmp(topic,piepkanaal) == 0){
                if(test == '1'){
                    beginPiep = true;
                    send_to_broker = false;
                    piepNonBlocking();
                    beginPiep = false;
                }
            }
        }
    }
}





//Setup

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
        client.subscribe(piepkanaal);
    }
    else {
        Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.println(" try again in 5 seconds");
        ESP.restart();
        // Wait 5 seconds before retrying
        delay(5000);
    }
  }
}
void setup() {

    Serial.begin(115200);
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
    initBuffers(size);

    pinMode(buzzerPin, OUTPUT);

    Serial.print("Piepkanaal: ");
    Serial.println(piepkanaal);

    Serial.println(cooldown);
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

    //BLEScanResults foundDevices = pBLEScan->start(scanTimeSeconds, false);
    Serial.println("Scanning");
    BLEScanResults foundDevices = pBLEScan->start(1);
    pBLEScan->clearResults();
}