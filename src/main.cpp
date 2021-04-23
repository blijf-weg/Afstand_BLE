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
#include <circ-buffer.h>
#include <Metingen.h>
#include <math.h>

using namespace std;

volatile int interruptCounter;

Metingen metingen;
double calibratie0 = -42.53;
double calibratie1 = -45.07;
double calibratie2 = -46.93;

double meterWaarde0 = (log10(2.5) * 40) + (calibratie0);
double meterWaarde1 = (log10(2.5) * 40) + (calibratie1); 
double meterWaarde2 = (log10(2.5) * 40) + (calibratie2); 

int size = 30;
int teller0 = 0;
int teller1 = 0;
int teller2=0;
int teller3=0;
int teller4 = 0;

CircBuffer buffer0;
CircBuffer buffer1;
CircBuffer buffer2;
CircBuffer buffer3;
CircBuffer buffer4;

//array om de coordinaten bij te houden
char** coordinaten;

CircBufferStatus_t initBuffers(uint8_t size){
    CircBufferStatus_t status = buffer0.init(size);
    status = buffer1.init(size);
    status = buffer2.init(size);
    status = buffer3.init(size);
    status = buffer4.init(size);
    if(status != CB_SUCCESS){
		return status;
	}
    return CB_SUCCESS;
}

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
//const char* mqtt_server = "192.168.137.1";
const char* mqtt_server = "192.168.43.101";
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

int teller=0;
uint8_t cnt = 0;
char data[5];

bool send_to_broker = true;

class AdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
    void onResult(BLEAdvertisedDevice advertisedDevice)
    {
        if (strcmp(advertisedDevice.getName().c_str(), "Afstand_0") == 0)
        {
            //int rssi = advertisedDevice.getRSSI();
            //String rssistring= String(rssi) + "_0" + esp_naam[8];
            //Serial.println(rssistring);
            buffer0.put(advertisedDevice.getRSSI());
            teller0++;
            if(send_to_broker && teller0 == size){
                teller0 =0;
                String rssistring = (String) buffer0.getAverage() + "_0" + esp_naam[8];
                client.publish("esp32/afstand/rssi",rssistring.c_str());
                Serial.print("Buffer0 gemiddelde: ");
                Serial.println(rssistring.c_str());
                metingen.addRSSI(buffer0.getAverage(),1);
                double afstand = pow(10, ( meterWaarde0 - buffer0.getAverage())/(10*4));
                metingen.addAfstand(afstand, 1);
            }
        }
        else if (strcmp(advertisedDevice.getName().c_str(), "Afstand_1") == 0)
        {
            //int rssi = advertisedDevice.getRSSI();
            buffer1.put(advertisedDevice.getRSSI());
            teller1++;
            if(send_to_broker && teller1 == size){
                teller1 = 0;

                String rssistring= String(buffer1.getAverage()) + "_1" + esp_naam[8];
                client.publish("esp32/afstand/rssi",rssistring.c_str());
                Serial.print("Buffer1 gemiddelde: ");
                Serial.println(rssistring.c_str());
                metingen.addRSSI(buffer1.getAverage(),2);
                double afstand = pow(10, (meterWaarde1 - buffer1.getAverage())/(10*4));
                metingen.addAfstand(afstand, 2);
            }

        }
        else if (strcmp(advertisedDevice.getName().c_str(), "Afstand_2") == 0)
        {
            int rssi = advertisedDevice.getRSSI();
            String rssistring= String(rssi) + "_2" + esp_naam[8];
            //Serial.println(rssistring.c_str());
            buffer2.put(advertisedDevice.getRSSI());
            teller2++;
            if(send_to_broker && teller2 == size){
                teller2 =0;
                String rssistring = (String) buffer2.getAverage() + "_2" + esp_naam[8];
                client.publish("esp32/afstand/rssi",rssistring.c_str());
                client.publish("esp32/afstand/rssi",rssistring.c_str());
                Serial.print("Buffer2 gemiddelde: ");
                Serial.println(rssistring.c_str());
                metingen.addRSSI(buffer2.getAverage(),0);
                double afstand = pow(10, (meterWaarde2 - buffer2.getAverage())/(10*4));
                metingen.addAfstand(afstand, 0);
            }
        }
        /*else if (strcmp(advertisedDevice.getName().c_str(), "Afstand_4") == 0)
        {
          buffer4.put(advertisedDevice.getRSSI());
            teller4++;
            if(send_to_broker && teller4 == size){
                teller4 =0;
                String rssistring = (String) buffer4.getAverage() + "_4" + esp_naam[8];
                client.publish("esp32/afstand/rssi",rssistring.c_str());
                Serial.print("Buffer4 gemiddelde: ");
                Serial.println(rssistring.c_str());
                metingen.addRSSI(buffer4.getAverage(),3);
                double afstand = pow(10, ( meterWaarde0 - buffer4.getAverage())/(10*4));
                metingen.addAfstand(afstand, 3);
            }
        }*/
    }
};

bool bepaalCentraal(double* p1, double* p2, double* p3, double* p4, double limiet){
    double x12 = abs(p1[0]-p2[0]);
    double y12 = abs(p1[1]-p2[1]);
    double l12 = sqrt(y12*y12+x12*x12);
    cout << l12 << endl;
    double x13 = abs(p3[0]-p1[0]);
    double y13 = abs(p3[1]-p1[1]);
    double l13 = sqrt(x13*x13+y13*y13);
    cout << l13 << endl;
    double x14 = abs(p4[0]-p1[0]);
    double y14 = abs(p4[1] - p1[0]);
    double l14 = sqrt(x14*x14 + y14*y14);
    cout << l14 << endl;
    return !(l12 < limiet || l13 < limiet || l14 < limiet);
}

bool bepaalTweede(double* p2, double* p3, double* p4, double limiet){
    double x23 = abs(p2[0]-p3[0]);
    double y23 = abs(p2[1]-p3[1]);
    double l23 = sqrt(x23*x23 + y23*y23);
    cout << l23 << endl;
    double x24 = abs(p2[0]-p4[0]);
    double y24 = abs(p2[1]-p4[1]);
    double l24 = sqrt(x24*x24 + y24*y24);
    cout << l24 << endl;
    return !(l23 < limiet || l24 < limiet);

}

bool bepaalDerde(double* p3, double* p4, double limiet){
    double x34 = abs(p3[0]-p4[0]);
    double y34 = abs(p3[1]-p4[1]);
    double l34 = sqrt(x34*x34 + y34*y34);
    cout << l34 << endl;
    return l34 > limiet;
}

bool checkAfstanden(double* p1, double* p2, double* p3, double* p4, double limiet){
    bool result1 = bepaalCentraal(p1, p2, p3, p4, limiet);
    cout << result1 << endl;
    bool result2 = bepaalTweede(p2, p3, p4, limiet);
    cout << result2 << endl;
    bool result3 = bepaalDerde(p3, p4, limiet);
    cout << result3 << endl;
    return (result1 && result2 && result3);
}

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
      client.subscribe("esp32/afstand/x1");
      client.subscribe("esp32/afstand/y1");
      client.subscribe("esp32/afstand/x2");
      client.subscribe("esp32/afstand/y2");
      client.subscribe("esp32/afstand/x3");
      client.subscribe("esp32/afstand/y3");
      client.subscribe("esp32/afstand/x4");
      client.subscribe("esp32/afstand/y4");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      ESP.restart();
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

bool controleerAfstand(char* x1, char* y1, char* x2, char* y2, double limiet){

    //coordinaten die aankomen onder de vorm van char* naar float omzetten
    float xfloat1 = String(x1).toFloat();
    float yfloat1 = String(y1).toFloat();
    float xfloat2 = String(x2).toFloat();
    float yfloat2 = String(y2).toFloat();

    //afstanden berekenen
    float x12 = abs(xfloat1-xfloat2);
    float y12 = abs(yfloat1-yfloat2);
    float l12 = sqrt(x12*x12+y12*y12);

    //limiet controleren
    if(l12 < limiet){
      return true;
    }
    else{
      return false;
    }
}

String bepaalAfstanden(char** punten, int lengte, double limiet){

    String inbreuken = "";
    //alle afstanden tegenover punt 1 controleren
    for (int i = 2; i<lengte; i++){
      if(controleerAfstand(punten[0], punten[1], punten[i], punten[i+1], limiet)){
        inbreuken = inbreuken + "," + "1" + String(i/2 + 1);
      }
      i++;
    }
    //alle afstanden tegenover punt 2 controleren
    for (int i = 4; i<lengte; i++){
      if(controleerAfstand(punten[2], punten[3], punten[i], punten[i+1], limiet)){
        inbreuken = inbreuken + "," + "2" + String(i/2 + 1);
      }
      i++;
    }
    //afstand tegenover punt 3 controleren
    if(controleerAfstand(punten[4], punten[5], punten[6], punten[7], limiet)){
        inbreuken = inbreuken + "," + "3" + "4";
      }

    return inbreuken;
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
    else{
        String onderwerp = String(topic);
        //nakijken ofdat het om een coordinaat gaat dat is binnengekomen
        if (onderwerp.charAt(onderwerp.length()-2) == 'x' || onderwerp.charAt(onderwerp.length()-2) == 'y'){
            char* coordinaat;
            for (int i = 0; i < length; i++) {
                coordinaat[i] = (char)message[i];
            }
        if (onderwerp.charAt(onderwerp.length()-2) == 'x'){
            if(onderwerp.charAt(onderwerp.length()-1) == '1'){
                coordinaten[0] = coordinaat;
            }
            if(onderwerp.charAt(onderwerp.length()-1) == '2'){
                coordinaten[2] = coordinaat;
            }
            if(onderwerp.charAt(onderwerp.length()-1) == '3'){
                coordinaten[4] = coordinaat;
            }
            if(onderwerp.charAt(onderwerp.length()-1) == '4'){
                coordinaten[6] = coordinaat;
            }
        }
        if (onderwerp.charAt(onderwerp.length()-2) == 'y'){
            if(onderwerp.charAt(onderwerp.length()-1) == '1'){
                coordinaten[1] = coordinaat;
            }
            if(onderwerp.charAt(onderwerp.length()-1) == '2'){
                coordinaten[3] = coordinaat;
            }
            if(onderwerp.charAt(onderwerp.length()-1) == '3'){
                coordinaten[5] = coordinaat;
            }
            if(onderwerp.charAt(onderwerp.length()-1) == '4'){
                coordinaten[7] = coordinaat;
            }
        }
        //nakijken of er overtreders zijn of niet
        String overtreders = bepaalAfstanden(coordinaten, 8, 1.5);
        //resultaten versturen naar de broker
        if (overtreders.length() != 0){
            for (int i = 0; i<overtreders.length(); i++){
            i++;
            client.publish("esp32/ontsmetten/id",overtreders.substring(i,i+2).c_str());
            i++;
            }
        }
    }
        
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
    Serial.println("allej werk");
    setup_wifi();
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);

    BLEDevice::init("Radiation Scan");
    pBLEScan = BLEDevice::getScan(); // create new scan
    pBLEScan->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks(),true);
    pBLEScan->setActiveScan(true); // active scan (true) uses more power, but get results faster, kheb dit zelf effe op true gezet
    pBLEScan->setInterval(40);
    pBLEScan->setWindow(39); // less or equal setInterval value

    bleCast.begin();

    if (esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV,ESP_PWR_LVL_P9) == OK) {
        Serial.println("Transmission power changed\n");
    }
    sprintf(data,"%d",9);
    initBuffers(size);

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
    BLEScanResults foundDevices = pBLEScan->start(0);
    pBLEScan->clearResults();
    

    if (interruptCounter > 0) {
        portENTER_CRITICAL(&timerMux);
        interruptCounter--;
        portEXIT_CRITICAL(&timerMux);
        Serial.print("An interrupt has occurred.");
    }
}