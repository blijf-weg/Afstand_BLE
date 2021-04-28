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
double calibratie0 = -40.23;
double calibratie1 = -40.37;
double calibratie2 = -40.27;
double calibratie3 = -42.27;
double calibratie4 = -40.27;

double meterWaarde0 = (log10(2.5) * 40) + (calibratie0);
double meterWaarde1 = (log10(2.5) * 40) + (calibratie1); 
double meterWaarde2 = (log10(2.5) * 40) + (calibratie2); 
double meterWaarde3 = -31;
//(log10(2.5) * 40) + (calibratie3);
double meterWaarde4 = -31;
//(log10(2.5) * 40) + (calibratie4);

int size = 20;
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

//pin om de buzzer aan te sturen
int buzzerPin = 15;

//array om de coordinaten bij te houden
char** coordinaten;

//variabele om een cooldown periode te implementeren na verzenden van het alarm
int wachttijd = 10000;
int cooldown = -wachttijd;


//variabele om begintijdstip van de buzzer bij te houden
bool piepActief = false;

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
const char* mqtt_server = "192.168.137.1";
//"192.168.1.2"; 
//"192.168.43.101";
//192.168.0.137
//const char* ssid = "telenet-648FE13";
//const char* password = "YF74spyvpdkp";
const char* ssid = "D84Z82H2 9418";
const char* password = "73U-229k";
const char* esp_naam = "Afstand_4";
const char* piepkanaal = "esp32/afstand/piep/4";

WiFiClient Afstand_4;
PubSubClient client(Afstand_4);

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

class AdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
    void onResult(BLEAdvertisedDevice advertisedDevice)
    {
        if (strcmp(advertisedDevice.getName().c_str(), "Afstand_0") == 0){
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

                double* coordinaten = metingen.berekenPositie();
                
                if(coordinaten != nullptr){
                    Serial.print("X (in main): ");
                    Serial.println(coordinaten[0]);
                    String s = "esp32/afstand/x" + esp_naam[8];
                    String s1 = (String) coordinaten[0];
                    client.publish(s.c_str(),s1.c_str());
                    s= "esp32/afstand/y" + esp_naam[8];
                    s1 = (String) coordinaten[1];
                    client.publish(s.c_str(),s1.c_str());
                }
            }
        }
        else if (strcmp(advertisedDevice.getName().c_str(), "Afstand_1") == 0)
        {   
            //int rssi = advertisedDevice.getRSSI();
            //buffer1 vullen met rssi waarden
            //als de buffer allemaal nieuwe waarden heeft ontvangen dan slaan we het op als rssi waarde in metingen
            //de afstand tot node 1 wordt ook direct berekend en opgeslaan in metingen
            //de teller wordt terug op nul gezet om dit proces opnieuw uit te voeren

            //als alle afstanden zijn geüpdate wordt de coordinaat van deze esp berekend
            //de x en y waarden worden doorgestuurd via het kanaal "esp32/afstand/piep/x(/y) + esp-nummer" 
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


                double* coordinaten = metingen.berekenPositie();
                
                if(coordinaten != nullptr){
                    Serial.print("X (in main): ");
                    Serial.println(coordinaten[0]);
                    String s = "esp32/afstand/x" + esp_naam[8];
                    String s1 = (String) coordinaten[0];
                    client.publish(s.c_str(),s1.c_str());
                    s= "esp32/afstand/y" + esp_naam[8];
                    s1 = (String) coordinaten[1];
                    client.publish(s.c_str(),s1.c_str());
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
                
                double* coordinaten = metingen.berekenPositie();
                if(coordinaten != nullptr){
                    Serial.print("X (in main): ");
                    Serial.println(coordinaten[0]);
                    String s = "esp32/afstand/x" + esp_naam[8];
                    String s1 = (String) coordinaten[0];
                    client.publish(s.c_str(),s1.c_str());
                    s= "esp32/afstand/y" + esp_naam[8];
                    s1 = (String) coordinaten[1];
                    client.publish(s.c_str(),s1.c_str());
                }                
            }
        }
        else if(strcmp(advertisedDevice.getName().c_str(),"Afstand_3") == 0){
            buffer3.put(advertisedDevice.getRSSI());
            teller3++;
            Serial.println(advertisedDevice.getRSSI());
            if(send_to_broker && teller3  == size){
                teller3 =0;
                String rssistring = (String) buffer3.getAverage() + "_3" + esp_naam[8];
                //client.publish("esp32/afstand/rssi",rssistring.c_str());
                Serial.print("Buffer3 gemiddelde: ");
                Serial.println(rssistring.c_str());
                
                Serial.println(meterWaarde3);
                if (buffer3.getAverage() > meterWaarde3){
                    Serial.println("buffer 3 alarm");
                    String s = esp_naam[8] + "3";
                    client.publish("esp32/ontsmetten/id", s.c_str());
                    client.publish("esp32/afstand/piep/3","1");
                    Serial.println("Alarm!!!!!!");
                    stuurAlarm();
                    piep();
                }
            }
        }
        else if (strcmp(advertisedDevice.getName().c_str(), "Afstand_4") == 0)
        {
          buffer4.put(advertisedDevice.getRSSI());
            teller4++;
            if(send_to_broker && teller4 == size){
                teller4 =0;
                String rssistring = (String) buffer4.getAverage() + "_4" + esp_naam[8];
                //client.publish("esp32/afstand/rssi",rssistring.c_str());
                Serial.print("Buffer4 gemiddelde: ");
                Serial.println(rssistring.c_str());
                
                Serial.println(meterWaarde4);
                if (buffer4.getAverage() > meterWaarde4){
                    Serial.println("buffer 4 alarm");
                    String s = esp_naam[8] + "4";
                    client.publish("esp32/ontsmetten/id", s.c_str());
                    client.publish("esp32/afstand/piep/4","1");
                    Serial.println("Alarm!!!!!!");
                    stuurAlarm();
                    piep();
                }
            }
        }
    }
};

//functie om het alarmsignaal naar de broker te sturen
void stuurAlarm(){
    if (send_to_broker && millis() - cooldown > wachttijd){
        client.publish("esp32/afstand/alarm", "1");
        cooldown = millis();
    }
}

//functie om de buzzer te laten piepen
void piep(){
        int begin = millis();
        digitalWrite(buzzerPin, HIGH);
        while(millis()- begin < 3000){
            Serial.println("aan het piepen");
            delay(500);
        }
        digitalWrite(buzzerPin, LOW);
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
    else {
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
    else {
        if (strcmp(topic, piepkanaal) == 0){
            if(strcmp(topic,piepkanaal) == 0){
                if(test == '1'){
                    piep();
                }
            }
            //piep();
        }
        //het ontvangen bericht is een coordinaat
        else{
            String onderwerp = String(topic);
            //nakijken ofdat het om een coordinaat gaat dat is binnengekomen
            if (onderwerp.charAt(onderwerp.length()-2) == 'x' || onderwerp.charAt(onderwerp.length()-2) == 'y'){
                char* coordinaat;
                for (int i = 0; i < length; i++) {
                    coordinaat[i] = (char)message[i];
                }
                //coordinaat op de juiste plek in de coordinaten array steken
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
                if(send_to_broker){
                    if (overtreders.length() != 0){
                        stuurAlarm();
                        for (int i = 0; i<overtreders.length(); i++){
                        i++;
                        client.publish("esp32/ontsmetten/id",overtreders.substring(i,i+2).c_str());
                        i++;
                        }
                    }
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
      /*volgende twee kanalen zijn een beetje dom, maar ik weet ni exact hoe de coördinaten gaan 
      bepaald worden dus dit is efkens gemakkelijk*/
      client.subscribe("esp32/afstand/x1");
      client.subscribe("esp32/afstand/y1");
      //deze zijn ni dom
      client.subscribe("esp32/afstand/x2");
      client.subscribe("esp32/afstand/y2");
      client.subscribe("esp32/afstand/x3");
      client.subscribe("esp32/afstand/y3");
      client.subscribe("esp32/afstand/x4");
      client.subscribe("esp32/afstand/y4");

      //pas deze waarde aan per verschillende module
    client.subscribe(piepkanaal);
    client.subscribe("esp32/afstand/piep/4");
    
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
    initBuffers(size);

    pinMode(buzzerPin, OUTPUT);

    Serial.print("Piepkanaal: ");
    Serial.println(piepkanaal);
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
    Serial.println("Hier");
    

    /*if (interruptCounter > 0) {
        portENTER_CRITICAL(&timerMux);
        interruptCounter--;
        portEXIT_CRITICAL(&timerMux);
        Serial.print("An interrupt has occurred.");
    }*/
}