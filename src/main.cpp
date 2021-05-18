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



//RSSI grenswaarde voor t.o.v elke speler
double waarde = -32;

int teller0 = 0;
int teller1 = 0;
int teller2 = 0;
int teller3 = 0;

//Er worden 4 circulaire buffers gebruikt met een grootte die gelijk is aan size
//In deze buffers komen de RSSI waarden die gemeten worden ten opzichte van de andere ESP's
CircBuffer buffer0;
CircBuffer buffer1;
CircBuffer buffer2;
CircBuffer buffer3;

int size = 20;

bool send_to_broker = true;

//pin om de buzzer aan te sturen
int buzzerPin = 15;

//methode voor het initialiseren van de buffers
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
//De juiste naam instellen voor de ESP voor de wifi client en de mqtt client. Deze naam moet per speler verschillen.
//De mogelijke namen zijn : "Afstand_0", "Afstand_1", "Afstand_2" en "Afstand_3"
//Het laatste cijfer van de naam is het spelernummer
const char* esp_naam = "Afstand_0";
//scan objecten die nodig zijn voor de BLE-scan
BLEScan *pBLEScan;
BLECast bleCast(esp_naam);



//variabelen om een cooldown periode te implementeren na verzenden van het alarm
//wachtijd is de minimale tijd die tussen twee overtredingen zit,
//met andere woorden er zit minimaal een tijd van "wachttijd" milliseconden tussen de keren dat een speler piept
//maxTijdTussenAlarm is de maximale tijd die tussen twee overtreding zit,
// met andere woorden er zit maximaal een tijd van "maxTijdTusselAlarm" milliseconden tussen de keren dat een speler piept  
int wachttijd = 180000;
//int maxTijdTussenAlarm = 20000000;
int cooldown = -wachttijd;
//int cooldown2 = 0;
//variabele om bij te houden of de buzzer actief is
bool piepActief = false;
//variabele om bij te houden sinds wanneer de buzzer al aan het piepen is
int beginTijdstip = 0;
//variabele die het startsein moet geven om te beginnen piepen
bool beginPiep = false;



//Parameters om verbinding te maken met de MQTT server
const char* ssid = "NETGEAR68";
const char* password = "excitedtuba713";
const char* mqtt_server = "192.168.1.2";

//Het laatste cijfer van het piepkanaal moet aangepast worden zodat dit overeenkomt met het spelernummer
//We stellen geven het WifiClient object en het PubSubClient object dezelfde naam als de esp_naam
const char* piepkanaal = "esp32/afstand/piep/0";
WiFiClient Afstand_0;
PubSubClient client(Afstand_0);




void stuurAlarm();
void piepNonBlocking();

//Callback functie die gelinkt wordt aan de BLE scan. Hier wordt de verwerking van de RSSI-waarden gedaan
class AdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
    void onResult(BLEAdvertisedDevice advertisedDevice)
    {
        beginPiep = false;
        if (strcmp(advertisedDevice.getName().c_str(), "Afstand_0") == 0){
            buffer0.put(advertisedDevice.getRSSI());
            teller0++;
            if(teller0 == size){
                teller0 =0;
                String rssistring = (String) buffer0.getAverage() + "_0" + esp_naam[8];
                Serial.print("Buffer0 gemiddelde: ");
                Serial.println(rssistring.c_str());
                //Mag er een alarm afgaan?
                //De voorwaarden zijn: er zijn geen personen hun handen aan het ontsmetten (send_to_broker == true), de wachttijd is voorbij en er is een overtreding
                if (send_to_broker && (millis() - cooldown) > wachttijd && buffer0.getAverage() > waarde){
                    Serial.println("buffer 0 alarm");
                    String s = String(esp_naam[8]) + "0";
                    Serial.println(s);
                    client.publish("esp32/ontsmetten/id", s.c_str());
                    client.publish("esp32/afstand/piep/0","1");
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
                Serial.print("Buffer1 gemiddelde: ");
                Serial.println(rssistring.c_str());

                if (send_to_broker && (millis() - cooldown) > wachttijd && buffer0.getAverage() > waarde){
                    Serial.println("buffer 1 alarm");
                    String s = String(esp_naam[8]) + "1";
                    client.publish("esp32/ontsmetten/id", s.c_str());
                    client.publish("esp32/afstand/piep/1","1");
                    beginPiep = true;
                }
            }
        }
        else if (strcmp(advertisedDevice.getName().c_str(), "Afstand_2") == 0)
        {
            int rssi = advertisedDevice.getRSSI();
            String rssistring= String(rssi) + "_2" + esp_naam[8];
            buffer2.put(advertisedDevice.getRSSI());
            teller2++;
            if( teller2 == size){
                teller2 =0;
                String rssistring = (String) buffer2.getAverage() + "_2" + esp_naam[8];
                Serial.print("Buffer2 gemiddelde: ");
                Serial.println(rssistring.c_str());   
               
                if (send_to_broker && (millis() - cooldown) > wachttijd && buffer2.getAverage() > waarde){
                    Serial.println("buffer 2 alarm");
                    String s = String(esp_naam[8]) + "2";
                    client.publish("esp32/ontsmetten/id", s.c_str());
                    client.publish("esp32/afstand/piep/2","1");
                    beginPiep = true;
                }                              
            }
        }
        else if(strcmp(advertisedDevice.getName().c_str(),"Afstand_3") == 0){
            buffer3.put(advertisedDevice.getRSSI());
            teller3++;
            if( teller3  == size){
                teller3 =0;
                String rssistring = (String) buffer3.getAverage() + "_3" + esp_naam[8];
                Serial.print("Buffer3 gemiddelde: ");
                Serial.println(rssistring.c_str());
                
                if (send_to_broker && (millis() - cooldown) > wachttijd && buffer3.getAverage() > waarde){
                    Serial.println("buffer 3 alarm");
                    String s = String(esp_naam[8]) + "3";
                    client.publish("esp32/ontsmetten/id", s.c_str());
                    client.publish("esp32/afstand/piep/3","1");
                    beginPiep = true;
                }
            }
        }
        piepNonBlocking();
    }
};

//functie om de alarmsignalen naar de andere groepen te sturen via de broker
void stuurAlarm(){
    if (send_to_broker){
        Serial.println("Alarm!");
        client.publish("esp32/afstand/control", "1");
        client.publish("esp32/ontsmetten/control","1");
        client.publish("esp32/vaccin/control","1");
        client.publish("esp32/5g/control","1");
        client.publish("esp32/morse/control","1");
        client.publish("esp32/fitness/control","1");        
    }
}

//non blocking functie om de buzzer te laten piepen
void piepNonBlocking(){  
   //Eerst controleren of er gepiept mag worden. Er moet dus een overtreding zijn en de module mag niet meer aan het piepen zijn.
    if (!piepActief && beginPiep){
        beginTijdstip = millis();
        digitalWrite(buzzerPin, HIGH);
        Serial.println("beginnen met piepen");
        stuurAlarm();
        piepActief = true;
        send_to_broker = false;
    }
    //vijf seconden piepen
    else if (millis() - beginTijdstip > 5000 && piepActief){
        digitalWrite(buzzerPin, LOW);
        piepActief = false;
        Serial.println("Stoppen met piepen");
    }
}

//De MQTT callback, verwerking van MQTT berichten op gesubscribede kanalen
void callback(char* topic, byte* message, unsigned int length) {
    Serial.print("Message arrived on topic: ");
    Serial.print(topic);
    Serial.print(". Message: ");
    String messageTemp;
    
    for (int i = 0; i < length; i++) {
        Serial.print((char)message[i]);
        messageTemp += (char)message[i];
    }

    //De verwachte berichten hebben maar een lengte van 1, enkel het nulde element van het bericht moet dus gecontroleerd worden.
    char test = (char)message[0];

    if (strcmp(topic, "esp32/afstand/control") == 0){
        if ('0' == test)
            ESP.restart();
        else if('2' == test){
            send_to_broker = true;
            cooldown = millis();
        }
        else if ('1' == test)
            send_to_broker = false;
    }
    else {
        if (strcmp(topic, piepkanaal) == 0){
            if(strcmp(topic,piepkanaal) == 0){
                if(test == '1'){
                    Serial.println("piep ontvangen");
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
    // verbinding maken met het wifi netwerk
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


void setup() {

    Serial.begin(115200);
    Serial.println("Scanning...");
    setup_wifi();
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);

    BLEDevice::init("RSSI scan");
    pBLEScan = BLEDevice::getScan(); // create new scan
    pBLEScan->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks(),true);
    pBLEScan->setActiveScan(true); // active scan (true) uses more power, but get results faster
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99); // less or equal setInterval value
    bleCast.begin();

    //De BLE power wordt op het maximum gezet
    if (esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV,ESP_PWR_LVL_P9) == OK) {
        Serial.println("Transmission power changed\n");
    }
    //initialiseren van de buffers
    initBuffers(size);

    pinMode(buzzerPin, OUTPUT);
}


//Loop
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

void loop() {
    //zorgen dat de connectie met de broker actief blijft
    if (!client.connected()) {
        reconnect();
    }
    //MQTT berichten bekijken
    client.loop();

    //BLE scannen
    Serial.println("Scanning:");
    BLEScanResults foundDevices = pBLEScan->start(1);
    pBLEScan->clearResults();
}