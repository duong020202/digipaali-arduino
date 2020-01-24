#include <Arduino.h>
#include "Wire.h"
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <SparkFun_I2C_GPS_Library.h> // https://github.com/sparkfunX/Qwiic_GPS-TitanX1
#include <TinyGPS++.h> // https://github.com/mikalhart/TinyGPSPlus
#include <Telaire.h>
#include <Adafruit_ADS1015.h>
#include <SoftwareSerial.h>
#include <NurMicroApi.h>

const char* ssid = "HomeAutomation";
const char* password = "TaloAutomaatio";
const char* mqtt_server = "192.168.0.110";

I2CGPS myGPS;
T9602 T9602;
TinyGPSPlus gps;
WiFiClient espClient;
PubSubClient client1(espClient);

String strReceivedMsg;
String strReceivedTopic;
String oldEPC;
String newEPC;

unsigned long startMillis;  //some global variables available anywhere in the program
unsigned long currentMillis;
const unsigned long period = 10000;  //the value is a number of milliseconds

void setup_wifi();
void callback(char* topic, byte* payload, unsigned int length);
void reconnect();

void setup_wifi() {
  delay(10);
  // Connecting to a WiFi network
  Serial.println("Connecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
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

void callback(char* topic, byte* payload, unsigned int length) {
  //Serial.print("Message arrived [");
  //Serial.print(topic);
  //Serial.print("] ");
  unsigned int i = 0;
  for (i = 0; i < length; i++) {
    strReceivedMsg.concat((char)payload[i]);
  }
  strReceivedTopic = topic;
  if (strReceivedTopic == "sensors"){
        static char result_str[1024] = "";
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.createObject();
    
        Serial.print("Sensor 40: "); T9602.showthedata(40);
        delay(5);
        json["temp1"] = roundf(T9602.temperature*100)/100;
        json["humid1"] = roundf(T9602.humidity*100)/100;

        Serial.print("Sensor 41: "); T9602.showthedata(41);
        delay(5);
        json["temp2"] = roundf(T9602.temperature*100)/100;
        json["humid2"] = roundf(T9602.humidity*100)/100;

        while (myGPS.available()) //available() returns the number of new bytes available from the GPS module
        {
           gps.encode(myGPS.read());
        }
          if (gps.location.isValid())
        {
          Serial.print("Location: ");
          Serial.print(gps.location.lat(), 6);
          Serial.print(F(", "));
          Serial.print(gps.location.lng(), 6);
          Serial.println();
        }
        else
        {
          Serial.println(F("Location not yet valid"));
        }
         
        json["long"] = gps.location.lng();
        json["lat"] = gps.location.lat();
        json.printTo(result_str);
        Serial.println(result_str);
        client1.publish("todatabase", result_str);
        delay(500);
    }
  strReceivedMsg = "";
  Serial.println();
}

void reconnect() {
  // Loop until we're reconnected
  while (!client1.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client1.connect("ESP8266Client1")) 
      {
        Serial.println("connected");
        client1.subscribe("sensors");
        startMillis = millis();  //initial start time
      }
      else {
        Serial.print("failed, rc=");
        Serial.print(client1.state());
        Serial.println(" try again in 5 seconds");
        // Wait 5 seconds before retryingco
        delay(5000);
        }
  }
}

void setup(){
  Serial.begin(38400);

  
  Wire.begin();
  if (myGPS.begin() == false)
  {
    Serial.println("Module failed to respond. Please check wiring.");
     //Freeze!
  } else Serial.println("GPS module found!");

  setup_wifi();
  client1.setServer(mqtt_server, 1883);
  client1.setCallback(callback);
}

void loop(){
  if (!client1.connected()) reconnect();
  client1.loop();        

  currentMillis = millis();  //get the current "time" (actually the number of milliseconds since the program started)
  if (currentMillis - startMillis >= period)  //test whether the period has elapsed
  {
    delay(0);
    yield();
    static char result_str[500] = "";
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
     while (myGPS.available()) //available() returns the number of new bytes available from the GPS module
    {
       gps.encode(myGPS.read());
       yield();
    }
      if (gps.location.isValid())
    {
      Serial.print("Location: ");
      Serial.print(gps.location.lat(), 6);
      Serial.print(F(", "));
      Serial.print(gps.location.lng(), 6);
      Serial.println();

    }
    else
    {
      Serial.println(F("Location not yet valid"));
    }
    
    json["long"] = gps.location.lng();
    json["lat"] = gps.location.lat();
    json.printTo(result_str);
    Serial.println(result_str);
    client1.publish("path", result_str); 
    startMillis = currentMillis;  //IMPORTANT to save the start time of the current LED state.
  }
 }
 
