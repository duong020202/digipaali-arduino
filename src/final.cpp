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

// NUR serial port baud rate.
// NOTE: By default NUR module is configured to 115200 baudrate.
//       If using something else baudrate (e.g. 38400) you'll need to reconfigure NUR for other baudrate using Nordic ID RFID Configurator app.
#define NUR_SERIAL_BAUDRATE   (38400)
// Print serial port baudrate. If not used, leave undefined
#define PRINT_SERIAL_BAUDRATE (38400)
// In this setup NUR is connected to software serial and print data in HW serial
#define NurSerial swSerial
#define PrintSerial Serial
#ifndef PRINT_SERIAL_BAUDRATE
  #undef PrintSerial
#endif

// The API's communication buffers. Adjust if needed
static BYTE ApiRxBuffer[256];
static BYTE ApiTxBuffer[128];

// True if NUR module detected in setup()
BOOL NurAvailable = FALSE;

// NurMicroApi handle
static struct NUR_API_HANDLE gApi =
{
  NULL, // void *UserData;
  NULL, // TransportReadDataFunction
  NULL, // TransportWriteDataFunction
  NULL, // UnsolEventHandler;

  NULL, // BYTE *TxBuffer;
  0,    // DWORD TxBufferLen;

  NULL, //BYTE *RxBuffer;
  0,    // DWORD RxBufferLen;
  0,    // DWORD RxBufferUsed;

  0,    // DWORD respLen;
  NULL  // struct NUR_CMD_RESP *resp;
};

//struct NUR_CMD_WRITE_PARAMS writeParams;

const char* ssid = "HomeAutomation";
const char* password = "TaloAutomaatio";
const char* mqtt_server = "192.168.0.110";

I2CGPS myGPS;
T9602 T9602;
TinyGPSPlus gps;
WiFiClient espClient;
PubSubClient client1(espClient);
SoftwareSerial swSerial(D3, D4); // RX, TX

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

Adafruit_ADS1115 ads(0x48); // ads1115's i2c id when addr pin is grounded
float Voltage0, Voltage1, proximity, dryMatter;
String epc = "";

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

        int16_t adc0;  // we read from the ADC, we have a sixteen bit integer as a result
        int16_t adc1; 
        adc0 = ads.readADC_SingleEnded(0); //proximity
        adc1 = ads.readADC_SingleEnded(1); //dry matter
        Voltage0 = (adc0 * 0.1875)/1000;
        Voltage1 = (adc1 * 0.1875)/1000;
        proximity = Voltage0*100/5;
        dryMatter = roundf(Voltage1*70/5*100)/100;
        json["dryMatter"] = dryMatter;
        //json["proximity"] = proximity;
        Serial.println(dryMatter);
        Serial.println(proximity);
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
  if (strReceivedTopic == "changeEPC"){
    for (i = 2; i < 26; i++) {
      oldEPC.concat((char)payload[i]);
    }
    for (i = 29; i < 53; i++) {
      newEPC.concat((char)payload[i]);
    }
    Serial.println(oldEPC);
    Serial.println(newEPC);
    //int rc = NurApiWriteTag(&gApi,  &writeParams);
  }
  strReceivedMsg = "";
  oldEPC = "";
  newEPC = "";
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
        client1.subscribe("changeEPC");
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

// Read buffer from NUR serial
// If no data available, this function should take 500us - 1000us
int nur_serial_read(struct NUR_API_HANDLE *hNurApi, BYTE *buffer, DWORD bufferLen, DWORD *bytesRead)
{
  DWORD dwRead = 0;
  DWORD retryCount = 200;

  // Wait for data
  while (dwRead == 0 && retryCount-- > 0)
  {
    // Read all data available
    while (NurSerial.available()) {
      buffer[dwRead++] = NurSerial.read();
      if (dwRead == bufferLen)
        break;
    }
  }

  if (dwRead == 0) {
    return NUR_ERROR_TR_TIMEOUT;
  }

  *bytesRead = dwRead;

  return NUR_SUCCESS;
}

// Write buffer to NUR serial
int nur_serial_write(struct NUR_API_HANDLE *hNurApi, BYTE *buffer, DWORD bufferLen, DWORD *bytesWritten)
{
  DWORD dwWritten = 0;

  while (dwWritten < bufferLen)
  {
    NurSerial.write(buffer[dwWritten++]);
  }

  *bytesWritten = dwWritten;
  return NUR_SUCCESS;
}

// Init Nur api buffers and transport
void nur_init_handle(struct NUR_API_HANDLE *hApi)
{
  // Init RX buffer
  hApi->RxBuffer = ApiRxBuffer;
  hApi->RxBufferLen = sizeof(ApiRxBuffer);

  // Init TX buffer
  hApi->TxBuffer = ApiTxBuffer;
  hApi->TxBufferLen = sizeof(ApiTxBuffer);

  // Init transport functions
  hApi->TransportReadDataFunction = nur_serial_read;
  hApi->TransportWriteDataFunction = nur_serial_write;
}

// Print NUR module mode (A = app, B = bootloader) and versions
static void nur_print_versions()
{
#ifdef PrintSerial
  struct NUR_CMD_VERSION_RESP *vr;
  int rc = NurApiGetVersions(&gApi);
  vr = &gApi.resp->versions;

  if (rc == NUR_SUCCESS) {
    PrintSerial.print(F("Versions, mode "));
    PrintSerial.print((char)vr->mode);
    PrintSerial.println("");

    PrintSerial.print(F(" - primary   : "));
    PrintSerial.print(vr->vMajor, DEC);
    PrintSerial.print(F("."));
    PrintSerial.print(vr->vMinor, DEC);
    PrintSerial.print(F("-"));
    PrintSerial.print((char)vr->vBuild);
    PrintSerial.println("");

    PrintSerial.print(F(" - secondary : "));
    PrintSerial.print(vr->otherMajor, DEC);
    PrintSerial.print(F("."));
    PrintSerial.print(vr->otherMinor, DEC);
    PrintSerial.print(F("-"));
    PrintSerial.print((char)vr->otherBuild);
    PrintSerial.println("");
  }
  else {
    PrintSerial.print(F("Version error: "));
    PrintSerial.print(rc, DEC);
    PrintSerial.println("");
  }
#endif
}

// Configure NUR module
static void nur_configure_module()
{
  int rc = NUR_SUCCESS;
  struct NUR_CMD_LOADSETUP_PARAMS params;
  // Flag settings that you want to chage
  params.flags = NUR_SETUP_TXLEVEL | NUR_SETUP_ANTMASK | NUR_SETUP_SELECTEDANT;
  // Set TxLevel to maximum (500mW/1000mW depending from the module)
  params.txLevel = 0;
  // Enable antenna 0.
  // Use bit operation if you want to enable multiple
  // antennas like antenna 0 and 1 (NUR_ANTENNAMASK_1 | NUR_ANTENNAMASK_2)
  params.antennaMask = NUR_ANTENNAMASK_3 | NUR_ANTENNAID_4;
  // Set antenna selection to auto mode
  params.selectedAntenna = NUR_ANTENNAID_AUTOSELECT;
#ifdef PrintSerial
  PrintSerial.print(F("Configure NUR module"));
#endif
  // Set new module setti
  rc = NurApiSetModuleSetup(&gApi, &params);
  if (rc == NUR_SUCCESS) {
#ifdef PrintSerial
    PrintSerial.print(F("OK"));
#endif
  }
  else
  {
#ifdef PrintSerial
    PrintSerial.print(F("SetModuleSetup error. Code = "));
    PrintSerial.print(rc, DEC);
    PrintSerial.println("");
#endif
  }
}

#ifdef PrintSerial
void print_hex(int val) {
  char tmp[3];
  sprintf(tmp, "%02X", val);
  PrintSerial.print(tmp);
  epc = epc + tmp;
}
#endif

// This function is called for each inventoried tag, see NurApiFetchTagAt in nur_tag_inventory()
int nur_fetch_tags_function(struct NUR_API_HANDLE *hNurApi, struct NUR_IDBUFFER_ENTRY *tag)
{
#ifdef PrintSerial
  int n;
  PrintSerial.print(F("Antenna "));
  PrintSerial.print(tag->antennaId, DEC);
  PrintSerial.print(F(" RSSI "));
  PrintSerial.print(tag->rssi, DEC);
  PrintSerial.print(F(" ("));
  PrintSerial.print(tag->scaledRssi, DEC);
  PrintSerial.print(F("%) EPC: "));

  static char result_str[1024] = "";
  DynamicJsonBuffer jsonBuffer;
  JsonObject& json = jsonBuffer.createObject();
  json["rssi"] = tag->scaledRssi;
  
  for (n = 0; n < tag->epcLen; n++) {
    print_hex(tag->epcData[n]);
    //epc = epc + tag->epcData[n];
  }
  json["id"] = epc;
  json.printTo(result_str);
  //Serial.println(result_str);
  client1.publish("nurapisample/epc", result_str);
  epc = "";
  Serial.println();
#endif
  return NUR_SUCCESS; // non-zero terminates tag buffer parsing
}

// Perform tag inventory
// 1. Clear tag buffer
// 2. Perform inventory
// 3. Fetch tags
void nur_tag_inventory()
{
  int rc;

#ifdef PrintSerial
  PrintSerial.println("Inventory begin");
#endif

  // Clear tag buffer
  yield();
  rc = NurApiClearTags(&gApi);

  if (rc == NUR_SUCCESS)
  {
    // Perform tag inventory
    rc = NurApiInventory(&gApi, NULL); // Pass NULL as params, uses default inventory settings from module setup
    yield();
    if (rc == NUR_SUCCESS)
    {
      // Fetch tags one by one
      int tagCount, n;
      tagCount = gApi.resp->inventory.numTagsMem;
      for (n = 0; n < tagCount; n++)
      {
        rc = NurApiFetchTagAt(&gApi, TRUE, n, nur_fetch_tags_function);
        delay(0);
        if (rc != NUR_SUCCESS) {
          break;
        }
      }
#ifdef PrintSerial
      PrintSerial.println(F("Inventory done"));
      yield();
#endif
    }
    else {
#ifdef PrintSerial
      PrintSerial.print(F("Inventory error: "));
      PrintSerial.print(rc, DEC);
      PrintSerial.println("");
#endif
    }
  }
  else
  {
#ifdef PrintSerial
    PrintSerial.print(F("ClearTags error: "));
    PrintSerial.print(rc, DEC);
    PrintSerial.println("");
#endif
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
  ads.begin();

  /* writeParams.flags = 0;
  writeParams.passwd = 0;*/

  void enableIntTx(bool on);
  // Disable or enable interrupts on the rx pin
  void enableRx(bool on);
  // One wire control
  void enableTx(bool on);

#ifdef PrintSerial
  // Open the print serial port
  PrintSerial.begin(PRINT_SERIAL_BAUDRATE);
  PrintSerial.println(F("Start"));
#endif

  // Open the NUR serial port
  NurSerial.begin(NUR_SERIAL_BAUDRATE);
  // Initialize NUR handle
  nur_init_handle(&gApi);

  // Send ping to NUR to test availability
  if (NurApiPing(&gApi) == NUR_SUCCESS) {
    // Got OK response
    NurAvailable = TRUE;
#ifdef PrintSerial
    PrintSerial.println(F("NUR DETECTED"));
#endif
    nur_print_versions();
  }
  else {
    // No response or invalid response
#ifdef PrintSerial
    PrintSerial.println(F("NUR NOT DETECTED"));
#endif
  }
  setup_wifi();
  client1.setServer(mqtt_server, 1883);
  client1.setCallback(callback);
}

void loop(){
  if (!client1.connected()) reconnect();
  client1.loop();

  if (!NurAvailable)
  {
    /*digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(100);                       // wait for a 100ms
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(100); */                      // wait for a 100ms
    #ifdef PrintSerial
    PrintSerial.println(F("ERROR"));
    yield();
    #endif
    return;
  }

  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  nur_tag_inventory();               // perform tag inventory
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(100);          

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
    client1.publish("outTopic3", result_str); 
    startMillis = currentMillis;  //IMPORTANT to save the start time of the current LED state.
  }
 }
 
