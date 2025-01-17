#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESPAsyncTCP.h>
#include "LittleFS.h"

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Search for parameter in HTTP POST request
const char* PARAM_INPUT_1 = "ssid";
const char* PARAM_INPUT_2 = "pass";
const char* PARAM_INPUT_3 = "ip";
const char* PARAM_INPUT_4 = "gateway";

//Variables to save values from HTML form
String ssid;
String pass;
String ip;
String gateway;

IPAddress localIP;
//IPAddress localIP(192, 168, 1, 200); // hardcoded

// Set your Gateway IP address
IPAddress localGateway;
//IPAddress localGateway(192, 168, 1, 1); //hardcoded
IPAddress subnet(255, 255, 0, 0);

// Timer variables
unsigned long previousMillis = 0;
const long interval = 10000;  // interval to wait for Wi-Fi connection (milliseconds)

String ledState;

bool restart = false;
bool ap_running = false;

// -------------------------------------------------------------- //
// ------------------------ [ LittleFS ] ------------------------ //
// -------------------------------------------------------------- //

// File paths to save input values permanently
const char* ssidPath = "/ssid.txt";
const char* passPath = "/pass.txt";
const char* ipPath = "/ip.txt";
const char* gatewayPath = "/gateway.txt";

// Initialize LittleFS
void initFS() {
  if (!LittleFS.begin()) {
    Serial.println("An error has occurred while mounting LittleFS");
  }
  else{
    Serial.println("LittleFS mounted successfully");
  }
}

// Read File from LittleFS
String readFile(fs::FS &fs, const char * path){
  Serial.printf("Reading file: %s\r\n", path);

  File file = fs.open(path, "r");
  if(!file || file.isDirectory()){
    Serial.println("- failed to open file for reading");
    return String();
  }

  String fileContent;
  while(file.available()){
    fileContent = file.readStringUntil('\n');
    break;
  }
  file.close();
  return fileContent;
}

// Write file to LittleFS
void writeFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Writing file: %s\r\n", path);

  File file = fs.open(path, "w");
  if(!file){
    Serial.println("- failed to open file for writing");
    return;
  }
  if(file.print(message)){
    Serial.println("- file written");
  } else {
    Serial.println("- frite failed");
  }
  file.close();
}

// -------------------------------------------------------------- //
// -------------------------- [ Wifi ] -------------------------- //
// -------------------------------------------------------------- //

const int WIFI_RESET_BUTTON = 14;

enum WIFI_ERROR_CODES {
  SUCCESFULLY_CONNECTED = 0,
  UNDEFINED_SSID_OR_IP = 1,
  STA_FAILED = 2,
  FAILED_TO_CONNECT = 3,
};

// Initialize WiFi
int initWiFi() {
  if(ssid=="" || ip==""){
    Serial.println("Undefined SSID or IP address.");
    return UNDEFINED_SSID_OR_IP;
  }

  WiFi.mode(WIFI_STA);
  localIP.fromString(ip.c_str());
  localGateway.fromString(gateway.c_str());

  if (!WiFi.config(localIP, localGateway, subnet)){
    Serial.println("STA Failed to configure");
    return STA_FAILED;
  }
  WiFi.begin(ssid.c_str(), pass.c_str());

  // TODO: Podría ser una especide for y que lo haga un par de veces?
  Serial.println("Connecting to WiFi...");
  while(WiFi.status() != WL_CONNECTED) {
    Serial.println("Failed to connect. Retrying WiFi connection in 5 seconds...");
    delay(5000);
  }

  Serial.println("Wifi connected!.");
  Serial.println(WiFi.localIP());
  return SUCCESFULLY_CONNECTED;
}

void initAP(){
  // Connect to Wi-Fi network with SSID and password
  Serial.println("Setting AP (Access Point)");
  // NULL sets an open Access Point
  WiFi.softAP("ESP-WIFI-MANAGER", NULL);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP); 

  // Web Server Root URL
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(LittleFS, "/wifimanager.html", "text/html");
  });
  
  server.serveStatic("/", LittleFS, "/");
  
  server.on("/", HTTP_POST, [](AsyncWebServerRequest *request) {
    int params = request->params();
    for(int i=0; i<params; i++) {
      AsyncWebParameter* p = request->getParam(i);
      if(p->isPost()){
        // HTTP POST ssid value
        if (p->name() == PARAM_INPUT_1) {
          ssid = p->value().c_str();
          Serial.print("SSID set to: ");
          Serial.println(ssid);
          // Write file to save value
          writeFile(LittleFS, ssidPath, ssid.c_str());
        }
        // HTTP POST pass value
        if (p->name() == PARAM_INPUT_2) {
          pass = p->value().c_str();
          Serial.print("Password set to: ");
          Serial.println(pass);
          // Write file to save value
          writeFile(LittleFS, passPath, pass.c_str());
        }
        // HTTP POST ip value
        if (p->name() == PARAM_INPUT_3) {
          ip = p->value().c_str();
          Serial.print("IP Address set to: ");
          Serial.println(ip);
          // Write file to save value
          writeFile(LittleFS, ipPath, ip.c_str());
        }
        // HTTP POST gateway value
        if (p->name() == PARAM_INPUT_4) {
          gateway = p->value().c_str();
          Serial.print("Gateway set to: ");
          Serial.println(gateway);
          // Write file to save value
          writeFile(LittleFS, gatewayPath, gateway.c_str());
        }
      }
    }
    restart = true;
    request->send(200, "text/plain", "Done. ESP will restart, connect to your router and go to IP address: " + ip);
  });
  server.begin();
  ap_running = true;
}

// -------------------------------------------------------------- //
// -------------------------- [ Setup ] ------------------------- //
// -------------------------------------------------------------- //

void setup() {
  
  // Serial port for debugging purposes
  Serial.begin(9600);
  
  pinMode(WIFI_RESET_BUTTON, INPUT_PULLUP);

  initFS();

  // Load values saved in LittleFS
  ssid = readFile(LittleFS, ssidPath);
  pass = readFile(LittleFS, passPath);
  ip = readFile(LittleFS, ipPath);
  gateway = readFile (LittleFS, gatewayPath);
  Serial.println(ssid);
  Serial.println(pass);
  Serial.println(ip);
  Serial.println(gateway);

  if(initWiFi() == (UNDEFINED_SSID_OR_IP || STA_FAILED)) {
    initAP();
  }
}

// -------------------------------------------------------------- //
// -------------------------- [ Loop ] -------------------------- //
// -------------------------------------------------------------- //

void loop() {
  if (digitalRead(WIFI_RESET_BUTTON) && !ap_running){
    Serial.print("Button: ");
    Serial.println(digitalRead(WIFI_RESET_BUTTON));
    initAP();
  }

  if (restart){
    delay(5000);
    ESP.restart();
  }
}