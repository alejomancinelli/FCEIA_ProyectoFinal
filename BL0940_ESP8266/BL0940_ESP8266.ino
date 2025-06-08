#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESPAsyncTCP.h>
#include "LittleFS.h"
#include "ArduinoJson.h"
#include <PubSubClient.h>
#include <WiFiUdp.h>
#include <NTPClient.h>

#include "circularBuffer.hpp"
#include "BL0940_SPI.hpp"

typedef struct {
  const char* ssid;
  const char* password;
  bool staticIpEnable;
  const char* staticIp;
  const char* gateway;
  bool customMqttBroker;
  const char* mqttBroker;
  int mqttPort;
} energyMeterConfigParams;

energyMeterConfigParams energyMeterConfig;

bool apRunning = false;

BL0940_SPI bl0940(SS);

// -------------------------------------------------------------- //
// ------------------------ [ LittleFS ] ------------------------ //
// -------------------------------------------------------------- //

/**
 * @brief Initialize LittleFS
 */
void initFS() 
{
  Serial.println("Mounting LittleFS. Please wait...");
  
  if (!LittleFS.begin()) {
    Serial.println("An error has occurred while mounting LittleFS");
  }
  else{
    Serial.println("LittleFS mounted successfully");
  }
}

// -------------------------------------------------------------- //
// -------------------------- [ Json ] -------------------------- //
// -------------------------------------------------------------- //

// JSON file path
const char* configPath = "/config.json";

/**
 * @brief Saves Wifi config data into JSON file
 * 
 * @param config 
 */
void saveConfig(const energyMeterConfigParams& config) 
{
  DynamicJsonDocument doc(256);

  doc["ssid"] = config.ssid;
  doc["password"] = config.password;
  doc["static_ip_enable"] = config.staticIpEnable;
  doc["static_ip"] = config.staticIp;
  doc["gateway"] = config.gateway;
  doc["custom_mqtt_broker"] = config.customMqttBroker;
  doc["mqtt_broker"] = config.mqttBroker;
  doc["mqtt_port"] = config.mqttPort;

  File file = LittleFS.open(configPath, "w");
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }

  serializeJson(doc, file);
  
  file.close();
  Serial.println("Wifi config saved");
}

/**
 * @brief Loads Wifi config data fron JSON file
 * 
 * @param config 
 */
void loadConfig(energyMeterConfigParams& config) 
{
  if (!LittleFS.exists(configPath)) {
    Serial.println("File doesn't exist");
    return;
  }

  File file = LittleFS.open(configPath, "r");
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  DynamicJsonDocument doc(256);
  DeserializationError error = deserializeJson(doc, file);
  if (error) {
    Serial.println("JSON deserialized error");
    return;
  }

  // Prints saved data
  serializeJson(doc, Serial);
  Serial.println();

  config.ssid = strdup(doc["ssid"] | "");
  config.password = strdup(doc["password"] | "");
  config.staticIpEnable = doc["static_ip_enable"] | false;
  config.staticIp = strdup(doc["static_ip"] | "");
  config.gateway = strdup(doc["gateway"] | "");
  config.customMqttBroker = doc["customMqttBroker"] | false;
  config.mqttBroker = strdup(doc["mqtt_broker"] | "");
  config.mqttPort = doc["mqtt_port"] | 0;

  file.close();
}

// -------------------------------------------------------------- //
// -------------------------- [ NTP ] --------------------------- //
// -------------------------------------------------------------- //

WiFiUDP ntpUDP;
// Configuración del cliente NTP para la Argentina (UTC-3)
NTPClient timeClient(ntpUDP, "pool.ntp.org", -3 * 3600, 60000);  // Último parámetro: actualización cada 60 seg

time_t currentTime;                           // Hora en formato UNIX
unsigned long lastMillisSync;                 // Último tiempo de sincronización con millis()
unsigned long lastNTPUpdate = 0;              // Última sincronización con NTP
const unsigned long syncInterval = 3600000;   // 1 hora en milisegundos

long prevNtpRetryTime = 0;
#define NTP_RETRY_TIME 2000

bool ntpStarted = false;
bool ntpSync = false;

/**
 * @brief Controls and updates system time every X time
 */
void timeControl(void)
{
  // Sincronizar con NTP solo si ha pasado syncInterval
  if (WiFi.status() == WL_CONNECTED && millis() - lastNTPUpdate > syncInterval) {
    timeClient.update();
    updateLocalTime();  // Actualiza la hora con NTP
    lastNTPUpdate = millis();  // Guarda el tiempo de la última actualización
  } 
}

/**
 * @brief Updates local time with the NTP information
 */
bool updateLocalTime(void) 
{
  if (!timeClient.update()) {
    Serial.println("NTP update failed");
    return false;
  }

  currentTime = timeClient.getEpochTime(); // Obtiene la hora en formato UNIX
  Serial.print("Current time: ");
  Serial.println(currentTime);
  lastMillisSync = millis();  // Guarda el tiempo del último sync
  return true;
}

/**
 * @brief Updates local time with millis() if there is no internet connection
 */
void updateTimeWithoutNTP(void) 
{
  unsigned long elapsedMillis = millis() - lastMillisSync;
  currentTime += elapsedMillis / 1000;  // Suma los segundos transcurridos
  lastMillisSync = millis();  // Reinicia el contador
}

/**
 * @brief Gets local time formatted
 *
 * @return String with the formatted local time (DD/MM/YYYY HH:MM:SS)
 */
String getFormattedTime(void) 
{
  updateTimeWithoutNTP();
  
  struct tm *timeInfo;
  timeInfo = localtime(&currentTime);  // Convierte el tiempo UNIX a estructura de fecha y hora

  Serial.print("Current time: ");
  Serial.println(currentTime);

  char buffer[20];  
  strftime(buffer, 20, "%Y-%m-%d %H:%M:%S", timeInfo); // Formato DD/MM/YYYY HH:MM:SS

  return String(buffer);
}

// -------------------------------------------------------------- //
// -------------------- [ Circular Buffer ] --------------------- //
// -------------------------------------------------------------- //

CircularBuffer circularBuffer;

// -------------------------------------------------------------- //
// --------------------- [ Config Button ] ---------------------- //
// -------------------------------------------------------------- //

#define DEBOUNCE_DELAY 50 

const int WIFI_RESET_BUTTON = 5;

bool configButtonState() 
{
  static unsigned long lastDebounceTime = 0; // Último tiempo que se detectó un cambio
  static bool lastButtonState = LOW;         // Último estado conocido del botón
  static bool buttonState = LOW;   
  
  bool reading = digitalRead(WIFI_RESET_BUTTON); 
  
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {

    if (reading != buttonState) {
      buttonState = reading;
    }
  }

  // Guarda el estado del botón para la próxima iteración
  lastButtonState = reading;

  return buttonState;
}

bool configButton() 
{
  static bool state = false;
  static bool lastState = false;

  state = configButtonState();
  if (state != lastState) {   // En caso de detectar un flanco, actualiza cual es el estado
    lastState = state;
    
    return state;
  } 
  
  // Si se mantiene el estado del botón, es como si no se hubiese presionado
  lastState = state;
  return false;
}

// -------------------------------------------------------------- //
// -------------------------- [ MQTT ] -------------------------- //
// -------------------------------------------------------------- //

const char* MQTT_SERVER = "raspberrypi.local";  // Dirección del broker MQTT
const int MQTT_PORT = 1883;                     // Puerto MQTT (usualmente 1883)
const char* MQTT_TOPIC = "energyMeter";         // Tópico donde publicar

WiFiClient espClient;  
PubSubClient mqttClient(espClient);

#define MQTT_RETRY_THRESHOLD 2000 // 2 seg

bool mqttActive = true; // Bandera para controlar el estado de MQTT

String message;

void updateMqttServer(const char* mqtt_server, const int mqtt_port)
{
  static IPAddress lastResolvedIP;
  IPAddress resolvedIP;
   
  if (WiFi.hostByName(mqtt_server, resolvedIP)) {
    if (resolvedIP != lastResolvedIP) {
      Serial.print("Nueva IP resuelta para MQTT: ");
      Serial.println(resolvedIP);

      lastResolvedIP = resolvedIP;
      mqttClient.setServer(resolvedIP, mqtt_port);
    }
  } 
  else {
    Serial.print("No se pudo resolver ");
    Serial.println(mqtt_server);
  }
}

void connectToMQTT() 
{
  static long prevMqttRetryTime = 0;

  if (millis() - prevMqttRetryTime >= MQTT_RETRY_THRESHOLD) {
    prevMqttRetryTime = millis();
    
    Serial.println("Conectando al broker MQTT...");
    if (mqttClient.connect("ESP8266Client")) { // Nombre del cliente MQTT
      Serial.println("Conectado al broker MQTT.");
      mqttActive = true;
    } 
    else {
      Serial.print("Falló la conexión MQTT. Código: ");
      Serial.println(mqttClient.state());
    }
  }
}

// Detener MQTT
void stopMQTT() 
{
  if (mqttClient.connected()) {
    mqttClient.disconnect(); // Cerrar la conexión con el broker
    mqttActive = false;  // Desactivar MQTT en el loop
    Serial.println("MQTT detenido.");
  }
}

void mqttConnectionControl()
{
  if (WiFi.status() == WL_CONNECTED) {
    
    if (!energyMeterConfig.customMqttBroker) {
      updateMqttServer(MQTT_SERVER, MQTT_PORT); 
    }
            
    if (!mqttClient.connected()) {
      connectToMQTT();
      return;
    }

    mqttClient.loop(); // Mantener conexión MQTT
  }
}

// -------------------------------------------------------------- //
// -------------------------- [ Wifi ] -------------------------- //
// -------------------------------------------------------------- //

const int LED_WEB_SERVER = 4;

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Search for parameter in HTTP POST request
const char* PARAM_INPUT_1 = "ssid";
const char* PARAM_INPUT_2 = "pass";
const char* PARAM_INPUT_3 = "static-ip-enable";
const char* PARAM_INPUT_4 = "static-ip";
const char* PARAM_INPUT_5 = "gateway";
const char* PARAM_INPUT_6 = "custom-mqtt-broker";
const char* PARAM_INPUT_7 = "mqtt-broker";
const char* PARAM_INPUT_8 = "mqtt-port";

const char* WIFI_AP_SSID = "Energy Monitor";
const char* WIFI_AP_PASSWORD = NULL;

enum WIFI_ERROR_CODES {
  SUCCESS = 0,
  UNDEFINED_SSID_OR_IP = 1,
  STA_FAILED = 2,
  FAILED_TO_CONNECT = 3,
};

#define RETRY_THRESHOLD 5000 // 5 seg
long prevWifiRetryTime = 0;
bool wifiStarted = false;

// Initialize WiFi
int initWiFi(const energyMeterConfigParams& config) 
{
  // Configure static IP
  if (config.staticIpEnable) {
    
    IPAddress localIP;
    IPAddress localGateway;
    IPAddress subnet(255, 255, 255, 0);
    IPAddress primaryDNS(8, 8, 8, 8);         
    IPAddress secondaryDNS(8, 8, 4, 4);       
    
    localIP.fromString(config.staticIp);
    localGateway.fromString(config.gateway);
    
    if (!WiFi.config(localIP, localGateway, subnet, primaryDNS, secondaryDNS)) {
      Serial.println("STA Failed to configure");
      return STA_FAILED;
    }
  }
  
  if (config.ssid == NULL) {
    Serial.println("Udefined SSID");
    return UNDEFINED_SSID_OR_IP;
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(config.ssid, config.password);

  Serial.println("Connecting to WiFi...");
  return SUCCESS;
}

int deinitWifi() 
{
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);

  return SUCCESS;
}

void initAP(energyMeterConfigParams &config)
{
  Serial.println("Setting AP (Access Point)");
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP); 

  // Web Server Root URL
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(LittleFS, "/wifimanager.html", "text/html");
  });

  server.serveStatic("/", LittleFS, "/");
  
  // Path to send dynamic variables
  server.on("/getConfig", HTTP_GET, [&config](AsyncWebServerRequest *request) {
    String json = "{";
    json += "\"ssid\":\"" + String(config.ssid) + "\",";
    json += "\"password\":\"" + String(config.password) + "\",";
    json += "\"static-ip-enable\":\"" + String(config.staticIpEnable ? "true" : "false") + "\",";
    json += "\"static-ip\":\"" + String(config.staticIp) + "\",";
    json += "\"gateway\":\"" + String(config.gateway) + "\"";
    json += "\"custom-mqtt-broker\":\"" + String(config.customMqttBroker ? "true" : "false") + "\",";
    json += "\"mqtt-broker\":\"" + String(config.mqttBroker) + "\",";
    json += "\"mqtt-port\":\"" + String(config.mqttPort) + "\"";
    json += "}";
    request->send(200, "application/json", json);
  });

  // Get wifi configuration
  server.on("/", HTTP_POST, [&config](AsyncWebServerRequest *request) {
    int params = request->params();
    for (int i=0; i<params; i++) {
      AsyncWebParameter* p = request->getParam(i);
      if (p->isPost()) {
        // HTTP POST ssid value
        if (p->name() == PARAM_INPUT_1) {
          config.ssid = p->value().c_str();
          Serial.print("SSID set to: ");
          Serial.println(config.ssid);
        }
        // HTTP POST pass value
        if (p->name() == PARAM_INPUT_2) {
          config.password = p->value().c_str();
          Serial.print("Password set to: ");
          Serial.println(config.password);
        }
        // HTTP POST static ip enable value
        if (p->name() == PARAM_INPUT_3) {
          config.staticIpEnable = (p->value() == "on") ;
          Serial.print("Static IP enable: ");
          Serial.println(config.staticIpEnable);
        }
        // HTTP POST ip value
        if (p->name() == PARAM_INPUT_4) {
          config.staticIp = p->value().c_str();
          Serial.print("IP Address set to: ");
          Serial.println(config.staticIp);
        }
        // HTTP POST gateway value
        if (p->name() == PARAM_INPUT_5) {
          config.gateway = p->value().c_str();
          Serial.print("Gateway set to: ");
          Serial.println(config.gateway);
        }
        // HTTP POST custom mqtt broker value
        if (p->name() == PARAM_INPUT_6) {
          config.customMqttBroker = (p->value() == "on") ;
          Serial.print("Custom MQTT Broker: ");
          Serial.println(config.staticIpEnable);
        }
        // HTTP POST mqtt broker ip value
        if (p->name() == PARAM_INPUT_7) {
          config.mqttBroker = p->value().c_str();
          Serial.print("MQTT Broker set to: ");
          Serial.println(config.mqttBroker);
        }
        // HTTP POST mqtt port value
        if (p->name() == PARAM_INPUT_8) {
          config.mqttPort = p->value().toInt();
          Serial.print("MQTT Port set to: ");
          Serial.println(config.mqttPort);
        }
      }
    }

    if (config.customMqttBroker) {
      mqttClient.setServer(config.mqttBroker, config.mqttPort);
    }

    saveConfig(config); 

    // Returns confirmation
    request->send(200, "text/plain", "OK");
  });
  
  server.begin();
  apRunning = true;
}

// -------------------------------------------------------------- //
// --------------------------- [ MEF ] -------------------------- //
// -------------------------------------------------------------- //

enum mefStates {
  sWebServer = 0,
  sConnectWifiSta,
  sSyncTime,
  sReadEnergy,
  sSendValues,
  sStandBy,
};

int state = 0;
bool stateEntry = true;

// #define READING_THS           500     // 0.5 seg
#define READING_THS              5000    // 5 seg
// #define SAVE_DATA_THS         300000  // 5.0 min
#define SAVE_DATA_THS            30000    // 30 seg

unsigned long prevReading = 0;
unsigned long prevSaveData = 0;
unsigned long prevSendData = 0;

bool saveDataFlag = false;

// -------------------------------------------------------------- //
// -------------------------- [ Setup ] ------------------------- //
// -------------------------------------------------------------- //

void setup() {
  
  // Serial port for debugging purposes
  Serial.begin(9600);
  
  pinMode(WIFI_RESET_BUTTON, INPUT_PULLUP);
  pinMode(LED_WEB_SERVER, OUTPUT);

  initFS();
  
  // Load values saved in LittleFS
  loadConfig(energyMeterConfig);

  // Initial state
  (energyMeterConfig.ssid != NULL) ? state = sConnectWifiSta : state = sWebServer;

  bl0940.Reset();
  bl0940.setFrequency(50);    // 50[Hz]
  bl0940.setUpdateRate(800);  // 800[ms]
}

// -------------------------------------------------------------- //
// -------------------------- [ Loop ] -------------------------- //
// -------------------------------------------------------------- //

void loop() {
  
  mqttConnectionControl();
  
  timeControl();

  switch (state) {
    case sWebServer:
    {
      if (stateEntry) {
        Serial.println("state: sWebServer");
        stateEntry = false;
        
        wifiStarted = false;

        if (!apRunning) {
          deinitWifi();
          digitalWrite(LED_WEB_SERVER, HIGH);
          initAP(energyMeterConfig);
        }
      }
      
      if (configButton()) {
        deinitWifi();
        digitalWrite(LED_WEB_SERVER, LOW);
        apRunning = false;

        loadConfig(energyMeterConfig);
        
        state = sConnectWifiSta;
        stateEntry = true;
      }
      
      break;
    } 

    case sConnectWifiSta:
    {
      if (stateEntry) {
        Serial.println("state: sConnectWifiSta");
        stateEntry = false;
        
        if (!wifiStarted) {
          if (initWiFi(energyMeterConfig) != SUCCESS) {
            state = sWebServer;
            stateEntry = true;
          }
          else {
            wifiStarted = true;
          }
        }

        mqttActive = false;

        prevWifiRetryTime = millis();
        prevSaveData = millis();
      }

      if (WiFi.status() != WL_CONNECTED && millis() - prevWifiRetryTime >= RETRY_THRESHOLD) {
        prevWifiRetryTime = millis();
        Serial.println("Failed to connect. Retrying WiFi connection in 5 seconds...");
      }

      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("Wifi connected!.");
        Serial.println(WiFi.localIP());
        
        // Iniciar cliente NTP
        if (!ntpStarted) {
          timeClient.begin();
          ntpStarted = true;

          state = sSyncTime;
          stateEntry = true;
        }
        else {
          state = sReadEnergy;
          stateEntry = true;
        }
      }

      if (millis() - prevSaveData >= SAVE_DATA_THS && ntpSync) {
        saveDataFlag = true;

        state = sReadEnergy;
        stateEntry = true;
      }

      if (configButton()) {
        state = sWebServer;
        stateEntry = true;
      }

      break;
    } 
    case sSyncTime:
    {
      if (stateEntry) {
        Serial.println("state: sSyncTime");

        if (updateLocalTime()) {
          ntpSync = true;

          state = sReadEnergy;
          stateEntry = true;
        }
        
        prevNtpRetryTime = millis();
      }
      
      if (!ntpSync && millis() - prevNtpRetryTime >= NTP_RETRY_TIME) {
        if (updateLocalTime()) {
          ntpSync = true;

          state = sReadEnergy;
          stateEntry = true;
        }
      }

      if (configButton()) {
        state = sWebServer;
        stateEntry = true;
      }
      
      break;
    }
    case sReadEnergy:
    {
      if (stateEntry) {
        Serial.println("state: sReadEnergy");
        stateEntry = false;

        prevReading = millis();
      }
    
      // Read BL0940 data
      float voltage;
      bl0940.getVoltageRMS( &voltage );

      float current;
      bl0940.getCurrentRMS( &current );

      float activePower;
      bl0940.getActivePower( &activePower );

      float activeEnergy;
      bl0940.getActiveEnergy( &activeEnergy );

      float powerFactor;
      bl0940.getPowerFactor( &powerFactor );

      // Format MQTT message
      // YYYY-MM-DD HH:mm:ss|VoltageRMS|CurrentRMS|ActivePower|ActiveEnergy|PowerFactor
      message = getFormattedTime() + "|" + 
                voltage + "|" +
                current + "|" + 
                activePower + "|" +
                activeEnergy + "|" +
                powerFactor;
      Serial.print("MQTT message: ");
      Serial.println(message);

      state = sSendValues;
      stateEntry = true;

      break;
    }
    case sSendValues:
    {
      if (stateEntry) {
        Serial.println("state: sSendValues");
        stateEntry = false;
      }

      static bool savedData = false;

      if (mqttActive) {
        if (mqttClient.publish(MQTT_TOPIC, message.c_str())) {
          Serial.println("Mensaje publicado! Mensaje: ");
          Serial.println(message.c_str());

          prevSendData = millis();

          if (circularBuffer.amountOfSavedData() != 0) {
            message = circularBuffer.getSavedData();
            savedData = true;
          }
          else {
            savedData = false;
            state = sStandBy;
            stateEntry = true;
          }
        }
      }

      if (millis() - prevSendData >= SAVE_DATA_THS || saveDataFlag) {
        saveDataFlag = false;
        
        prevSendData = millis();
        
        circularBuffer.saveData(message);
      }
      
      if (millis() - prevReading > READING_THS && !savedData) {
        state = sReadEnergy;
        stateEntry = true;
      }

      if (WiFi.status() != WL_CONNECTED) {
        state = sConnectWifiSta;
        stateEntry = true;
      }

      if (configButton()) {
        state = sWebServer;
        stateEntry = true;
      }

      break;
    }

    case sStandBy: 
    {
      if (stateEntry) {
        Serial.println("state: sStandBy");
        stateEntry = false;
      }

      if (millis() - prevReading > READING_THS) {
        state = sReadEnergy;
        stateEntry = true;
      }
      
      if (WiFi.status() != WL_CONNECTED) {
        state = sConnectWifiSta;
        stateEntry = true;
      }

      if (configButton()) {
        state = sWebServer;
        stateEntry = true;
      }

      break;
    }
  }
}

// TODO: Ver de cambiar algunas variables globales a variables de función con static