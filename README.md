# FCEIA_ProyectoFinal
Proyecto Final de Ing. Electrónica: Sistema de monitoreo de red eléctrica doméstica y/o industrial

## ESP8266 Setup
The ESP8266 NodeMCU v3 board is used for prototyping. To connect to this board, it is necessary to install the CH340G driver by following the instructions in  [this tutorial](https://www.instructables.com/Instalar-driver-para-CH340G/).

In addition to this, you need to add the ESP8266 boards to the Arduino IDE with the following steps:

1. Open the Arduino IDE, select `File -> Preferences`, and enter `http://arduino.esp8266.com/stable/package_esp8266com_index.json` into the Additional Board Manager URLs field. You can add multiple URLs by separating them with commas.

2. Open Boards Manager from `Tools -> Board -> Boards Manager` and install the esp8266 platform. To easily find the correct platform, search for "ESP8266" in the search bar.

3. Select the ESP8266 module that you are using. In this case, select **NodeMCU 1.0 (ESP-12E Module)**.

4. Go to `Tools -> Port` and select the appropriate PORT for your device.

## Installing libraries
You need to install the following libraries in your Arduino IDE to build the web server for this project.

- ESPAsyncWebServer
- ESPAsyncTCP

## LittleFS
LittleFS is a lightweight filesystem created for microcontrollers that lets you access the flash memory like you would do in a standard file system on your computer, but it’s simpler and more limited. You can read, write, close, and delete files and folders. 

### Filesystem Uploader
Before proceeding, you need to have the ESP8266 Uploader Plugin installed in your Arduino IDE. 

- [ArduinoIDE 1.8.x](https://randomnerdtutorials.com/install-esp8266-nodemcu-littlefs-arduino/)
- [ArduinoIDE 2.1.1 or higher](https://randomnerdtutorials.com/arduino-ide-2-install-esp8266-littlefs/)