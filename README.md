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
- TODO: Add MQTT libraries PubSubClient y no me acuerdo cuál otra

## LittleFS
LittleFS is a lightweight filesystem created for microcontrollers that lets you access the flash memory like you would do in a standard file system on your computer, but it’s simpler and more limited. You can read, write, close, and delete files and folders. 

### Filesystem Uploader
Before proceeding, you need to have the ESP8266 Uploader Plugin installed in your Arduino IDE. 

- [ArduinoIDE 1.8.x](https://randomnerdtutorials.com/install-esp8266-nodemcu-littlefs-arduino/)
- [ArduinoIDE 2.1.1 or higher](https://randomnerdtutorials.com/arduino-ide-2-install-esp8266-littlefs/)

## Raspberry Pi Model 3 B+

TODO: Description

### Mosquitto MQTT Broker
Steps:

**1)** Update and upgrade the system 

```
sudo apt-get update && sudo apt-get upgrade
```

**2)** Install the Mosquitto Broker

```
sudo apt install -y mosquitto mosquitto-clients
``` 

**3)** Run the following command to open the mosquitto.conf file

```
sudo nano /etc/mosquitto/mosquitto.conf
```

**4)** Move to the end of the file using the arrow keys and paste the following two lines

```
listener 1883
allow_anonymous true
```
This allows the connection of remote devices on port 1883

**5)** To make Mosquitto auto start when the Raspberry Pi boots, you need to run the following command (this means that the Mosquitto broker will automatically start when the Raspberry Pi starts)

```
sudo systemctl enable mosquitto.service
```
Mosquitto can also be run in the background as a daemon. This has to be done every time the RPi is turned on. 
```
mosquitto -d
```
We can test if everything is in order by creating a publisher and a subscriber in separate shells:

```
mosquitto_sub -d -t testTopic
```
```
mosquitto_pub -d -t testTopic -m "Hello world!"
```









