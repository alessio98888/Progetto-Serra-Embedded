# Smart Greenhouse
## Description
This project provides various features related to the maintenance of a plant.
The main features are the following:
- automatic irrigation based on the soil moisture level
- automatic switching on of lights to illuminate the plant in times of low ambient light (for example at night)
- display on the mobile app (via MQTT) of the values ​​of brightness, soil humidity, air humidity and temperature
- possibility of configuring various parameters related to irrigation and plant lighting. In particular, it is possible to configure parameters belonging to the following macro-categories:
  - activation thresholds (for example under which soil moisture level you intend to start irrigation)
  - execution times (for example how long to water the plant after it has been verified that the soil moisture is too low)
## Requirements
- Hardware Requirements
  - ESP32 Board
  - DHT11 Temperature and Humidity Sensor
  - YL-69 Soil Moisture Sensor
- Software Requirements
  - FreeRTOS OS
  - Platform IO
  - MQTT
  
# Software Tools
The software tools used for the realization of the project are:
- Visual Studio Code with *Platform IO* for compiling and uploading code on the ESP32 Board
- Git for code sharing between contributors
