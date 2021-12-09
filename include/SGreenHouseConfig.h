#ifndef S_GREENHOUSE_CONFIG_H /*== INCLUDE ==*/
#define S_GREENHOUSE_CONFIG_H /*=== GUARD ===*/

#include <Arduino.h>
#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include <DHT_U.h>
#include <DHT.h>
#include "secrets.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#if CONFIG_FREERTOS_UNICORE
  #define ARDUINO_RUNNING_CORE 0
#else
  #define ARDUINO_RUNNING_CORE 1
#endif

#ifndef LED_BUILTIN
  #define LED_BUILTIN 2
#endif

/*--------------------------------------------------*/
/*-------------------- DEBUG -----------------------*/
/*--------------------------------------------------*/
/* 
All the debug macros are defined as: "DEBUG && ( user_defined_flag )"
so that if the debug mode is turned off, all the macros will
have value zero.
*/ 
#define DEBUG                                1  // Enables the debug mode 

  // Queues
  #define PRINT_COORDINATOR_QUEUE_USAGE    DEBUG && ( 0 )    
  
  // WiFi connectivity
  #define WiFi_CONNECTION_VERBOSE_DEBUG    DEBUG && ( 1 )    
  
  // MQTT connectivity
  #define MQTT_CONNECTION_VERBOSE_DEBUG    DEBUG && ( 1 )

  // MQTT queue reset
  #define MQTT_QUEUE_VERBOSE_REMOVE_DEBUG  DEBUG && ( 0 )

  // MQTT publish fail
  #define MQTT_PUBLISH_FAIL_VERBOSE_DEBUG  DEBUG && ( 0 )

  // MQTT subscriptions
  #define MQTT_FETCH_SUB_VERBOSE_DEBUG     DEBUG && ( 1 )

  #if MQTT_FETCH_SUB_VERBOSE_DEBUG
    #define printFetchedValue(var_name, updated_variable) {\
                                                            Serial.print("Fetched a new value for ");\
                                                            Serial.print(var_name);\
                                                            Serial.print(": ");\
                                                            Serial.println(updated_variable);\
                                                          }
  #endif

  // Sensors
  #define DEBUG_SENSORS                    DEBUG && ( 1 ) // Enables sensor simulation for debug purposes (It also disables the actual sensor readings)
  #define SENSORS_VERBOSE_DEBUG            DEBUG && ( 0 ) // Enables verbose sensor output

  #if DEBUG_SENSORS
    #define sensorSim() ((float32_t) random(0,100)) // This *SHOULD* be substituted with a regular function according to the MISRA C guidelines
  #endif

  // Actuators
  #define DEBUG_ACTUATORS                  DEBUG && ( 1 ) // Disables the actual activation of the actuator devices
  #define ACTUATORS_VERBOSE_DEBUG          DEBUG && ( 0 ) // Enables verbose actuator output

  // Memory usage
  #define PRINT_TASK_MEMORY_USAGE          DEBUG && ( 0 )     

/*--------------------------------------------------*/
/*----------- Task priority assignation ------------*/
/*--------------------------------------------------*/
#define COORDINATOR_PRIORITY 2
#define CONNECT_PRIORITY 4
#define MQTT_PUBLISH_PRIORITY 2
#define MQTT_FETCH_SUBSCRIPTIONS_PRIORITY 2
#define SENSOR_TASKS_PRIORITY 1
#define ACTUATOR_TASKS_PRIORITY 5

/*--------------------------------------------------*/
/*----------- Connectivity configuration -----------*/
/*--------------------------------------------------*/
// WiFi credentials
const char* ssid = SECRET_SSID;
const char* password = SECRET_PASS;
// MQTT server
const char* AIO_SERVER = SECRET_SERVER_ADDR;
const int AIO_SERVERPORT = SECRET_SERVER_PORT;

// WiFi connection configuration
#define WiFiConnectAttemptDelay 5000

// MQTT connection
#define MAX_CONNECTION_ATTEMPTS 3
#define MQTTConnectAttemptDelay 3000

// MQTT publish
#define MQTT_PUBLISH_PER_EXECUTION 5
#define MQTT_MAX_PUBLISHING_ATTEMPTS 3

// MQTT subscribe
#define MQTT_SUBSCRIPTION_READING_TIMEOUT 1000

/*--------------------------------------------------*/
/*------------------- MQTT topics ------------------*/
/*--------------------------------------------------*/
// Main topics
#define MAIN_TOPIC        "garden/greenhouse"
#define SENS_TOPIC        MAIN_TOPIC "/sensors"
#define SETT_TOPIC        MAIN_TOPIC "/settings"
// Sensor topics
#define SENS_TEMP_TOPIC        SENS_TOPIC "/f/temperature"
#define SENS_AIR_HUM_TOPIC     SENS_TOPIC "/f/airhumidity"
#define SENS_SOIL_HUM_TOPIC    SENS_TOPIC "/f/soilhumidity"
#define SENS_LUX_TOPIC         SENS_TOPIC "/f/lux"
// Actuator topics (Last activation logs?)

// Settings topics

// --Thresholds
#define THRESHOLDS_TOPIC SETT_TOPIC "/thresholds"
// ----Soil humidity thresholds
#define SOIL_HUM_THR_TOPIC        THRESHOLDS_TOPIC "/soilhumidity"
#define MIN_SOIL_HUM_THR_TOPIC    SOIL_HUM_THR_TOPIC "/min"
#define MAX_SOIL_HUM_THR_TOPIC    SOIL_HUM_THR_TOPIC "/max"
// ----Air humidity thresholds
#define AIR_HUM_THR_TOPIC         THRESHOLDS_TOPIC "/airhumidity"
#define MIN_AIR_HUM_THR_TOPIC     AIR_HUM_THR_TOPIC "/min"
#define MAX_AIR_HUM_THR_TOPIC     AIR_HUM_THR_TOPIC "/max"
// ----Temperature thresholds
#define TEMPERATURE_THR_TOPIC     THRESHOLDS_TOPIC "/temperature"
#define MIN_TEMP_THR_TOPIC        TEMPERATURE_THR_TOPIC "/min"
#define MAX_TEMP_THR_TOPIC        TEMPERATURE_THR_TOPIC "/max"
// ----Light thresholds
#define LIGHTS_THR_TOPIC          THRESHOLDS_TOPIC "/lights"
#define ON_LIGHTS_THR_TOPIC       LIGHTS_THR_TOPIC "/on"
#define OFF_LIGHTS_THR_TOPIC      LIGHTS_THR_TOPIC "/off"

// --Other settings
// ----Irrigatior activation duration and delay between activations
#define IRRIG_ACT_TOPIC           SETT_TOPIC "/irrigator/activation"
#define IRRIG_ACT_DURATION_TOPIC  IRRIG_ACT_TOPIC "/duration"
#define IRRIG_ACT_DELAY_TOPIC     IRRIG_ACT_TOPIC "/delay"

/*--------------------------------------------------*/
/*------------------ Pin Defines -------------------*/
/*--------------------------------------------------*/
// Sensors
#define DHT11PIN 4 // Digital temperature and air humidity sensor
#define YL69PIN 22 // Analog soil humidity sensor
// Actuators
#define irrigatorPIN 32 // Controls the custom PCB in charge of the irrigator management
#define lightsPIN 33 // Controls a relè designed to switch on and off a 220V growlamp

/*--------------------------------------------------*/
/*------ MQTT Default configurable parameters ------*/
/*--------------------------------------------------*/
#define DEFAULT_IrrigatorActivationThreshold 30
#define DEFAULT_MaxSoilHumidityThreshold 80
#define DEFAULT_MinAirHumidityThreshold 10
#define DEFAULT_MaxAirHumidityThreshold 80
#define DEFAULT_MinTemperatureThreshold 15
#define DEFAULT_MaxTemperatureThreshold 45
#define DEFAULT_IrrigatorExecutionTime 5000
#define DEFAULT_IrrigatorBetweenActivationsDelay 10000
#define DEFAULT_LightsActivationThreshold 30
#define DEFAULT_LightsDeactivationThreshold 80

/*--------------------------------------------------*/
/*--------- Task Periods(in milliseconds) ----------*/
/*--------------------------------------------------*/
#define CoordinatorPeriod 500
#define MQTTPublishPeriod 2500
#define MQTTSubscribePeriod 10000
#define DHT11TemperaturePeriod 5000 // 5 sec -- Temporary debug value
#define DHT11HumidityPeriod 5000    // 5 sec -- Temporary debug value
#define YL69SoilHumidityPeriod 5000 // 5 sec -- Temporary debug value
#define LuxReadingPeriod 5000       // 5 sec -- Temporary debug value

/*--------------------------------------------------*/
/*----------------- Queue Config -------------------*/
/*--------------------------------------------------*/
#define coordinator_queue_len 10 // Coordinator queue length
#define MQTTpub_queue_len 10 

/*--------------------------------------------------*/
/*------- Type, enum and struct definitions --------*/
/*--------------------------------------------------*/
// *Type definitions*
// MISRA-C Compliance (size specific data types)
typedef float float32_t; //ESP32 boards use 32 bit for storing floats

// *Enum definitions*
enum sensor_id{Sensor_Id_DHT11Temperature, Sensor_Id_DHT11Humidity, Sensor_Id_YL69SoilHumidity, Sensor_Id_Lux};

// *Struct definitions*
struct sensor_msg{
  enum sensor_id sensor;
  float32_t sensor_reading;
};

struct MQTT_subscription{
  Adafruit_MQTT_Subscribe sub_obj;
  void (*callback) (char*, uint16_t);
};

#endif // S_GREENHOUSE_CONFIG_H -- End of the header file
