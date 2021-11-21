#ifndef S_GREENHOUSE_CONFIG_H /*== INCLUDE ==*/
#define S_GREENHOUSE_CONFIG_H /*=== GUARD ===*/

#include "secrets.h"

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

/*--------------------------------------------------*/
/*------------------- MQTT topics ------------------*/
/*--------------------------------------------------*/
// Main topics
#define MAIN_TOPIC        "garden/greenhouse"
#define SENS_TOPIC        MAIN_TOPIC "/sensors"
#define SETTINGS_TOPIC    MAIN_TOPIC "/settings"
// Sensor topics
#define SENS_TEMP_TOPIC        SENS_TOPIC "/f/temperature"
#define SENS_AIR_HUM_TOPIC     SENS_TOPIC "/f/air_humidity"
#define SENS_SOIL_HUM_TOPIC    SENS_TOPIC "/f/soil_humidity"
#define SENS_LUX_TOPIC       SENS_TOPIC "/f/lux"
// Actuator topics (Last activation logs?)

// Settings topics
// --Soil humidity thresholds
#define SOIL_HUM_THR_TOPIC        SETT_TOPIC "/soil_humidity/tresholds"
#define MIN_SOIL_HUM_THR_TOPIC    SOIL_HUM_THR_TOPIC "/min"
#define MAX_SOIL_HUM_THR_TOPIC    SOIL_HUM_THR_TOPIC "/max"
// --Light threshold
#define LIGHTS_THR_TOPIC           SETT_TOPIC "/light/tresholds"
#define ON_LIGHTS_THR_TOPIC       LIGHTS_THR_TOPIC "/turn_on"
#define OFF_LIGHTS_THR_TOPIC       LIGHTS_THR_TOPIC "/turn_off"
// --Irrigatior activation duration and delay between activations
#define IRRIG_ACT_TOPIC           SETT_TOPIC "/irrigator/activation"
#define IRRIG_ACT_DURATION_TOPIC  IRRIG_ACT_TOPIC "/duration"
#define IRRIG_ACT_DELAY_TOPIC     IRRIG_ACT_TOPIC "/delay"

/*--------------------------------------------------*/
/*------------ Task specific parameters ------------*/
/*--------------------------------------------------*/
// Irrigator
#define IrrigatorActivationThreshold 30 // in per cent 
#define IrrigatorExecutionTime 5000

// Lights
#define LightsActivationThreshold 30
#define LightsDeactivationThreshold 80

/*--------------------------------------------------*/
/*------------------ Pin Defines -------------------*/
/*--------------------------------------------------*/
// Sensors
#define DHT11PIN 4 // Digital temperature and air humidity sensor
#define YL69PIN 22 // Analog soil humidity sensor
// Actuators
#define irrigatorPIN 32 // Controls the custom PCB in charge of the irrigator management
#define lightsPIN 33 // Controls a relè designed to switch on and off 220V growlamp

/*--------------------------------------------------*/
/*--------- Task Periods(in milliseconds) ----------*/
/*--------------------------------------------------*/
#define CoordinatorPeriod 500
#define DHT11TemperaturePeriod 5000 // 5 sec -- Temporary debug value
#define DHT11HumidityPeriod 5000    // 5 sec -- Temporary debug value
#define YL69SoilHumidityPeriod 5000 // 5 sec -- Temporary debug value
#define LuxReadingPeriod 5000       // 5 sec -- Temporary debug value

/*--------------------------------------------------*/
/*----------- Coordinator Queue Config -------------*/
/*--------------------------------------------------*/
#define coordinator_queue_len 10 // Coordinator queue length

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

#endif // S_GREENHOUSE_CONFIG_H -- End of the header file
