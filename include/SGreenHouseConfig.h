#ifndef S_GREENHOUSE_CONFIG_H /*== INCLUDE ==*/
#define S_GREENHOUSE_CONFIG_H /*=== GUARD ===*/

#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include "secrets.h"
#include "DHTesp.h"
#include <BH1750.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <Preferences.h>

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

  // Preferences
  #define DEBUG_PREFERENCES                DEBUG && ( 1 )

  // Queues
  #define PRINT_COORDINATOR_QUEUE_USAGE    DEBUG && ( 0 )    
  
  // Mutex
  #define MUTEX_ACCESS_VERBOSE_DEBUG       DEBUG && ( 1 )

  #if MUTEX_ACCESS_VERBOSE_DEBUG
    #define PRINT_MUTEX_TAKE(mutex_name, task_name){\
                                                      Serial.print(mutex_name);\
                                                      Serial.print(" TAKEN by task: ");\
                                                      Serial.println(task_name);\
                                                    }
    #define PRINT_MUTEX_GIVE(mutex_name, task_name){\
                                                      Serial.print(mutex_name);\
                                                      Serial.print(" GIVEN by task: ");\
                                                      Serial.println(task_name);\
                                                    }
  #endif

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
                                                            Serial.print(var_name);\
                                                            Serial.print(" updated to: ");\
                                                            Serial.println(updated_variable);\
                                                          }
  #endif

  // Sensors
  #define DEBUG_SENSORS                    DEBUG && ( 0 ) // Enables sensor simulation for debug purposes (It also disables the actual sensor readings)
  #define SENSORS_VERBOSE_DEBUG            DEBUG && ( 1 ) // Enables verbose sensor output
  #define BH1750_LUX_SENSOR_STATUS_DEBUG   DEBUG && ( 1 )           

  #if DEBUG_SENSORS
    #define sensorSim() ((float32_t) random(0,100)) // This *SHOULD* be substituted with a regular function according to the MISRA C guidelines
  #endif

  // Actuators
  #define DEBUG_ACTUATORS                  DEBUG && ( 0 ) // Disables the actual activation of the actuator devices
  #define ACTUATORS_VERBOSE_DEBUG          DEBUG && ( 1 ) // Enables verbose actuator output

  // Memory usage
  #define PRINT_TASK_MEMORY_USAGE          DEBUG && ( 0 )     

/*--------------------------------------------------*/
/*----------- Task priority assignation ------------*/
/*--------------------------------------------------*/
#define COORDINATOR_PRIORITY 2
#define CONNECT_PRIORITY 3
#define MQTT_PUBLISH_PRIORITY 1
#define MQTT_FETCH_SUBSCRIPTIONS_PRIORITY 2
#define SENSOR_TASKS_PRIORITY 1
#define ACTUATOR_TASKS_PRIORITY 4

/*--------------------------------------------------*/
/*----------------- Queue Config -------------------*/
/*--------------------------------------------------*/
#define coordinator_queue_len Amount_of_sensor_ids*2 // Coordinator queue length
#define MQTTpub_queue_len Amount_of_sensor_ids*2

/*--------------------------------------------------*/
/*--------------- Other task config ----------------*/
/*--------------------------------------------------*/
// *CONNECTIVITY*
// Setup connection configuration
#define SETUP_MAX_CONNECTION_ATTEMPTS 5

// WiFi connection configuration
#define WiFiConnectAttemptDelay 5000

// MQTT connection
#define MAX_CONNECTION_ATTEMPTS 3
#define MQTTConnectAttemptDelay 3000

// MQTT publish
#define MQTT_PUBLISH_PER_EXECUTION MQTTpub_queue_len
#define MQTT_MAX_PUBLISHING_ATTEMPTS 3

// MQTT subscribe
#define MQTT_SUBSCRIPTION_READING_TIMEOUT 1000

// *SENSORS*
// YL-69 soil moisture sensor reading time
#define YL69_READING_TIME 200
// Max BH1750 light intensity (lux) sensor reding attempts
#define BH1750_LUX_SENSOR_MAX_READING_ATTEMPTS 1 // Can only be 1 or 2 tries

// *ACTUATORS*
// Minimum irrigator execution time allowed
#define IRRIG_MIN_ACTUATION_DURATION 1000
#define IRRIG_MIN_ACTUATION_DELAY 1000

/*--------------------------------------------------*/
/*------------------- MQTT topics ------------------*/
/*--------------------------------------------------*/
// Main topics
#define MAIN_TOPIC        "garden/greenhouse"
#define SENS_TOPIC        MAIN_TOPIC "/sensors"
#define SETT_TOPIC        MAIN_TOPIC "/settings"
// Sensor topics
#define SENS_TEMP_TOPIC        SENS_TOPIC "/f/temperature"
#define SENS_AIR_MOIST_TOPIC     SENS_TOPIC "/f/airhumidity"
#define SENS_SOIL_MOIST_TOPIC    SENS_TOPIC "/f/soilhumidity"
#define SENS_LUX_TOPIC         SENS_TOPIC "/f/lux"
// Actuator topics (Last activation logs?)

// Settings topics

// --Thresholds
#define THRESHOLDS_TOPIC SETT_TOPIC "/thresholds"
// ----Soil humidity thresholds
#define SOIL_MOIST_THR_TOPIC        THRESHOLDS_TOPIC "/soilhumidity"
#define MIN_SOIL_MOIST_THR_TOPIC    SOIL_MOIST_THR_TOPIC "/min"
#define MAX_SOIL_MOIST_THR_TOPIC    SOIL_MOIST_THR_TOPIC "/max"
// ----Air humidity thresholds
#define AIR_MOIST_THR_TOPIC         THRESHOLDS_TOPIC "/airhumidity"
#define MIN_AIR_MOIST_THR_TOPIC     AIR_MOIST_THR_TOPIC "/min"
#define MAX_AIR_MOIST_THR_TOPIC     AIR_MOIST_THR_TOPIC "/max"
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

// Preferences
#define PREFERENCES_SETTINGS_SCOPE_NAME "settings"
#define MAX_CHARS_FOR_TOPIC_KEY 2

/*--------------------------------------------------*/
/*------------------ Pin Defines -------------------*/
/*--------------------------------------------------*/
// Sensors
#define DHT11PIN 27 // Digital temperature and air humidity sensor
#define YL69PIN 34 // Analog soil humidity sensor
#define YL69ACTIVATIONPIN 16
#define BH1750CLPIN 25 // BH1750 I2C clk
#define BH1750SDAPIN 26 // BH1750 I2C data
// Actuators
#define irrigatorPIN 32 // Controls the custom PCB in charge of the irrigator management
#define lightsPIN 33 // Controls a relè designed to switch on and off a 220V growlamp

// LED signals
#define connectionLED 14 // ON as long as the system doensn't recognize connectivity issues
#define irrigatorLED 12 // ON as long as the irrigator is active
#define greenhouseStateWarningLED 13 // ON if the greenhouse is in a bad stategreenhouse

/*--------------------------------------------------*/
/*------ MQTT Default configurable parameters ------*/
/*--------------------------------------------------*/
#define DEFAULT_MinAirMoistureThreshold 10
#define DEFAULT_MaxAirMoistureThreshold 80
#define DEFAULT_MinTemperatureThreshold 15
#define DEFAULT_MaxTemperatureThreshold 45
#define DEFAULT_MinSoilMoistureThreshold 30
#define DEFAULT_MaxSoilMoistureThreshold 80
#define DEFAULT_IrrigatorActuationDuration 5000
#define DEFAULT_IrrigatorBetweenActivationsDelay 10000
#define DEFAULT_LightsActivationThreshold 30
#define DEFAULT_LightsDeactivationThreshold 80

/*--------------------------------------------------*/
/*--------- Task Periods(in milliseconds) ----------*/
/*--------------------------------------------------*/
#define CoordinatorPeriod 5000
#define MQTTPublishPeriod 2500
#define MQTTSubscribePeriod 10000
#define DHT11TemperaturePeriod 5000 // 5 sec -- Temporary debug value
#define DHT11HumidityPeriod 5000    // 5 sec -- Temporary debug value
#define YL69SoilHumidityPeriod 10000 // 5 sec -- Temporary debug value
#define LuxReadingPeriod 5000       // 5 sec -- Temporary debug value

/*--------------------------------------------------*/
/*-------------- Lights toggle delay----------------*/
/*--------------------------------------------------*/
#define lightsToggleDelay 5000

/*--------------------------------------------------*/
/*------- Type, enum and struct definitions --------*/
/*--------------------------------------------------*/
// *Type definitions*
// MISRA-C Compliance (size specific data types)
typedef float float32_t; //ESP32 boards use 32 bit for storing floats

// *Enum definitions*
enum sensor_id{
  Sensor_Id_DHT11Temperature, 
  Sensor_Id_DHT11Humidity, 
  Sensor_Id_YL69SoilHumidity, 
  Sensor_Id_Lux,
  Amount_of_sensor_ids
  };

// *Struct definitions*
struct sensor_msg{
  enum sensor_id sensor;
  float32_t sensor_reading;
};

struct MQTT_subscription{
  Adafruit_MQTT_Subscribe sub_obj;
  void (*callback) (char*, uint16_t);
  uint32_t default_setting_value;
  void* setting_variable;
};


// *MQTT Configurable parameters*

struct threshold_setting_manage_uint8_t{

  /* 
    Contains pointers to important variables used by the functions 
    that implement the protocol for updating thresholds with values fetched from MQTT.
  */
   
  uint8_t* min_thr; // actual setting variable, different from effective only inside the relative MQTT callback 
                 // in which it contains the new value fetched from MQTT broker.

  const char* min_thr_readable_name;

  uint8_t* max_thr; // actual setting variable, different from effective only inside the relative MQTT callback 
                 // in which it contains the new value fetched from MQTT broker.

  const char* max_thr_readable_name;
  uint8_t effective_min_thr;
  uint8_t effective_max_thr;

  uint8_t pending_min_thr;
  uint8_t pending_max_thr;
  bool pending_min_thr_flag;
  bool pending_max_thr_flag;

  const char* min_thr_topic_key;
  const char* max_thr_topic_key;
};

struct threshold_setting_manage_uint32_t{

  /* 
    Contains pointers to important variables used by the functions 
    that implement the protocol for updating thresholds with values fetched from MQTT.
  */
   
  uint32_t* min_thr; // actual setting variable, different from effective only inside the relative MQTT callback 
                 // in which it contains the new value fetched from MQTT broker.

  const char* min_thr_readable_name;

  uint32_t* max_thr; // actual setting variable, different from effective only inside the relative MQTT callback 
                 // in which it contains the new value fetched from MQTT broker.

  const char* max_thr_readable_name;
  uint32_t effective_min_thr;
  uint32_t effective_max_thr;

  uint32_t pending_min_thr;
  uint32_t pending_max_thr;
  bool pending_min_thr_flag;
  bool pending_max_thr_flag;

  const char* min_thr_topic_key;
  const char* max_thr_topic_key;
};

struct threshold_setting_manage_int8_t{

  /* 
    Contains pointers to important variables used by the functions 
    that implement the protocol for updating thresholds with values fetched from MQTT.
  */
   
  int8_t* min_thr; // actual setting variable, different from effective only inside the relative MQTT callback 
                 // in which it contains the new value fetched from MQTT broker.

  const char* min_thr_readable_name;

  int8_t* max_thr; // actual setting variable, different from effective only inside the relative MQTT callback 
                 // in which it contains the new value fetched from MQTT broker.

  const char* max_thr_readable_name;
  int8_t effective_min_thr;
  int8_t effective_max_thr;

  int8_t pending_min_thr;
  int8_t pending_max_thr;
  bool pending_min_thr_flag;
  bool pending_max_thr_flag;

  const char* min_thr_topic_key;
  const char* max_thr_topic_key;
};

#endif // S_GREENHOUSE_CONFIG_H -- End of the header file
