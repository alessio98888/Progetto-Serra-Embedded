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
#define DEBUG

#ifdef DEBUG

  // Connectivity
  #define WiFi_CONNECTION_VERBOSE_DEBUG
  
  // Sensors
  #define DEBUG_SENSORS // Enables sensor simulation for debug purposes (It also disables the actual sensor readings)
  #define SENSORS_VERBOSE_DEBUG // Enables verbose sensor output

  #ifdef DEBUG_SENSORS
    #define sensorSim() ((float32_t) random(0,100))
  #endif

  // Actuators
  #define DEBUG_ACTUATORS // Disables the actual activation of the actuator devices
  #define ACTUATORS_VERBOSE_DEBUG // Enables verbose atuator output

  // Memory usage
  //#define PRINT_TASK_MEMORY_USAGE
  
#endif

/*--------------------------------------------------*/
/*----------- Task priority assignation ------------*/
/*--------------------------------------------------*/
#define COORDINATOR_PRIORITY 2
#define CONNECT_WIFI_PRIORITY 4
#define MQTT_PUBLISH_PRIORITY 2
#define SENSOR_TASKS_PRIORITY 1
#define ACTUATOR_TASKS_PRIORITY 5
/*--------------------------------------------------*/
/*----------- Connectivity configuration -----------*/
/*--------------------------------------------------*/
// WiFi credentials
const char* ssid = SECRET_SSID;
const char* password = SECRET_PASS;

// WiFi connection configuration
#define ConnectionTimeDelay 5000

// MQTT broker identifiers 
const char* serverAddress = SECRET_SERVER_ADDR;
const int port = SECRET_SERVER_PORT;
/*--------------------------------------------------*/
/*-------------- Temporary Defines -----------------*/
/*--------------------------------------------------*/
#define IrrigatorActivationThreshold 30 // in per cent 
#define IrrigatorExecutionTime 5000
/*--------------------------------------------------*/
/*------------------ Pin Defines -------------------*/
/*--------------------------------------------------*/
// Sensors
#define DHT11PIN 4 // Digital temperature and air humidity sensor
#define YL69PIN 22 // Analog soil humidity sensor
// Actuators
#define irrigatorPIN 32 // Controls the custom PCB in charge of the irrigator management
/*--------------------------------------------------*/
/*--------- Task Periods(in milliseconds) ----------*/
/*--------------------------------------------------*/
#define CoordinatorPeriod 500
#define DHT11TemperaturePeriod 5000 // 5 sec -- Temporary debug value
#define DHT11HumidityPeriod 5000    // 5 sec -- Temporary debug value
#define YL69SoilHumidityPeriod 5000 // 5 sec -- Temporary debug value

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
enum sensor_id{Sensor_Id_DHT11Temperature, Sensor_Id_DHT11Humidity, Sensor_Id_YL69SoilHumidity};

// *Struct definitions*
struct sensor_msg{
  enum sensor_id sensor;
  float32_t sensor_reading;
};

#endif // S_GREENHOUSE_CONFIG_H -- End of the header file