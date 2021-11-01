#ifndef S_GREENHOUSE_CONFIG_H /*== INCLUDE ==*/
#define S_GREENHOUSE_CONFIG_H /*=== GUARD ===*/

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
  #define sensorSim() ((float32_t) random(0,100))
#endif

/*--------------------------------------------------*/
/*-------------- Temporary Defines -----------------*/
/*--------------------------------------------------*/
#define SoilHumidityIrrigatorThreshold_IfBelowActivate 30 // in per cent 
#define IrrigatorExecutionTime 5000
/*--------------------------------------------------*/
/*------------------ Pin Defines -------------------*/
/*--------------------------------------------------*/
#define DHT11PIN 4 // Digital temperature and air humidity sensor
#define YL69PIN 22 // Analog soil humidity sensor

/*--------------------------------------------------*/
/*--------- Task Periods(in milliseconds) ----------*/
/*--------------------------------------------------*/
#define CoordinatorPeriod 100
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
