#include "DHT.h"
// MISRA-C Compliance (size specific data types). ESP32 boards use 32 bit for storing floats
typedef float float32_t;


#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

//#ifndef LED_BUILTIN
#define LED_BUILTIN 2
//#endif

/*--------------------------------------------------*/
/*------------------ Pin Defines ------------------*/
/*--------------------------------------------------*/
#define DHT11PIN 4

/*--------------------------------------------------*/
/*----------- Task Function Prototypes ------------*/
/*--------------------------------------------------*/
void TaskCoordinator ( void *pvParameters );
void TaskReadDHT11Temperature( void *pvParameters );
void TaskReadDHT11Humidity( void *pvParameters );

/*--------------------------------------------------*/
/*--------- Task Periods(in milliseconds) ----------*/
/*--------------------------------------------------*/
#define CoordinatorPeriod 100
#define DHT11TemperaturePeriod 100
#define DHT11HumidityPeriod 100

/*--------------------------------------------------*/
/*------------------ Task Handles ------------------*/
/*--------------------------------------------------*/
static TaskHandle_t task_handle_Coordinator = NULL;
static TaskHandle_t task_handle_ReadDHT11Temperature = NULL;
static TaskHandle_t task_handle_ReadDHT11Humidity = NULL;

/*--------------------------------------------------*/
/*------------ Queue Handle and Config -------------*/
/*--------------------------------------------------*/
QueueHandle_t coordinator_queue = NULL;
#define coordinator_queue_len 10

enum sensor_id{Sensor_Id_DHT11Temperature, Sensor_Id_DHT11Humidity};

struct sensor_msg{
  enum sensor_id sensor;
  float32_t sensor_reading;
};


//Create a DHT object called dht on the pin and with the sensor type you’ve specified previously
DHT dht(DHT11PIN, DHT11);

// the setup function runs once when you press reset or power the board
void setup() {
  
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);

  xTaskCreatePinnedToCore(
    TaskCoordinator
    ,  "TaskCoordinator"   // A name just for humans
    ,  1024  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  &task_handle_Coordinator 
    ,  ARDUINO_RUNNING_CORE);
    
  xTaskCreatePinnedToCore(
    TaskReadDHT11Temperature
    ,  "TaskReadDHT11Temperature"   
    ,  1024  
    ,  NULL
    ,  1  
    ,  &task_handle_ReadDHT11Temperature 
    ,  ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    TaskReadDHT11Humidity
    ,  "TaskReadDHT11Humidity"   
    ,  1024  
    ,  NULL
    ,  1  
    ,  &task_handle_ReadDHT11Humidity
    ,  ARDUINO_RUNNING_CORE);

  coordinator_queue = xQueueCreate(coordinator_queue_len, sizeof(struct sensor_msg));

  dht.begin();
  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop()
{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskCoordinator(void *pvParameters)
{
  (void) pvParameters;

  /*
    Task Coordinator
  */
  struct sensor_msg sensor_reading_struct;
  for (;;) 
  {
    xQueueReceive(coordinator_queue, &sensor_reading_struct, portMAX_DELAY);

    switch (sensor_reading_struct.sensor)
    {
      case Sensor_Id_DHT11Temperature:
      {
        
        break;
      }
      
      case Sensor_Id_DHT11Humidity:
      {
        
        break;
      }
    }
    vTaskDelay(CoordinatorPeriod / portTICK_PERIOD_MS);  
    
  }
}

void TaskReadDHT11Temperature(void *pvParameters)
{
  (void) pvParameters;

  /*
    Read temperature as Celsius (the default)
  */
  
  float32_t temperature_reading;
  struct sensor_msg sensor_reading_struct;
  sensor_reading_struct.sensor = Sensor_Id_DHT11Temperature;
  
  for (;;) 
  {
    temperature_reading = dht.readTemperature();
    sensor_reading_struct.sensor_reading = temperature_reading;
    
    if( xQueueSend ( coordinator_queue , 
                    (void *) &sensor_reading_struct , 
                    0) // xTicksToWait: The maximum amount of time the task should block waiting for space to become available on the queue.
                       // The call will return immediately if the queue is full and xTicksToWait is set to 0.
                    != pdPASS )    
    {
      /* Failed to post the message */
    }
                    
    vTaskDelay(DHT11TemperaturePeriod / portTICK_PERIOD_MS);  
    // Print out remaining stack memory (in words of 4 bytes)
    Serial.print("TaskReadDHT11Temperature high water mark (words): ");
    Serial.println(uxTaskGetStackHighWaterMark(NULL));
  }
}

void TaskReadDHT11Humidity(void *pvParameters)
{
  (void) pvParameters;

  /*
    Read humidity in per cent. 
    So, if the humidity is 60 per cent(which is the average humidity), then 60 per cent of the air around you is water vapor
  */
  
  float32_t humidity_reading;
  struct sensor_msg sensor_reading_struct;
  sensor_reading_struct.sensor = Sensor_Id_DHT11Humidity;
  
  for (;;) 
  {
    humidity_reading = dht.readHumidity();
    sensor_reading_struct.sensor_reading = humidity_reading;
    
    if( xQueueSend ( coordinator_queue , 
                    (void *) &sensor_reading_struct , 
                    0) // xTicksToWait: The maximum amount of time the task should block waiting for space to become available on the queue.
                       // The call will return immediately if the queue is full and xTicksToWait is set to 0.
                    != pdPASS )    
    {
      /* Failed to post the message */
    }
                    
    vTaskDelay(DHT11HumidityPeriod / portTICK_PERIOD_MS);  
    // Print out remaining stack memory (in words of 4 bytes)
    Serial.print("TaskReadDHT11Humidity high water mark (words): ");
    Serial.println(uxTaskGetStackHighWaterMark(NULL));
  }
}
