#include "DHT.h"
#include "SGreenHouseConfig.h"

/*--------------------------------------------------*/
/*------------------ Task Handles ------------------*/
/*--------------------------------------------------*/
static TaskHandle_t task_handle_Coordinator = NULL;
static TaskHandle_t task_handle_ReadDHT11Temperature = NULL;
static TaskHandle_t task_handle_ReadDHT11Humidity = NULL;
static TaskHandle_t task_handle_ReadYL69SoilHumidity = NULL;
static TaskHandle_t task_handle_ActuatorIrrigator = NULL;

/*--------------------------------------------------*/
/*----------- Task Function Prototypes -------------*/
/*--------------------------------------------------*/
void TaskCoordinator ( void *pvParameters );
void TaskReadDHT11Temperature( void *pvParameters );
void TaskReadDHT11Humidity( void *pvParameters );
void TaskReadYL69SoilHumidity( void *pvParameters );
void TaskActuatorIrrigator( void *pvParameters );

/*--------------------------------------------------*/
/*----------------- Queue Handles ------------------*/
/*--------------------------------------------------*/
QueueHandle_t coordinator_queue = NULL;

/*--------------------------------------------------*/
/*------------ Digital sensors Config --------------*/
/*--------------------------------------------------*/
//Create a DHT object called dht on the pin and with the sensor type you’ve specified previously
DHT dht(DHT11PIN, DHT11);


// the setup function runs once when you press reset or power the board
void setup() {
  
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);

  xTaskCreatePinnedToCore(
    TaskActuatorIrrigator
    ,  "TaskActuatorIrrigator"   // A name just for humans
    ,  1024  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Higher priority for actuator
    ,  &task_handle_ActuatorIrrigator
    ,  ARDUINO_RUNNING_CORE);
    
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

   xTaskCreatePinnedToCore(
    TaskReadYL69SoilHumidity
    ,  "TaskReadYL69SoilHumidity"   
    ,  1024  
    ,  NULL
    ,  1  
    ,  &task_handle_ReadYL69SoilHumidity
    ,  ARDUINO_RUNNING_CORE);

  coordinator_queue = xQueueCreate(coordinator_queue_len, sizeof(struct sensor_msg));

  // Necessary initializations to perform sensor readings
  dht.begin();
  pinMode(YL69PIN, INPUT);

  // Necessary initializations to access actuators
  pinMode(irrigatorPIN, OUTPUT);
  
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
        
        #ifdef DEBUG_SENSORS
          #ifdef SENSORS_VERBOSE_DEBUG
            Serial.print("Temperature: ");                       
            Serial.println(sensor_reading_struct.sensor_reading);
          #endif
        #else
          // **WIP**
        #endif
        
        break;
      
      case Sensor_Id_DHT11Humidity:
        
        #ifdef DEBUG_SENSORS
          #ifdef SENSORS_VERBOSE_DEBUG
            Serial.print("Air humidity: ");
            Serial.println(sensor_reading_struct.sensor_reading);
          #endif
        #else
          // **WIP**
        #endif
        
        break;

      case Sensor_Id_YL69SoilHumidity:

        #ifdef DEBUG_SENSORS
          #ifdef SENSORS_VERBOSE_DEBUG
            Serial.print("Soil humidity: ");
            Serial.println(sensor_reading_struct.sensor_reading);
          #endif
        #else
          // **WIP**
        #endif

        // Irrigator actuation logic
        if(sensor_reading_struct.sensor_reading < IrrigatorActivationThreshold)
        {
          vTaskResume(task_handle_ActuatorIrrigator);
        }
        break;
        
      default:
        break;
    }
    
    vTaskDelay(CoordinatorPeriod / portTICK_PERIOD_MS);

    #ifdef PRINT_TASK_MEMORY_USAGE
      // Print out remaining stack memory (in words of 4 bytes)
      Serial.print("TaskCoordinator high water mark (words): ");
      Serial.println(uxTaskGetStackHighWaterMark(NULL));
    #endif
    
  }
}

void TaskReadDHT11Temperature(void *pvParameters)
{
  (void) pvParameters;

  /*
    Read temperature as Celsius (the default)
  */
  
  struct sensor_msg sensor_reading_struct;
  sensor_reading_struct.sensor = Sensor_Id_DHT11Temperature;
  
  for (;;) 
  {
    #ifdef DEBUG_SENSORS
      sensor_reading_struct.sensor_reading = sensorSim(); // DEBUG: sensor simulation
    #else
      sensor_reading_struct.sensor_reading = dht.readTemperature(); // actual sensor reading
    #endif
    
    if( xQueueSend ( coordinator_queue , 
                    (void *) &sensor_reading_struct , 
                    0) // xTicksToWait: The maximum amount of time the task should block waiting for space to become available on the queue.
                       // The call will return immediately if the queue is full and xTicksToWait is set to 0.
                    != pdPASS )    
    {
      /* Failed to post the message */
    }
                    
    vTaskDelay(DHT11TemperaturePeriod / portTICK_PERIOD_MS);

    #ifdef PRINT_TASK_MEMORY_USAGE
      // Print out remaining stack memory (in words of 4 bytes)
      Serial.print("TaskReadDHT11Temperature high water mark (words): ");
      Serial.println(uxTaskGetStackHighWaterMark(NULL));
    #endif
  }
}

void TaskReadDHT11Humidity(void *pvParameters)
{
  (void) pvParameters;
  
  /*
    Read humidity in per cent. 
    So, if the humidity is 60 per cent(which is the average humidity), then 60 per cent of the air around you is water vapor
  */
  
  struct sensor_msg sensor_reading_struct;
  sensor_reading_struct.sensor = Sensor_Id_DHT11Humidity;
  
  
  for (;;) 
  {
    
    #ifdef DEBUG_SENSORS
      sensor_reading_struct.sensor_reading = sensorSim(); // DEBUG: sensor simulation
    #else
      sensor_reading_struct.sensor_reading = dht.readHumidity(); // actual sensor reading
    #endif
    
    if( xQueueSend ( coordinator_queue , 
                    (void *) &sensor_reading_struct , 
                    0) // xTicksToWait: The maximum amount of time the task should block waiting for space to become available on the queue.
                       // The call will return immediately if the queue is full and xTicksToWait is set to 0.
                    != pdPASS )    
    {
      /* Failed to post the message */
    }
                    
    vTaskDelay(DHT11HumidityPeriod / portTICK_PERIOD_MS);

    #ifdef PRINT_TASK_MEMORY_USAGE
      // Print out remaining stack memory (in words of 4 bytes)
      Serial.print("TaskReadDHT11Humidity high water mark (words): ");
      Serial.println(uxTaskGetStackHighWaterMark(NULL));
    #endif
  }
}

void TaskReadYL69SoilHumidity(void *pvParameters)
{
  (void) pvParameters;

  /*
    Reads *soil* humidity in per cent.
  */
  
  struct sensor_msg sensor_reading_struct;
  sensor_reading_struct.sensor = Sensor_Id_YL69SoilHumidity;
  
  for (;;) 
  {
    #ifdef DEBUG_SENSORS
      sensor_reading_struct.sensor_reading = sensorSim(); // DEBUG: sensor simulation
    #else
      sensor_reading_struct.sensor_reading = map(analogRead(YL69PIN), 1023, 0, 0, 100);  // actual sensor reading
    #endif
    
    if( xQueueSend ( coordinator_queue , 
                    (void *) &sensor_reading_struct , 
                    0) // xTicksToWait: The maximum amount of time the task should block waiting for space to become available on the queue.
                       // The call will return immediately if the queue is full and xTicksToWait is set to 0.
                    != pdPASS )    
    {
      /* Failed to post the message */
    }
                    
    vTaskDelay(YL69SoilHumidityPeriod / portTICK_PERIOD_MS);

    #ifdef PRINT_TASK_MEMORY_USAGE
      // Print out remaining stack memory (in words of 4 bytes)
      Serial.print("TaskReadYL69SoilHumidity high water mark (words): ");
      Serial.println(uxTaskGetStackHighWaterMark(NULL));
    #endif
  }
}

void TaskActuatorIrrigator(void *pvParameters)
{
  (void) pvParameters;
  
  /* 
    This task executes only when the Coordinator Task decides to do so (that is when soil humidity is under certain threshold).
    This task executes only for a certain number of ticks, then it is suspended. 
  */

  // When first created, this task is suspended until it's resumed by the Coordinator
  vTaskSuspend( NULL );
  
  

  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  for(;;)
  {
    #ifdef DEBUG_ACTUATORS
      #ifdef ACTUATORS_VERBOSE_DEBUG
        Serial.println("Irrigator Activated.");
      #endif
    #else
      digitalWrite(irrigatorPIN, HIGH);                  
    #endif
    
    /* put on high the pin that starts the irrigator */

    // The task will be unblocked at time (*xLastWakeTime + (IrrigatorExecutionTime / portTICK_PERIOD_MS) ) 
    vTaskDelayUntil(&xLastWakeTime, IrrigatorExecutionTime / portTICK_PERIOD_MS);

    /* put on low the pin that starts the irrigator */

    #ifdef DEBUG_ACTUATORS
      #ifdef ACTUATORS_VERBOSE_DEBUG
        Serial.println("Irrigator Suspended.");
      #endif
    #else
      digitalWrite(irrigatorPIN, LOW);                    
    #endif

    #ifdef PRINT_TASK_MEMORY_USAGE
      // Print out remaining stack memory (in words of 4 bytes)
      Serial.print("TaskActuatorIrrigator high water mark (words): ");
      Serial.println(uxTaskGetStackHighWaterMark(NULL));
    #endif
    
    vTaskSuspend(NULL);
  } 
}
