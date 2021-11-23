#include <Arduino.h>
#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include <DHT_U.h>
#include <DHT.h>
#include "SGreenHouseConfig.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#if DEBUG
  #include "SUtils.h"
#endif

/*--------------------------------------------------*/
/*------------------ Task Handles ------------------*/
/*--------------------------------------------------*/
static TaskHandle_t task_handle_Coordinator = NULL;
static TaskHandle_t task_handle_ReadDHT11Temperature = NULL;
static TaskHandle_t task_handle_ReadDHT11Humidity = NULL;
static TaskHandle_t task_handle_ReadYL69SoilHumidity = NULL;
static TaskHandle_t task_handle_ReadLux = NULL;
static TaskHandle_t task_handle_ActuatorIrrigator = NULL;
static TaskHandle_t task_handle_ActuatorLights = NULL;
static TaskHandle_t task_handle_Connect = NULL;
static TaskHandle_t task_handle_MQTTpublish = NULL;

/*--------------------------------------------------*/
/*----------- Task Function Prototypes -------------*/
/*--------------------------------------------------*/
void TaskCoordinator ( void *pvParameters );
void TaskReadDHT11Temperature( void *pvParameters );
void TaskReadDHT11Humidity( void *pvParameters );
void TaskReadYL69SoilHumidity( void *pvParameters );
void TaskReadLux(void * pvParameters);
void TaskActuatorIrrigator( void *pvParameters );
void TaskActuatorLights( void *pvParameters );
void TaskConnect( void *pvParameters );
void TaskMQTTpublish( void* pvParameters );

/*--------------------------------------------------*/
/*-------------- Function prototypes----------------*/
/*--------------------------------------------------*/
void connectWiFi();
void connectMQTT();
void MQTTQueueSend( sensor_msg sensor_reading_struct );
bool MQTTPublishMessage( Adafruit_MQTT_Publish MQTT_topic_pub, float32_t msg);

/*--------------------------------------------------*/
/*----------------- Queue Handles ------------------*/
/*--------------------------------------------------*/
QueueHandle_t coordinator_queue = NULL;
QueueHandle_t MQTTpub_queue = NULL;

/*--------------------------------------------------*/
/*----------------- Connectivity -------------------*/
/*--------------------------------------------------*/
// WiFi
WiFiClient client;
// MQTT
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT); // client
// --Publish
Adafruit_MQTT_Publish MQTTpub_temp = Adafruit_MQTT_Publish(&mqtt, SENS_TEMP_TOPIC);
Adafruit_MQTT_Publish MQTTpub_air_hum = Adafruit_MQTT_Publish(&mqtt, SENS_AIR_HUM_TOPIC);
Adafruit_MQTT_Publish MQTTpub_soil_hum = Adafruit_MQTT_Publish(&mqtt, SENS_SOIL_HUM_TOPIC);
Adafruit_MQTT_Publish MQTTpub_lux = Adafruit_MQTT_Publish(&mqtt, SENS_LUX_TOPIC);
// --Subscribe

/*--------------------------------------------------*/
/*------------ Digital sensors Config --------------*/
/*--------------------------------------------------*/
// Create a DHT object called dht on the pin and with the sensor type you’ve specified previously
DHT dht(DHT11PIN, DHT11);

/*--------------------------------------------------*/
/*----------------- Boolean flags ------------------*/
/*--------------------------------------------------*/
bool lightsOn;

// the setup function runs once when you press reset or power the board
void setup() {
  
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);

  lightsOn = false;

  // Queue creation
  coordinator_queue = xQueueCreate(coordinator_queue_len, sizeof(struct sensor_msg));
  MQTTpub_queue = xQueueCreate(MQTTpub_queue_len, sizeof(struct sensor_msg));

  // Necessary initializations to perform sensor readings
  dht.begin();
  pinMode(YL69PIN, INPUT);

  // Necessary initializations to access actuators
  pinMode(irrigatorPIN, OUTPUT);
  pinMode(lightsPIN, OUTPUT);

  xTaskCreatePinnedToCore(
    TaskCoordinator
    ,  "TaskCoordinator"   // A name just for humans
    ,  1024  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  COORDINATOR_PRIORITY  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  &task_handle_Coordinator 
    ,  ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    TaskConnect
    ,  "TaskConnect"   
    ,  5024  
    ,  NULL
    ,  CONNECT_PRIORITY
    ,  &task_handle_Connect
    ,  ARDUINO_RUNNING_CORE);
   
  xTaskCreatePinnedToCore(
    TaskMQTTpublish
    ,  "TaskMQTTpublish"  
    ,  2024  
    ,  NULL
    ,  MQTT_PUBLISH_PRIORITY
    ,  &task_handle_MQTTpublish
    ,  ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    TaskReadDHT11Temperature
    ,  "TaskReadDHT11Temperature"   
    ,  1024  
    ,  NULL
    ,  SENSOR_TASKS_PRIORITY  
    ,  &task_handle_ReadDHT11Temperature 
    ,  ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    TaskReadDHT11Humidity
    ,  "TaskReadDHT11Humidity"   
    ,  1024  
    ,  NULL
    ,  SENSOR_TASKS_PRIORITY  
    ,  &task_handle_ReadDHT11Humidity
    ,  ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    TaskReadYL69SoilHumidity
    ,  "TaskReadYL69SoilHumidity"   
    ,  1024  
    ,  NULL
    ,  SENSOR_TASKS_PRIORITY  
    ,  &task_handle_ReadYL69SoilHumidity
    ,  ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    TaskReadLux
    ,  "TaskReadLux"   
    ,  1024  
    ,  NULL
    ,  SENSOR_TASKS_PRIORITY  
    ,  &task_handle_ReadLux
    ,  ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    TaskActuatorIrrigator
    ,  "TaskActuatorIrrigator"  
    ,  1024 
    ,  NULL
    ,  ACTUATOR_TASKS_PRIORITY
    ,  &task_handle_ActuatorIrrigator
    ,  ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    TaskActuatorLights
    ,  "TaskActuatorLights"  
    ,  1024  
    ,  NULL
    ,  ACTUATOR_TASKS_PRIORITY
    ,  &task_handle_ActuatorLights
    ,  ARDUINO_RUNNING_CORE);

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
    #if PRINT_COORDINATOR_QUEUE_USAGE
      printQueueUsageInfo(coordinator_queue, "Coordinator queue");
    #endif

    xQueueReceive(coordinator_queue, &sensor_reading_struct, portMAX_DELAY);

    switch (sensor_reading_struct.sensor)
    {
      case Sensor_Id_DHT11Temperature:
        
        #if DEBUG_SENSORS
          #if SENSORS_VERBOSE_DEBUG
            Serial.print("Temperature: ");                       
            Serial.println(sensor_reading_struct.sensor_reading);
          #endif
        #endif

        MQTTQueueSend( sensor_reading_struct );

        break;
      
      case Sensor_Id_DHT11Humidity:
        
        #if DEBUG_SENSORS
          #if SENSORS_VERBOSE_DEBUG
            Serial.print("Air humidity: ");
            Serial.println(sensor_reading_struct.sensor_reading);
          #endif
        #endif

        MQTTQueueSend( sensor_reading_struct );
        
        break;

      case Sensor_Id_YL69SoilHumidity:

        #if DEBUG_SENSORS
          #if SENSORS_VERBOSE_DEBUG
            Serial.print("Soil humidity: ");
            Serial.println(sensor_reading_struct.sensor_reading);
          #endif
        #endif

        MQTTQueueSend( sensor_reading_struct );

        // Irrigator actuation logic
        if(sensor_reading_struct.sensor_reading < IrrigatorActivationThreshold)
        {
          vTaskResume(task_handle_ActuatorIrrigator);
        }
        break;

      case Sensor_Id_Lux:

        #if DEBUG_SENSORS
          #if SENSORS_VERBOSE_DEBUG
            Serial.print("Lux: ");
            Serial.println(sensor_reading_struct.sensor_reading);
          #endif
        #endif

        MQTTQueueSend( sensor_reading_struct );

        // Lights actuation logic
        if (sensor_reading_struct.sensor_reading < LightsActivationThreshold 
          && lightsOn == false)
        {
          vTaskResume(task_handle_ActuatorLights); // Activate lights
        }
        else if (sensor_reading_struct.sensor_reading > LightsDeactivationThreshold 
          && lightsOn == true)
        {
          vTaskResume(task_handle_ActuatorLights); // Deactivate lights
        }
        break;

      default:
        break;
    }
    
    vTaskDelay(CoordinatorPeriod / portTICK_PERIOD_MS);

    #if PRINT_TASK_MEMORY_USAGE
      printStackUsageInfo("TaskCoordinator");
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
    #if DEBUG_SENSORS
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

    #if PRINT_TASK_MEMORY_USAGE
      printStackUsageInfo("TaskReadDHT11Temperature");
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
    
    #if DEBUG_SENSORS
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

    #if PRINT_TASK_MEMORY_USAGE
      printStackUsageInfo("TaskReadDHT11Humidity");
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
    #if DEBUG_SENSORS
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

    #if PRINT_TASK_MEMORY_USAGE
      printStackUsageInfo("TaskReadYL69SoilHumidity");
    #endif
  }
}

void TaskReadLux(void *pvParameters)
{
  (void) pvParameters;

  /*
    Reads the amount of ambient light.  
  */
  
  struct sensor_msg sensor_reading_struct;
  sensor_reading_struct.sensor = Sensor_Id_Lux;
  
  for (;;) 
  {
    #if DEBUG_SENSORS
      sensor_reading_struct.sensor_reading = sensorSim(); // DEBUG: sensor simulation
    #else
      // actual sensor reading
    #endif
    
    if( xQueueSend ( coordinator_queue , 
                    (void *) &sensor_reading_struct , 
                    0) // xTicksToWait: The maximum amount of time the task should block waiting for space to become available on the queue.
                       // The call will return immediately if the queue is full and xTicksToWait is set to 0.
                    != pdPASS )    
    {
      /* Failed to post the message */
    }
                    
    vTaskDelay(LuxReadingPeriod / portTICK_PERIOD_MS);

    #if PRINT_TASK_MEMORY_USAGE
      printStackUsageInfo("TaskReadLux");
    #endif
  }
}

void TaskActuatorIrrigator(void *pvParameters)
{
  (void) pvParameters;
  
  /* 
    This task executes only when the Coordinator Task decides to do so (that is when soil humidity is under certain threshold).
  */

  // When first created, this task is suspended until it's resumed by the Coordinator
  vTaskSuspend( NULL );
  
  
  for(;;)
  {  
    #if DEBUG_ACTUATORS
      #if ACTUATORS_VERBOSE_DEBUG
        Serial.println("***Irrigator Activated***");
      #endif
    #else
      digitalWrite(irrigatorPIN, HIGH);                  
    #endif


    // The task will be unblocked after the specified amount of time (this represents the amount of time spent irrigating)
    vTaskDelay(IrrigatorExecutionTime / portTICK_PERIOD_MS);


    #if DEBUG_ACTUATORS
      #if ACTUATORS_VERBOSE_DEBUG
        Serial.println("***Irrigator Suspended***");
      #endif
    #else
      digitalWrite(irrigatorPIN, LOW);                    
    #endif
    

    #if PRINT_TASK_MEMORY_USAGE
      printStackUsageInfo("TaskActuatorIrrigator");
    #endif
    
    vTaskSuspend(NULL);
  } 
}

void TaskActuatorLights(void *pvParameters)
{
  (void) pvParameters;
  
  /* 
    This task executes only when the Coordinator Task decides to do so (that is when the amount of ambient light is under certain threshold).
  */

  // When first created, this task is suspended until it's resumed by the Coordinator
  vTaskSuspend( NULL );
  
  
  for(;;)
  {  
    #if DEBUG_ACTUATORS
      #if ACTUATORS_VERBOSE_DEBUG
        Serial.println("***Lights Activated***");
      #endif
    #else
      digitalWrite(lightsPIN, HIGH);                  
    #endif

    lightsOn = true;

    vTaskSuspend( NULL );

    lightsOn = false;

    #if DEBUG_ACTUATORS
      #if ACTUATORS_VERBOSE_DEBUG
        Serial.println("***Lights Suspended***");
      #endif
    #else
      digitalWrite(lightsPIN, LOW);                    
    #endif
    

    #if PRINT_TASK_MEMORY_USAGE
      printStackUsageInfo("TaskActuatorLights");
    #endif
    
    vTaskSuspend(NULL);
  } 
}

void TaskConnect( void *pvParameters )
{
  (void) pvParameters;


  for(;;)
  {
    if (WiFi.status() != WL_CONNECTED)
    {
      connectWiFi();
      connectMQTT();
    }
    else if (!mqtt.connected())
    {
      connectMQTT();
    }

    vTaskSuspend(NULL);
  }
}

void TaskMQTTpublish( void* pvParameters )
{
  (void) pvParameters;

  struct sensor_msg sensor_reading_struct;

  for (;;)
  {
    if(mqtt.ping())
    {
      for( int i = 0; i < MQTT_PUBLISH_PER_EXECUTION ; i++)
      {
        if(xQueueReceive(MQTTpub_queue, &sensor_reading_struct, portMAX_DELAY) != pdPASS)
          break;

        switch (sensor_reading_struct.sensor)
        {
          case Sensor_Id_DHT11Temperature:
            MQTTPublishMessage(MQTTpub_temp, sensor_reading_struct.sensor_reading);
            break;
          
          case Sensor_Id_DHT11Humidity:
            MQTTPublishMessage(MQTTpub_air_hum, sensor_reading_struct.sensor_reading);
            break;
          
          case Sensor_Id_YL69SoilHumidity:
            MQTTPublishMessage(MQTTpub_soil_hum, sensor_reading_struct.sensor_reading);
            break;

          case Sensor_Id_Lux:
            MQTTPublishMessage(MQTTpub_lux, sensor_reading_struct.sensor_reading);
            break;

          default:
            break;
        }
      }
    }
    else
    {
      mqtt.disconnect();
      vTaskResume(task_handle_Connect);
    }

    vTaskDelay(MQTTPublishPeriod / portTICK_PERIOD_MS);
  }
}
/*--------------------------------------------------*/
/*------------------- Functions --------------------*/
/*--------------------------------------------------*/

void connectWiFi()
{
  wl_status_t status = WL_DISCONNECTED; // set the known WiFi status to disconneted

  while ( status != WL_CONNECTED) 
  { 
    
    #if WiFi_CONNECTION_VERBOSE_DEBUG
      Serial.print("Attempting to connect to WEP network, SSID: ");
      Serial.println(ssid);
    #endif

    // start a connection attempt
    status = WiFi.begin(ssid, password);
    
    // wait a set amount of time for connection:
    vTaskDelay(WiFiConnectAttemptDelay / portTICK_PERIOD_MS);
  }

  #if WiFi_CONNECTION_VERBOSE_DEBUG
    Serial.print("SUCCESSFULLY CONNECTED TO: ");
    Serial.println(ssid);
  #endif

  #if PRINT_TASK_MEMORY_USAGE
    printStackUsageInfo("TaskConnectWiFi");
  #endif
}

void connectMQTT()
{
    #if MQTT_CONNECTION_VERBOSE_DEBUG
      Serial.println("Attempting to connect to the MQTT broker...");
    #endif

    int8_t ret;

    for ( int i = 0; i < MAX_CONNECTION_ATTEMPTS; i++ )
    { 
      ret = mqtt.connect();
      if(ret == 0)
        break;
        
      #if MQTT_CONNECTION_VERBOSE_DEBUG
        Serial.println(mqtt.connectErrorString(ret));
      #endif
    
      vTaskDelay(MQTTConnectAttemptDelay / portTICK_PERIOD_MS);
    }

    #if MQTT_CONNECTION_VERBOSE_DEBUG
      if(ret == 0)
        Serial.println("MQTT connected successfully!");
      else
        Serial.println("All MQTT connection attempts failed!");
    #endif
}

void MQTTQueueSend( sensor_msg sensor_reading_struct )
{
  if (uxQueueSpacesAvailable( MQTTpub_queue ) == 0 )
   {
     sensor_msg scrapped_msg;

     xQueueReceive(MQTTpub_queue, &scrapped_msg, portMAX_DELAY);

     #if MQTT_QUEUE_VERBOSE_REMOVE_DEBUG
      Serial.println("Eliminated an old message from the MQTT queue!");
      Serial.print("Number of available spaces after the elimination: ");
      Serial.println(uxQueueSpacesAvailable( MQTTpub_queue ));
     #endif
   }

  if( xQueueSend ( MQTTpub_queue , 
          (void *) &sensor_reading_struct , 
          0) // xTicksToWait: The maximum amount of time the task should block waiting for space to become available on the queue.
              // The call will return immediately if the queue is full and xTicksToWait is set to 0.
          != pdPASS )    
  {
    /* Failed to post the message */
  }
}

bool MQTTPublishMessage( Adafruit_MQTT_Publish MQTT_topic_pub, float32_t msg)
{
  for( int i = 0; i < MQTT_MAX_PUBLISHING_ATTEMPTS; i++ )
  {  
    if(MQTT_topic_pub.publish(msg))
      return true;

    #if MQTT_PUBLISH_FAIL_VERBOSE_DEBUG
      Serial.print("Failed attempt number "); 
      Serial.print(i+1);
      Serial.println(" to publish a message!");
    #endif
  }
  return false;
}
