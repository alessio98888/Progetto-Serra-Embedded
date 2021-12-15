#include "SGreenHouseConfig.h"

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
static TaskHandle_t task_handle_MQTTfetchSubscriptions = NULL;
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
void TaskMQTTfetchSubscriptions( void *pvParameters );

/*--------------------------------------------------*/
/*-------------- Function prototypes----------------*/
/*--------------------------------------------------*/
void connectWiFi();
void connectMQTT();
void MQTTQueueSend( sensor_msg sensor_reading_struct );
bool MQTTPublishMessage( Adafruit_MQTT_Publish MQTT_topic_pub, float32_t msg);
void MQTTSetSubscriptions();

//--Callback functions (subscriptions)
bool MQTT_uint32_callbackCore(uint32_t *variable_ptr, char* str, uint16_t len);
void MQTTIrrigActuationDuration_callback(char *str, uint16_t len);
void MQTTIrrigActuationDelay_callback(char *str, uint16_t len);
void MQTTOnLightThr_callback(char *str, uint16_t len);
void MQTTOffLightThr_callback(char *str, uint16_t len);

bool MQTT_uint8_callbackCore(uint8_t *variable_ptr, char* str, uint16_t len);
void MQTTMinSoilHumThr_callback(char *str, uint16_t len);
void MQTTMaxSoilHumThr_callback(char *str, uint16_t len);
void MQTTMinAirHumThr_callback(char* str, uint16_t len);
void MQTTMaxAirHumThr_callback(char* str, uint16_t len);

bool MQTT_int8_callbackCore(int8_t *variable_ptr, char* str, uint16_t len);
void MQTTMaxTempThr_callback(char *str, uint16_t len);
void MQTTMinTempThr_callback(char *str, uint16_t len);

// Safe conversion functions
bool SafeStrToInt32(char *str, uint16_t len, int32_t *int_value);
bool SafeStrToUInt32(char *str, uint16_t len, uint32_t *uint_value);

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
// ----Sensors publish
Adafruit_MQTT_Publish MQTTpub_temp = Adafruit_MQTT_Publish(&mqtt, SENS_TEMP_TOPIC);
Adafruit_MQTT_Publish MQTTpub_air_hum = Adafruit_MQTT_Publish(&mqtt, SENS_AIR_HUM_TOPIC);
Adafruit_MQTT_Publish MQTTpub_soil_hum = Adafruit_MQTT_Publish(&mqtt, SENS_SOIL_HUM_TOPIC);
Adafruit_MQTT_Publish MQTTpub_lux = Adafruit_MQTT_Publish(&mqtt, SENS_LUX_TOPIC);

// ----Settings publish
Adafruit_MQTT_Publish MQTTpub_valid_min_soil_hum_thr = Adafruit_MQTT_Publish(&mqtt, MIN_SOIL_HUM_THR_TOPIC);
Adafruit_MQTT_Publish MQTTpub_valid_max_soil_hum_thr = Adafruit_MQTT_Publish(&mqtt, MAX_SOIL_HUM_THR_TOPIC);
Adafruit_MQTT_Publish MQTTpub_valid_min_air_hum_thr = Adafruit_MQTT_Publish(&mqtt, MIN_AIR_HUM_THR_TOPIC);
Adafruit_MQTT_Publish MQTTpub_valid_max_air_hum_thr = Adafruit_MQTT_Publish(&mqtt, MAX_AIR_HUM_THR_TOPIC);
Adafruit_MQTT_Publish MQTTpub_valid_min_temp_thr = Adafruit_MQTT_Publish(&mqtt, MIN_TEMP_THR_TOPIC);
Adafruit_MQTT_Publish MQTTpub_valid_max_temp_thr = Adafruit_MQTT_Publish(&mqtt, MAX_TEMP_THR_TOPIC);
Adafruit_MQTT_Publish MQTTpub_valid_on_lights_thr = Adafruit_MQTT_Publish(&mqtt, ON_LIGHTS_THR_TOPIC);
Adafruit_MQTT_Publish MQTTpub_valid_off_lights_thr = Adafruit_MQTT_Publish(&mqtt, OFF_LIGHTS_THR_TOPIC);
Adafruit_MQTT_Publish MQTTpub_valid_irrig_act_duration = Adafruit_MQTT_Publish(&mqtt, IRRIG_ACT_DURATION_TOPIC);
Adafruit_MQTT_Publish MQTTpub_valid_irrig_act_delay = Adafruit_MQTT_Publish(&mqtt, IRRIG_ACT_DELAY_TOPIC);

// --Subscribe
MQTT_subscription MQTTsub_min_soil_hum_thr = { Adafruit_MQTT_Subscribe(&mqtt, MIN_SOIL_HUM_THR_TOPIC), MQTTMinSoilHumThr_callback };
MQTT_subscription MQTTsub_max_soil_hum_thr = { Adafruit_MQTT_Subscribe(&mqtt, MAX_SOIL_HUM_THR_TOPIC), MQTTMaxSoilHumThr_callback };
MQTT_subscription MQTTsub_min_air_hum_thr = { Adafruit_MQTT_Subscribe(&mqtt, MIN_AIR_HUM_THR_TOPIC), MQTTMinAirHumThr_callback };
MQTT_subscription MQTTsub_max_air_hum_thr = { Adafruit_MQTT_Subscribe(&mqtt, MAX_AIR_HUM_THR_TOPIC), MQTTMaxAirHumThr_callback };
MQTT_subscription MQTTsub_min_temp_thr = { Adafruit_MQTT_Subscribe(&mqtt, MIN_TEMP_THR_TOPIC), MQTTMinTempThr_callback };
MQTT_subscription MQTTsub_max_temp_thr = { Adafruit_MQTT_Subscribe(&mqtt, MAX_TEMP_THR_TOPIC), MQTTMaxTempThr_callback };
MQTT_subscription MQTTsub_on_lights_thr = { Adafruit_MQTT_Subscribe(&mqtt, ON_LIGHTS_THR_TOPIC), MQTTOnLightThr_callback };
MQTT_subscription MQTTsub_off_lights_thr = { Adafruit_MQTT_Subscribe(&mqtt, OFF_LIGHTS_THR_TOPIC), MQTTOffLightThr_callback };
MQTT_subscription MQTTsub_irrig_act_duration = { Adafruit_MQTT_Subscribe(&mqtt, IRRIG_ACT_DURATION_TOPIC), MQTTIrrigActuationDuration_callback };
MQTT_subscription MQTTsub_irrig_act_delay = { Adafruit_MQTT_Subscribe(&mqtt, IRRIG_ACT_DELAY_TOPIC), MQTTIrrigActuationDelay_callback };

MQTT_subscription *MQTT_subscription_array[MAXSUBSCRIPTIONS] = {
  &MQTTsub_min_soil_hum_thr,
  &MQTTsub_max_soil_hum_thr,
  &MQTTsub_min_air_hum_thr,
  &MQTTsub_max_air_hum_thr,
  &MQTTsub_min_temp_thr,
  &MQTTsub_max_temp_thr,
  &MQTTsub_on_lights_thr,
  &MQTTsub_off_lights_thr,
  &MQTTsub_irrig_act_duration,
  &MQTTsub_irrig_act_delay
};


/*--------------------------------------------------*/
/*---------- MQTT Configurable parameters ----------*/
/*--------------------------------------------------*/
// Soil humidity
uint8_t IrrigatorActivationThreshold = DEFAULT_IrrigatorActivationThreshold; // in per cent
uint8_t MaxSoilHumidityThreshold = DEFAULT_MaxSoilHumidityThreshold; 

// Air humidity
uint8_t MinAirHumidityThreshold = DEFAULT_MinAirHumidityThreshold; 
uint8_t MaxAirHumidityThreshold = DEFAULT_MaxAirHumidityThreshold; 

// Irrigator
uint32_t IrrigatorExecutionTime = DEFAULT_IrrigatorExecutionTime;
uint32_t IrrigatorBetweenActivationsDelay = DEFAULT_IrrigatorBetweenActivationsDelay;

// Lights
uint32_t LightsActivationThreshold = DEFAULT_LightsActivationThreshold;
uint32_t LightsDeactivationThreshold = DEFAULT_LightsDeactivationThreshold;

// Temperature
int8_t MinTemperatureThreshold = DEFAULT_MinTemperatureThreshold;
int8_t MaxTemperatureThreshold = DEFAULT_MaxTemperatureThreshold;

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

  // Signal LEDs initializations
  pinMode(connectionLED, OUTPUT);
  pinMode(irrigatorLED, OUTPUT);
  pinMode(greenhouseStateWarningLED, OUTPUT);

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
    TaskMQTTfetchSubscriptions
    ,  "TaskMQTTfetchSubscriptions"  
    ,  2024  
    ,  NULL
    ,  MQTT_FETCH_SUBSCRIPTIONS_PRIORITY
    ,  &task_handle_MQTTfetchSubscriptions
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

  // True if greenhouseStateWarningLED is ON, false otherwise
  bool ghStateWarningLED_ON = false; 
  
  // Array of flags signaling the state detected by each sensor during
  // their last read, true if it was a bad state, false otherwise 
  bool BadSensorStateArray[Amount_of_sensor_ids] = { false }; 

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
        
        // If a bad state is detected
        if ( sensor_reading_struct.sensor_reading < MinTemperatureThreshold ||\
             sensor_reading_struct.sensor_reading > MaxTemperatureThreshold   )
        {
          BadSensorStateArray[Sensor_Id_DHT11Temperature] = true;
        } // else if the bad state is over
        else if ( BadSensorStateArray[Sensor_Id_DHT11Temperature] )
          BadSensorStateArray[Sensor_Id_DHT11Temperature] = false;
        
        break;  

      case Sensor_Id_DHT11Humidity:
        
        #if DEBUG_SENSORS
          #if SENSORS_VERBOSE_DEBUG
            Serial.print("Air humidity: ");
            Serial.println(sensor_reading_struct.sensor_reading);
          #endif
        #endif

        if ( sensor_reading_struct.sensor_reading < MinAirHumidityThreshold ||\
              sensor_reading_struct.sensor_reading > MaxAirHumidityThreshold   )
        {
          BadSensorStateArray[Sensor_Id_DHT11Humidity] = true;
        }
        else if ( BadSensorStateArray[Sensor_Id_DHT11Humidity] )
          BadSensorStateArray[Sensor_Id_DHT11Humidity] = false;

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

        if ( sensor_reading_struct.sensor_reading < IrrigatorActivationThreshold ||\
              sensor_reading_struct.sensor_reading > MaxSoilHumidityThreshold       )
        {
          BadSensorStateArray[Sensor_Id_YL69SoilHumidity] = true;
        }
        else if ( BadSensorStateArray[Sensor_Id_YL69SoilHumidity] )
          BadSensorStateArray[Sensor_Id_YL69SoilHumidity] = false;
      
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

    /* 
      Update the status of the led signaling possible dangerous states of
      the greenhouse.

      If the led is off, check if there is at least a sensor which detected
      a bad state.
    */
    if ( !ghStateWarningLED_ON )
    {
      for ( uint8_t i = 0; i < Amount_of_sensor_ids; i++ )
      {
        if ( BadSensorStateArray[i] )
        {
          ghStateWarningLED_ON = true;
          digitalWrite(greenhouseStateWarningLED, HIGH);
          break;
        }
      }
    }
    else // If the led is already ON check if there are still sensors detecting
    {    // a bad state, then, if there are no more bad states, turn it OFF.
      bool bad_state = false; // 

      for ( uint8_t i = 0; i < Amount_of_sensor_ids; i++ )
      {
        if ( BadSensorStateArray[i] )
          bad_state = true;
      }

      if ( !bad_state )
      {
        ghStateWarningLED_ON = false;
        digitalWrite(greenhouseStateWarningLED, LOW);
      }
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
    digitalWrite(irrigatorLED, HIGH);

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
    
    digitalWrite(irrigatorLED, LOW);
    // This delay makes sure the task won't be reactivated before a certain amount of time
    vTaskDelay( IrrigatorBetweenActivationsDelay / portTICK_PERIOD_MS );
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

    // Wait some time before allowing to turn off the lights
    vTaskDelay(lightsToggleDelay / portTICK_PERIOD_MS);
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
    
    // Wait some time before allowing to turn the lights back on
    vTaskDelay(lightsToggleDelay / portTICK_PERIOD_MS);
    vTaskSuspend(NULL);
  } 
}

void TaskConnect( void *pvParameters )
{
  (void) pvParameters;

  MQTTSetSubscriptions(); // must be called before mqtt.connect()

  for(;;)
  {
    digitalWrite(connectionLED, LOW);

    if (WiFi.status() != WL_CONNECTED)
    {
      connectWiFi();

      connectMQTT();
    }
    else if (!mqtt.connected())
    {
      connectMQTT();
    }

    if (mqtt.connected())
      digitalWrite(connectionLED, HIGH);

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
      for( uint8_t i = 0; i < MQTT_PUBLISH_PER_EXECUTION ; i++)
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

void TaskMQTTfetchSubscriptions( void *pvParameters )
{
  (void) pvParameters;

  for(;;)
  {
    if(mqtt.ping())
    {
      mqtt.processPackets(MQTT_SUBSCRIPTION_READING_TIMEOUT);
    }
    else
    {
      mqtt.disconnect();
      vTaskResume(task_handle_Connect);
    }

    vTaskDelay(MQTTSubscribePeriod / portTICK_PERIOD_MS);
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

    for ( uint8_t i = 0; i < MAX_CONNECTION_ATTEMPTS; i++ )
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
     struct sensor_msg scrapped_msg;

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
  for( uint8_t i = 0; i < MQTT_MAX_PUBLISHING_ATTEMPTS; i++ )
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

void MQTTSetSubscriptions()
{
  for(uint8_t i = 0; i < MAXSUBSCRIPTIONS; i++)
  {
    if ( MQTT_subscription_array[i] && (MQTT_subscription_array[i]->callback) )
    {
      // Set the callback that will handle the values fetched from the topic
      (MQTT_subscription_array[i]->sub_obj).setCallback( MQTT_subscription_array[i]->callback );
      // Set the subscription on the mqtt client
      mqtt.subscribe( &(MQTT_subscription_array[i]->sub_obj) );
    }
  }
}

// ====Callbacks====

// == uint32_t ==
bool MQTT_uint32_callbackCore(uint32_t *variable_ptr, char* str, uint16_t len)
{
  uint32_t uint32_value;

  if(SafeStrToUInt32(str, len, &uint32_value))
  {
    *variable_ptr = uint32_value;
    return true;
  }
  return false;
}

void MQTTIrrigActuationDuration_callback(char* str, uint16_t len)
{
  bool success;
  success = MQTT_uint32_callbackCore(&IrrigatorExecutionTime, str, len);

  #if MQTT_FETCH_SUB_VERBOSE_DEBUG
    if(success)
    {
      printFetchedValue("IrrigatorExecutionTime", IrrigatorExecutionTime);
    }
    else
    {
      Serial.println("***__Ignoring an unvalid value for: IrrigatorExecutionTime__***");
      MQTTpub_valid_irrig_act_duration.publish(IrrigatorExecutionTime);
    }
  #else
    if(!success)
      MQTTpub_valid_irrig_act_duration.publish(IrrigatorExecutionTime);
  #endif
}

void MQTTIrrigActuationDelay_callback(char* str, uint16_t len)
{
  bool success;
  success = MQTT_uint32_callbackCore(&IrrigatorBetweenActivationsDelay, str, len);

  #if MQTT_FETCH_SUB_VERBOSE_DEBUG
    if(success)
    {
      printFetchedValue("IrrigatorBetweenActivationsDelay", IrrigatorBetweenActivationsDelay);
    }
    else
    {
      Serial.println("***__Ignoring an unvalid value for: IrrigatorBetweenActivationsDelay__***");
      MQTTpub_valid_irrig_act_delay.publish(IrrigatorBetweenActivationsDelay);
    }
  #else
    if(!success)
      MQTTpub_valid_irrig_act_delay.publish(IrrigatorBetweenActivationsDelay);
  #endif
}

void MQTTOnLightThr_callback(char* str, uint16_t len)
{
  bool success;
  success = MQTT_uint32_callbackCore(&LightsActivationThreshold, str, len);

  #if MQTT_FETCH_SUB_VERBOSE_DEBUG
    if(success)
    {
      printFetchedValue("LightsActivationThreshold", LightsActivationThreshold);
    }
    else
    {
      Serial.println("***__Ignoring an unvalid value for: LightsActivationThreshold__***");
      MQTTpub_valid_on_lights_thr.publish(LightsActivationThreshold);
    }
  #else
    if(!success)
      MQTTpub_valid_on_lights_thr.publish(LightsActivationThreshold);
  #endif
}

void MQTTOffLightThr_callback(char* str, uint16_t len)
{
  bool success;
  success = MQTT_uint32_callbackCore(&LightsDeactivationThreshold, str, len);

  #if MQTT_FETCH_SUB_VERBOSE_DEBUG
    if(success)
    {
      printFetchedValue("LightsDeactivationThreshold", LightsDeactivationThreshold);
    }
    else
    {
      Serial.println("***__Ignoring an unvalid value for: LightsDeactivationThreshold__***");
      MQTTpub_valid_off_lights_thr.publish(LightsDeactivationThreshold);
    }
  #else
    if(!success)
      MQTTpub_valid_off_lights_thr.publish(LightsDeactivationThreshold);
  #endif
}

// == uint8_t ==
bool MQTT_uint8_callbackCore(uint8_t *variable_ptr, char* str, uint16_t len)
{
  uint32_t uint32_value;

  if(SafeStrToUInt32(str, len, &uint32_value))
  {
    if(uint32_value <= UINT8_MAX)
    {
      *variable_ptr = (uint8_t) uint32_value;
      return true;
    }
  }
  return false;
}

void MQTTMaxSoilHumThr_callback(char* str, uint16_t len)
{
  bool success;
  success = MQTT_uint8_callbackCore(&MaxSoilHumidityThreshold, str, len);
  
  #if MQTT_FETCH_SUB_VERBOSE_DEBUG
    if(success)
    {
      printFetchedValue("MaxSoilHumidityThreshold", MaxSoilHumidityThreshold);
    }
    else
    {
      Serial.println("***__Ignoring an unvalid value for: MaxSoilHumidityThreshold__***");
      MQTTpub_valid_max_soil_hum_thr.publish(MaxSoilHumidityThreshold);
    }
  #else
    if(!success)
      MQTTpub_valid_max_soil_hum_thr.publish(MaxSoilHumidityThreshold);
  #endif
}

void MQTTMinSoilHumThr_callback(char* str, uint16_t len)
{
  bool success;
  success = MQTT_uint8_callbackCore(&IrrigatorActivationThreshold, str, len);
  
  #if MQTT_FETCH_SUB_VERBOSE_DEBUG
    if(success)
    {
      printFetchedValue("IrrigatorActivationThreshold", IrrigatorActivationThreshold);
    }
    else
    {
      Serial.println("***__Ignoring an unvalid value for: IrrigatorActivationThreshold__***");
      MQTTpub_valid_min_soil_hum_thr.publish(IrrigatorActivationThreshold);
    }
  #else
    if(!success)
      MQTTpub_valid_min_soil_hum_thr.publish(IrrigatorActivationThreshold);
  #endif
}

void MQTTMaxAirHumThr_callback(char* str, uint16_t len)
{
  bool success;
  success = MQTT_uint8_callbackCore(&MaxAirHumidityThreshold, str, len);
  
  #if MQTT_FETCH_SUB_VERBOSE_DEBUG
    if(success)
    {
      printFetchedValue("MaxAirHumidityThreshold", MaxAirHumidityThreshold);
    }
    else
    {
      Serial.println("***__Ignoring an unvalid value for: MaxAirHumidityThreshold__***");
      MQTTpub_valid_max_air_hum_thr.publish(MaxAirHumidityThreshold);
    }
  #else
    if(!success)
      MQTTpub_valid_max_air_hum_thr.publish(MaxAirHumidityThreshold);
  #endif
}

void MQTTMinAirHumThr_callback(char* str, uint16_t len)
{
  bool success;
  success = MQTT_uint8_callbackCore(&MinAirHumidityThreshold, str, len);
  
  #if MQTT_FETCH_SUB_VERBOSE_DEBUG
    if(success)
    {
      printFetchedValue("MinAirHumidityThreshold", MinAirHumidityThreshold);
    }
    else
    {
      Serial.println("***__Ignoring an unvalid value for: MinAirHumidityThreshold__***");
      MQTTpub_valid_min_air_hum_thr.publish(MinAirHumidityThreshold);
    }
  #else
    if(!success)
      MQTTpub_valid_min_air_hum_thr.publish(MinAirHumidityThreshold);
  #endif
}
// == int8_t ==
bool MQTT_int8_callbackCore(int8_t *variable_ptr, char* str, uint16_t len)
{
  int32_t int32_value;

  if(SafeStrToInt32(str, len, &int32_value))
  {
    if( INT8_MIN <= int32_value && int32_value <= INT8_MAX )
    {
      *variable_ptr = (int8_t) int32_value;
      return true;
    }
  }
  return false;
}

void MQTTMaxTempThr_callback(char *str, uint16_t len)
{
  #if MQTT_FETCH_SUB_VERBOSE_DEBUG
    bool success;
    success = MQTT_int8_callbackCore(&MaxTemperatureThreshold, str, len);

    if(success)
    {
      printFetchedValue("MaxTemperatureThreshold", MaxTemperatureThreshold);
    }
    else
    {
      Serial.println("***__Ignoring an unvalid value for: MaxTemperatureThreshold__***");
      MQTTpub_valid_max_temp_thr.publish(MaxTemperatureThreshold);
    }
  #else
    if(!success)
      MQTTpub_valid_max_temp_thr.publish(MaxTemperatureThreshold);
  #endif
}

void MQTTMinTempThr_callback(char *str, uint16_t len)
{
  #if MQTT_FETCH_SUB_VERBOSE_DEBUG
    bool success;
    success = MQTT_int8_callbackCore(&MinTemperatureThreshold, str, len);

    if(success)
    {
      printFetchedValue("MinTemperatureThreshold", MinTemperatureThreshold);
    }
    else
    {
      Serial.println("***__Ignoring an unvalid value for: MinTemperatureThreshold__***");
      MQTTpub_valid_min_temp_thr.publish(MinTemperatureThreshold);
    }
  #else
    if(!success)
      MQTTpub_valid_min_temp_thr.publish(MinTemperatureThreshold);
  #endif
}

// ==== Safe conversions ====

bool __isStrIntZero(char* str, uint16_t len){

  for ( uint16_t i = 0; i < len; i++ )
  {

    if (isSpace(str[i]))
      continue;
    else if (isDigit(str[i]) && str[i] == '0')
      return true;
    else
      return false;
  
  }

  return false;
}

bool SafeStrToInt32(char *str, uint16_t len, int32_t *int_value)
{
  /*
    The function returns true if it successfully managed to
    parse the string to an int32_t value (The obtained integer 
    value can be retrieved through the referenced int_value 
    parameter)

    On ESP32 the long long int type is made of two words (8 bytes,
    64 bits).
  */

  int64_t int64_value = strtoll(str, nullptr, 10);

  /* 
    Checking if the number is between the INT32 extremes
    guarantees it to be a 32 bit integer.
  */
  if ( INT32_MIN <= int64_value && int64_value <= INT32_MAX )
  {
    *int_value = (int32_t) int64_value;
    /*
      In case the value is zero, it is necessary to check
      if the value represents an actual integer zero or 
      it represents an input non convertible into an integer
    */
    if ( (*int_value) == 0 )
    {   
        return __isStrIntZero(str, len);
    }
    else
      return true;
  }
  else
    return false;
}

bool SafeStrToUInt32(char *str, uint16_t len, uint32_t *uint_value)
{
  /*
    The function returns true if it successfully managed to
    parse the string to an uint32_t value (The obtained integer 
    value can be retrieved through the referenced uint_value 
    parameter)

    On ESP32 the unsigned long long int type is made of two words 
    (8 bytes, 64 bits).
  */
  uint64_t uint64_value = strtoull(str, nullptr, 10);

  /* 
    Checking if the number is under the UINT32 maximum
    value guarantees it to be a 32 bit unsigned int.
  */
  if ( uint64_value <= UINT32_MAX )
  {
    *uint_value = (uint32_t) strtoull(str, nullptr, 10);
    
    /*
      In case the value is zero, it is necessary to check
      if the value represents an actual integer zero or 
      it represents an input non convertible into an integer
    */
    if ( (*uint_value) == 0 )
    { 
        return __isStrIntZero(str, len);
    }
    else
      return true;
  }
  else
    return false;
}
