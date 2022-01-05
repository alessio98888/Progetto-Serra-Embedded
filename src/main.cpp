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
void TaskCoordinator(void *pvParameters);
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
void loop(void);
void setup(void);
void connectWiFi(void);
void connectMQTT(void);
void MQTTQueueSend( sensor_msg sensor_reading_struct );
bool MQTTPublishMessage( Adafruit_MQTT_Publish MQTT_topic_pub, float32_t msg);
void MQTTSetSubscriptions(void);
void MQTTFetchSubscriptions(const char* preferencesScopeName, int16_t timeout);

// Protocol functions for the updating thresholds from MQTT
bool validThresholdsSetMax(intmax_t min, intmax_t max);
bool validThresholdsSetMin(intmax_t max, intmax_t min);
void init_setting_manage_structs(void);

// == uint8_t ==
void setThreshold_uint8_t(uint8_t* new_thr, 
                          const char* thr_readable_name,
                          uint8_t* effective_thr,
                          const char* other_thr_readable_name,
                          uint8_t* other_thr,
                          uint8_t* pending_thr, 
                          bool* pending_thr_flag, 
                          uint8_t* pending_other_thr, 
                          bool* pending_other_thr_flag, 
                          const char* threshold_topic_key,
                          const char* other_threshold_topic_key,
                          bool (*validThresholdsFun) (intmax_t other_thr, intmax_t thr));

void setMaxThreshold_uint8_t(struct threshold_setting_manage_uint8_t* threshold_setting_info);
void setMinThreshold_uint8_t(struct threshold_setting_manage_uint8_t* threshold_setting_info);
void init_setting_manage_struct_uint8_t(struct threshold_setting_manage_uint8_t* threshold_setting_manage, 
                                        uint8_t* min_thr,
                                        uint8_t* max_thr,
                                        const char* minThresholdReadableName,
                                        const char* maxThresholdReadableName,
                                        const char* minThresholdTopicKey,
                                        const char* maxThresholdTopicKey);
// == uint32_t ==
void setThreshold_uint32_t(uint32_t* new_thr, 
                          const char* thr_readable_name,
                          uint32_t* effective_thr,
                          const char* other_thr_readable_name,
                          uint32_t* other_thr,
                          uint32_t* pending_thr, 
                          bool* pending_thr_flag, 
                          uint32_t* pending_other_thr, 
                          bool* pending_other_thr_flag, 
                          const char* threshold_topic_key,
                          const char* other_threshold_topic_key,
                          bool (*validThresholdsFun) (intmax_t other_thr, intmax_t thr));

void setMaxThreshold_uint32_t(struct threshold_setting_manage_uint32_t* threshold_setting_info);
void setMinThreshold_uint32_t(struct threshold_setting_manage_uint32_t* threshold_setting_info);
void init_setting_manage_struct_uint32_t(struct threshold_setting_manage_uint32_t* threshold_setting_manage, 
                                        uint32_t* min_thr,
                                        uint32_t* max_thr,
                                        const char* minThresholdReadableName,
                                        const char* maxThresholdReadableName,
                                        const char* minThresholdTopicKey,
                                        const char* maxThresholdTopicKey);

// == int8_t ==
void setThreshold_int8_t(int8_t* new_thr, 
                          const char* thr_readable_name,
                          int8_t* effective_thr,
                          const char* other_thr_readable_name,
                          int8_t* other_thr,
                          int8_t* pending_thr, 
                          bool* pending_thr_flag, 
                          int8_t* pending_other_thr, 
                          bool* pending_other_thr_flag, 
                          const char* threshold_topic_key,
                          const char* other_threshold_topic_key,
                          bool (*validThresholdsFun) (intmax_t other_thr, intmax_t thr));

void setMaxThreshold_int8_t(struct threshold_setting_manage_int8_t* threshold_setting_info);
void setMinThreshold_int8_t(struct threshold_setting_manage_int8_t* threshold_setting_info);
void init_setting_manage_struct_int8_t(struct threshold_setting_manage_int8_t* threshold_setting_manage, 
                                        int8_t* min_thr,
                                        int8_t* max_thr,
                                        const char* minThresholdReadableName,
                                        const char* maxThresholdReadableName,
                                        const char* minThresholdTopicKey,
                                        const char* maxThresholdTopicKey);
// Preferences functions
void init_get_key_from_topic(void);
void get_settings_from_preferences(void);
void init_topic_keys(void);
const char* get_key_from_topic(const char* topic);

//--Callback functions (subscriptions)
bool MQTT_uint32_callbackCore(uint32_t *variable_ptr, char* str, uint16_t len);
void MQTTIrrigActuationDuration_callback(char *str, uint16_t len);
void MQTTIrrigActuationDelay_callback(char *str, uint16_t len);
void MQTTOnLightThr_callback(char *str, uint16_t len);
void MQTTOffLightThr_callback(char *str, uint16_t len);

bool MQTT_uint8_callbackCore(uint8_t *variable_ptr, char* str, uint16_t len);
void MQTTMinSoilMoistThr_callback(char *str, uint16_t len);
void MQTTMaxSoilMoistThr_callback(char *str, uint16_t len);
void MQTTMinAirMoistThr_callback(char* str, uint16_t len);
void MQTTMaxAirMoistThr_callback(char* str, uint16_t len);

bool MQTT_int8_callbackCore(int8_t *variable_ptr, char* str, uint16_t len);
void MQTTMaxTempThr_callback(char *str, uint16_t len);
void MQTTMinTempThr_callback(char *str, uint16_t len);

// Safe conversion functions
bool isStrIntZero(char* str, uint16_t len);
bool SafeStrToInt32(char *str, uint16_t len, int32_t *int_value);
bool SafeStrToUInt32(char *str, uint16_t len, uint32_t *uint_value);
void MQTTSetSubscriptions(void);
/*--------------------------------------------------*/
/*----------- Connectivity configuration -----------*/
/*--------------------------------------------------*/
// WiFi credentials
static const char* ssid = SECRET_SSID;
static const char* password = SECRET_PASS;

// MQTT server
static const char* AIO_SERVER = SECRET_SERVER_ADDR;
static const int AIO_SERVERPORT = SECRET_SERVER_PORT;

/*--------------------------------------------------*/
/*----------------- Queue Handles ------------------*/
/*--------------------------------------------------*/
static QueueHandle_t coordinator_queue = NULL;
static QueueHandle_t MQTTpub_queue = NULL;

/*--------------------------------------------------*/
/*----------------- Connectivity -------------------*/
/*--------------------------------------------------*/
// WiFi
static WiFiClient client;
// MQTT
static Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT); // client

// --Publish
// ----Sensors publish
static Adafruit_MQTT_Publish MQTTpub_temp = Adafruit_MQTT_Publish(&mqtt, SENS_TEMP_TOPIC);
static Adafruit_MQTT_Publish MQTTpub_air_hum = Adafruit_MQTT_Publish(&mqtt, SENS_AIR_MOIST_TOPIC);
static Adafruit_MQTT_Publish MQTTpub_soil_hum = Adafruit_MQTT_Publish(&mqtt, SENS_SOIL_MOIST_TOPIC);
static Adafruit_MQTT_Publish MQTTpub_lux = Adafruit_MQTT_Publish(&mqtt, SENS_LUX_TOPIC);

// ----Settings publish
static Adafruit_MQTT_Publish MQTTpub_valid_min_soil_hum_thr = Adafruit_MQTT_Publish(&mqtt, MIN_SOIL_MOIST_THR_TOPIC);
static Adafruit_MQTT_Publish MQTTpub_valid_max_soil_hum_thr = Adafruit_MQTT_Publish(&mqtt, MAX_SOIL_MOIST_THR_TOPIC);
static Adafruit_MQTT_Publish MQTTpub_valid_min_air_hum_thr = Adafruit_MQTT_Publish(&mqtt, MIN_AIR_MOIST_THR_TOPIC);
static Adafruit_MQTT_Publish MQTTpub_valid_max_air_hum_thr = Adafruit_MQTT_Publish(&mqtt, MAX_AIR_MOIST_THR_TOPIC);
static Adafruit_MQTT_Publish MQTTpub_valid_min_temp_thr = Adafruit_MQTT_Publish(&mqtt, MIN_TEMP_THR_TOPIC);
static Adafruit_MQTT_Publish MQTTpub_valid_max_temp_thr = Adafruit_MQTT_Publish(&mqtt, MAX_TEMP_THR_TOPIC);
static Adafruit_MQTT_Publish MQTTpub_valid_on_lights_thr = Adafruit_MQTT_Publish(&mqtt, ON_LIGHTS_THR_TOPIC);
static Adafruit_MQTT_Publish MQTTpub_valid_off_lights_thr = Adafruit_MQTT_Publish(&mqtt, OFF_LIGHTS_THR_TOPIC);
static Adafruit_MQTT_Publish MQTTpub_valid_irrig_act_duration = Adafruit_MQTT_Publish(&mqtt, IRRIG_ACT_DURATION_TOPIC);
static Adafruit_MQTT_Publish MQTTpub_valid_irrig_act_delay = Adafruit_MQTT_Publish(&mqtt, IRRIG_ACT_DELAY_TOPIC);


// Irrigator
static uint32_t IrrigatorActuationDuration;
static uint32_t IrrigatorBetweenActivationsDelay;
static const char* IrrigatorActuationDuration_TopicKey;
static const char* IrrigatorBetweenActivationsDelay_TopicKey;

// -- Thresholds --
// Soil humidity
static uint8_t MinSoilMoistureThreshold; // in per cent
static const char* MinSoilMoistureThreshold_TopicKey;
static uint8_t MaxSoilMoistureThreshold; 
static const char* MaxSoilMoistureThreshold_TopicKey;


// Air humidity
static uint8_t MinAirMoistureThreshold; 
static const char* MinAirMoistureThreshold_TopicKey;
static uint8_t MaxAirMoistureThreshold; 
static const char* MaxAirMoistureThreshold_TopicKey;


// Lights
static uint32_t LightsActivationThreshold; // min threshold
static const char* LightsActivationThreshold_TopicKey;
static uint32_t LightsDeactivationThreshold; // max threshold
static const char* LightsDeactivationThreshold_TopicKey;


// Temperature
static int8_t MinTemperatureThreshold;
static const char* MinTemperatureThreshold_TopicKey;
static int8_t MaxTemperatureThreshold; 
static const char* MaxTemperatureThreshold_TopicKey;


// -- Necessary structs for updating thresholds via mqtt --

// Soil humidity
static struct threshold_setting_manage_uint8_t soil_thr_manage;

// Air humidity
static struct threshold_setting_manage_uint8_t air_thr_manage;

// Lights
static struct threshold_setting_manage_uint32_t lights_thr_manage;

// Temperature
static struct threshold_setting_manage_int8_t temp_thr_manage;

// --Subscribe
static MQTT_subscription MQTTsub_min_soil_hum_thr = { Adafruit_MQTT_Subscribe(&mqtt, MIN_SOIL_MOIST_THR_TOPIC), MQTTMinSoilMoistThr_callback, DEFAULT_MinSoilMoistureThreshold, &MinSoilMoistureThreshold};
static MQTT_subscription MQTTsub_max_soil_hum_thr = { Adafruit_MQTT_Subscribe(&mqtt, MAX_SOIL_MOIST_THR_TOPIC), MQTTMaxSoilMoistThr_callback, DEFAULT_MaxSoilMoistureThreshold, &MaxSoilMoistureThreshold};
static MQTT_subscription MQTTsub_on_lights_thr = { Adafruit_MQTT_Subscribe(&mqtt, ON_LIGHTS_THR_TOPIC), MQTTOnLightThr_callback, DEFAULT_LightsActivationThreshold, &LightsActivationThreshold};
static MQTT_subscription MQTTsub_off_lights_thr = { Adafruit_MQTT_Subscribe(&mqtt, OFF_LIGHTS_THR_TOPIC), MQTTOffLightThr_callback, DEFAULT_LightsDeactivationThreshold, &LightsDeactivationThreshold};
static MQTT_subscription MQTTsub_irrig_act_duration = { Adafruit_MQTT_Subscribe(&mqtt, IRRIG_ACT_DURATION_TOPIC), MQTTIrrigActuationDuration_callback, DEFAULT_IrrigatorActuationDuration, &IrrigatorActuationDuration};
static MQTT_subscription MQTTsub_irrig_act_delay = { Adafruit_MQTT_Subscribe(&mqtt, IRRIG_ACT_DELAY_TOPIC), MQTTIrrigActuationDelay_callback, DEFAULT_IrrigatorBetweenActivationsDelay, &IrrigatorBetweenActivationsDelay};
static MQTT_subscription MQTTsub_min_air_hum_thr = { Adafruit_MQTT_Subscribe(&mqtt, MIN_AIR_MOIST_THR_TOPIC), MQTTMinAirMoistThr_callback, DEFAULT_MinAirMoistureThreshold, &MinAirMoistureThreshold };
static MQTT_subscription MQTTsub_max_air_hum_thr = { Adafruit_MQTT_Subscribe(&mqtt, MAX_AIR_MOIST_THR_TOPIC), MQTTMaxAirMoistThr_callback, DEFAULT_MaxAirMoistureThreshold, &MaxAirMoistureThreshold };
static MQTT_subscription MQTTsub_min_temp_thr = { Adafruit_MQTT_Subscribe(&mqtt, MIN_TEMP_THR_TOPIC), MQTTMinTempThr_callback, DEFAULT_MinTemperatureThreshold, &MinTemperatureThreshold };
static MQTT_subscription MQTTsub_max_temp_thr = { Adafruit_MQTT_Subscribe(&mqtt, MAX_TEMP_THR_TOPIC), MQTTMaxTempThr_callback, DEFAULT_MaxTemperatureThreshold, &MaxTemperatureThreshold};

static MQTT_subscription *MQTT_subscription_array[MAXSUBSCRIPTIONS] = {
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
/*------------ Digital sensors Config --------------*/
/*--------------------------------------------------*/
// DHT11
// Create a DHT object called dht
static DHTesp dht;
static TempAndHumidity last_DHT11_reading = { NAN, NAN };

// last_DHT11_reading mutex
static SemaphoreHandle_t xlast_DHT11_reading_Mutex = NULL;
static StaticSemaphore_t xlast_DHT11_reading_MutexBuffer;

// BH1750
static BH1750 luxsensor(0x23);
static bool luxsensor_online = false;
/*--------------------------------------------------*/
/*----------------- Boolean flags ------------------*/
/*--------------------------------------------------*/
static bool lightsOn;

static Preferences preferences;

// Contains the const char* value for the integer i for every integer i from 0 to MAXSUBSCRIPTIONS-1
static const char* int_to_charpointer[MAXSUBSCRIPTIONS];

// the setup function runs once when you press reset or power the board
void setup(void) 
{

  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  
  lightsOn = false;

  // Queue creation
  coordinator_queue = xQueueCreate((UBaseType_t) coordinator_queue_len, sizeof(struct sensor_msg));
  MQTTpub_queue = xQueueCreate((UBaseType_t) MQTTpub_queue_len, sizeof(struct sensor_msg));

  // Necessary initializations to perform sensor readings
  // DHT11
  dht.setup(DHT11PIN, DHTesp::DHT11);

  // BH1750 lux sensor
  #if DEBUG_SENSORS
    luxsensor_online = true;
    #if BH1750_LUX_SENSOR_STATUS_DEBUG
        Serial.println("___________________________BH1750 Lux sensor SIMULATION MODE");
    #endif
  #else
    if( Wire.begin(BH1750SDAPIN, BH1750CLPIN) && luxsensor.begin(BH1750::ONE_TIME_HIGH_RES_MODE_2) )
    {
      luxsensor_online = true;
      #if BH1750_LUX_SENSOR_STATUS_DEBUG
        Serial.println("___________________________BH1750 Lux sensor ONLINE");
      #endif
    }
    else
    {
      luxsensor_online = false;
      #if BH1750_LUX_SENSOR_STATUS_DEBUG
        Serial.println("___________________________BH1750 Lux sensor OFFLINE");
      #endif
    }
  #endif

  //YL69
  pinMode(YL69PIN, INPUT);
  pinMode(YL69ACTIVATIONPIN, OUTPUT);

  // last_DHT11_reading mutex creation
  xlast_DHT11_reading_Mutex = xSemaphoreCreateMutexStatic( &xlast_DHT11_reading_MutexBuffer );

  // Necessary initializations to access actuators
  pinMode(irrigatorPIN, OUTPUT);
  pinMode(lightsPIN, OUTPUT);

  // Signal LEDs initializations
  pinMode(connectionLED, OUTPUT);
  pinMode(irrigatorLED, OUTPUT);
  pinMode(greenhouseStateWarningLED, OUTPUT);

  // *Start connection phase*
  xTaskCreatePinnedToCore(
    TaskConnect
    ,  "TaskConnect"   
    ,  ( 2144 + 128 )  
    ,  NULL
    ,  CONNECT_PRIORITY
    ,  &task_handle_Connect
    ,  ARDUINO_RUNNING_CORE);

  for(uint8_t i = 0; i < (uint8_t) SETUP_MAX_CONNECTION_ATTEMPTS; i++)
  {
    if((WiFi.status() == WL_CONNECTED) && (mqtt.ping()))
    {
      break;
    }

    vTaskDelay(WiFiConnectAttemptDelay / portTICK_PERIOD_MS);
  }

  // End connection phase


  init_get_key_from_topic();

  init_topic_keys();

  get_settings_from_preferences();


  #if DEBUG_PREFERENCES
    Serial.println(" ------ START PREFERENCES DEBUG ------");
    // Serial.println("If the value from preferences equals the default value then, 
    // if there wasn't a stored value equal to the default one, the value was not fetched from preferences.");

    Serial.print("MAX_CHARS_FOR_TOPIC_KEY: ");
    Serial.println(MAX_CHARS_FOR_TOPIC_KEY);
    Serial.println();
    for(uint8_t i = 0; i < MAXSUBSCRIPTIONS; i++)
    {
      if ( MQTT_subscription_array[i] && (MQTT_subscription_array[i]->callback) )
      {
        Serial.print((MQTT_subscription_array[i]->sub_obj).topic);
        Serial.println(":");
        Serial.print("Obtained value from preferences get function: ");

        // variables with type uint8_t
        if(  (strcmp((MQTT_subscription_array[i]->sub_obj).topic, MIN_SOIL_MOIST_THR_TOPIC) == 0)
          || (strcmp((MQTT_subscription_array[i]->sub_obj).topic, MAX_SOIL_MOIST_THR_TOPIC) == 0)
          || (strcmp((MQTT_subscription_array[i]->sub_obj).topic, MAX_AIR_MOIST_THR_TOPIC) == 0)
          || (strcmp((MQTT_subscription_array[i]->sub_obj).topic, MIN_AIR_MOIST_THR_TOPIC) == 0)
          )
        {
          Serial.println(*(uint8_t*)MQTT_subscription_array[i]->setting_variable);
        }
        // int8_t
        else if(  (strcmp((MQTT_subscription_array[i]->sub_obj).topic, MAX_TEMP_THR_TOPIC) == 0)
               || (strcmp((MQTT_subscription_array[i]->sub_obj).topic, MIN_TEMP_THR_TOPIC) == 0)
          )
        {
          Serial.println(*(int8_t*)MQTT_subscription_array[i]->setting_variable);
        }
        // uint32_t 
        else
        {
          Serial.println(*(uint32_t*)MQTT_subscription_array[i]->setting_variable);
        }
        
        Serial.print("Default value: ");
        Serial.println(MQTT_subscription_array[i]->default_setting_value);
        Serial.println();
      }
    }
    Serial.println("------ END PREFERENCES DEBUG ------");
  #endif


  if(mqtt.ping() == true)
  {
    MQTTFetchSubscriptions(PREFERENCES_SETTINGS_SCOPE_NAME, MQTT_SUBSCRIPTION_READING_TIMEOUT);
  }

  xTaskCreatePinnedToCore(
    TaskCoordinator
    ,  "TaskCoordinator"   // A name just for humans
    ,  ( 504 + 128 )  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  COORDINATOR_PRIORITY  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  &task_handle_Coordinator 
    ,  ARDUINO_RUNNING_CORE);

   xTaskCreatePinnedToCore(
    TaskMQTTfetchSubscriptions
    ,  "TaskMQTTfetchSubscriptions"  
    ,  ( 1440 + 128 )
    ,  NULL
    ,  MQTT_FETCH_SUBSCRIPTIONS_PRIORITY
    ,  &task_handle_MQTTfetchSubscriptions
    ,  ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    TaskMQTTpublish
    ,  "TaskMQTTpublish"  
    ,  ( 1200 + 128 ) 
    ,  NULL
    ,  MQTT_PUBLISH_PRIORITY
    ,  &task_handle_MQTTpublish
    ,  ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    TaskReadDHT11Temperature
    ,  "TaskReadDHT11Temperature"   
    ,  ( 648 + 128 )   
    ,  NULL
    ,  SENSOR_TASKS_PRIORITY  
    ,  &task_handle_ReadDHT11Temperature 
    ,  ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    TaskReadDHT11Humidity
    ,  "TaskReadDHT11Humidity"   
    ,  ( 644 + 128 )  
    ,  NULL
    ,  SENSOR_TASKS_PRIORITY  
    ,  &task_handle_ReadDHT11Humidity
    ,  ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    TaskReadYL69SoilHumidity
    ,  "TaskReadYL69SoilHumidity"   
    ,  ( 704 + 128 )  
    ,  NULL
    ,  SENSOR_TASKS_PRIORITY  
    ,  &task_handle_ReadYL69SoilHumidity
    ,  ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    TaskReadLux
    ,  "TaskReadLux"   
    ,  ( 1676 + 128 ) 
    ,  NULL
    ,  SENSOR_TASKS_PRIORITY  
    ,  &task_handle_ReadLux
    ,  ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    TaskActuatorIrrigator
    ,  "TaskActuatorIrrigator"  
    ,  ( 496 + 128 ) 
    ,  NULL
    ,  ACTUATOR_TASKS_PRIORITY
    ,  &task_handle_ActuatorIrrigator
    ,  ARDUINO_RUNNING_CORE);

  if ( luxsensor_online )
  {
    xTaskCreatePinnedToCore(
      TaskActuatorLights
      ,  "TaskActuatorLights"  
      ,  ( 496 + 128 )    
      ,  NULL
      ,  ACTUATOR_TASKS_PRIORITY
      ,  &task_handle_ActuatorLights
      ,  ARDUINO_RUNNING_CORE);
  }

  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.

}

void loop(void)
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
  bool BadSensorStateArray[Amount_of_sensor_ids] = { 0 }; 

  TickType_t xLastActivationTime = xTaskGetTickCount();
  for (;;) 
  {
    vTaskDelayUntil(&xLastActivationTime, CoordinatorPeriod / portTICK_PERIOD_MS);
    for(uint8_t i = 0; i < (uint8_t) Amount_of_sensor_ids; i++)
    {
      #if PRINT_COORDINATOR_QUEUE_USAGE
        printQueueUsageInfo(coordinator_queue, "Coordinator queue");
      #endif

      xQueueReceive(coordinator_queue, &sensor_reading_struct, 0);

      switch (sensor_reading_struct.sensor)
      {
        case Sensor_Id_DHT11Temperature:
          
          #if SENSORS_VERBOSE_DEBUG
            Serial.print("Temperature: ");                       
            Serial.println(sensor_reading_struct.sensor_reading);
          #endif
          
          MQTTQueueSend( sensor_reading_struct );

          // If a bad state is detected
          if ( ( sensor_reading_struct.sensor_reading < MinTemperatureThreshold )
              || ( sensor_reading_struct.sensor_reading > MaxTemperatureThreshold ))
          {
            BadSensorStateArray[Sensor_Id_DHT11Temperature] = true;
          } // else if the bad state is over
          else if ( BadSensorStateArray[Sensor_Id_DHT11Temperature] )
          {
            BadSensorStateArray[Sensor_Id_DHT11Temperature] = false;
          }
          else {}
          
          break;  

        case Sensor_Id_DHT11Humidity:
        
          #if SENSORS_VERBOSE_DEBUG
            Serial.print("Air humidity: ");
            Serial.println(sensor_reading_struct.sensor_reading);
          #endif

          if ( (sensor_reading_struct.sensor_reading < MinAirMoistureThreshold)
              || (sensor_reading_struct.sensor_reading > MaxAirMoistureThreshold ))
          {
            BadSensorStateArray[Sensor_Id_DHT11Humidity] = true;
          }
          else if ( BadSensorStateArray[Sensor_Id_DHT11Humidity] )
          {
            BadSensorStateArray[Sensor_Id_DHT11Humidity] = false;
          }
          else {}

          MQTTQueueSend( sensor_reading_struct );
          
          break;

        case Sensor_Id_YL69SoilHumidity:

          #if SENSORS_VERBOSE_DEBUG
            Serial.print("Soil humidity: ");
            Serial.println(sensor_reading_struct.sensor_reading);
          #endif

          MQTTQueueSend( sensor_reading_struct );

          // Irrigator actuation logic
          if(sensor_reading_struct.sensor_reading < MinSoilMoistureThreshold)
          {
            vTaskResume(task_handle_ActuatorIrrigator);
          }

          if ( (sensor_reading_struct.sensor_reading < MinSoilMoistureThreshold)
              || (sensor_reading_struct.sensor_reading > MaxSoilMoistureThreshold))
          {
            BadSensorStateArray[Sensor_Id_YL69SoilHumidity] = true;
          }
          else if ( BadSensorStateArray[Sensor_Id_YL69SoilHumidity] )
          {
            BadSensorStateArray[Sensor_Id_YL69SoilHumidity] = false;
          }
          else {}

          break;

        case Sensor_Id_Lux:

          #if SENSORS_VERBOSE_DEBUG
            Serial.print("Lux: ");
            Serial.println(sensor_reading_struct.sensor_reading);
          #endif

          MQTTQueueSend( sensor_reading_struct );

          if ( ( sensor_reading_struct.sensor_reading != NAN ) && ( sensor_reading_struct.sensor_reading > 0 )
                && luxsensor_online )
          {
            // Lights actuation logic
            if ((sensor_reading_struct.sensor_reading < LightsActivationThreshold)
              && (lightsOn == false))
            {
              vTaskResume(task_handle_ActuatorLights); // Activate lights
            }
            else if ((sensor_reading_struct.sensor_reading > LightsDeactivationThreshold)
              && (lightsOn == true))
            {
              vTaskResume(task_handle_ActuatorLights); // Deactivate lights
            }
            else
            {
              // Do nothing
            }
          }
          else
          {
            // If the lux sensor is offline or reads NAN don't interact with the actuator task
          }

          break;

        default:
          break;
      }
      
    }
    #if SENSORS_VERBOSE_DEBUG
      Serial.println("==================================");
    #endif
    /* 
      Update the status of the led signaling possible dangerous states of
      the greenhouse.

      If the led is off, check if there is at least a sensor which detected
      a bad state.
    */
    if ( !ghStateWarningLED_ON )
    {
      for ( uint8_t i = 0; i < (uint8_t) Amount_of_sensor_ids; i++ )
      {
        if ( BadSensorStateArray[i] == true)
        {
          ghStateWarningLED_ON = true;
          digitalWrite(greenhouseStateWarningLED, HIGH);
          break;
        }
        else {}        
      }
    }
    else // If the led is already ON check if there are still sensors detecting
    {    // a bad state, then, if there are no more bad states, turn it OFF.
      bool bad_state = false; // 

      for ( uint8_t i = 0; i < (uint8_t) Amount_of_sensor_ids; i++ )
      {
        if ( BadSensorStateArray[i] == true)
        {
          bad_state = true;
        }
      }

      if ( !bad_state )
      {
        ghStateWarningLED_ON = false;
        digitalWrite(greenhouseStateWarningLED, LOW);
      }
    }
    

    #if PRINT_TASK_MEMORY_USAGE
      printStackUsageInfo("TaskCoordinator");
    #endif
    
  }
}

void TaskReadDHT11Temperature(void *pvParameters)
{
  /*
    Read temperature as Celsius (the default)
  */
  
  (void) pvParameters;

  TempAndHumidity new_DHT11_reading = { 0.0, 0.0 };
  
  struct sensor_msg sensor_reading_struct;
  sensor_reading_struct.sensor = Sensor_Id_DHT11Temperature;
  
  TickType_t xLastActivationTime = xTaskGetTickCount();
  for (;;) 
  {
    vTaskDelayUntil(&xLastActivationTime, DHT11TemperaturePeriod / portTICK_PERIOD_MS);

    #if DEBUG_SENSORS
      sensor_reading_struct.sensor_reading = sensorSim(100); // DEBUG: sensor simulation
    #else

      new_DHT11_reading = dht.getTempAndHumidity(); // The read is thread safe

      // Critical section defined with a coarse grained mutex
      xSemaphoreTake(xlast_DHT11_reading_Mutex, portMAX_DELAY);
      #if MUTEX_ACCESS_VERBOSE_DEBUG
        PRINT_MUTEX_TAKE("xlast_DHT11_reading_Mutex", "TaskReadDHT11Temperature");
      #endif

      if ( isnan(new_DHT11_reading.temperature) != true )
      {
        sensor_reading_struct.sensor_reading = new_DHT11_reading.temperature;
        last_DHT11_reading.temperature = new_DHT11_reading.temperature;
      }
      else
      {
        sensor_reading_struct.sensor_reading = last_DHT11_reading.temperature;
      }

      if( isnan(new_DHT11_reading.humidity) != true )
      {
          last_DHT11_reading.humidity = new_DHT11_reading.humidity;
      }

      #if MUTEX_ACCESS_VERBOSE_DEBUG
        PRINT_MUTEX_GIVE("xlast_DHT11_reading_Mutex", "TaskReadDHT11Temperature");
      #endif
      xSemaphoreGive(xlast_DHT11_reading_Mutex);
      // End of the critical section

    #endif
    
    if( xQueueSend ( coordinator_queue , 
                    (void *) &sensor_reading_struct , 
                    0) // xTicksToWait: The maximum amount of time the task should block waiting for space to become available on the queue.
                       // The call will return immediately if the queue is full and xTicksToWait is set to 0.
                    != pdPASS )    
    {
      /* Failed to post the message */
    }
                    

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
  TempAndHumidity new_DHT11_reading = { 0.0, 0.0 };
  
  struct sensor_msg sensor_reading_struct;
  sensor_reading_struct.sensor = Sensor_Id_DHT11Humidity;
  
  
  TickType_t xLastActivationTime = xTaskGetTickCount();
  for (;;) 
  {
    
    vTaskDelayUntil(&xLastActivationTime, DHT11HumidityPeriod / portTICK_PERIOD_MS);

    #if DEBUG_SENSORS
      sensor_reading_struct.sensor_reading = sensorSim(100); // DEBUG: sensor simulation
    #else
      new_DHT11_reading = dht.getTempAndHumidity();

      // Critical section defined with a coarse grained mutex
      xSemaphoreTake(xlast_DHT11_reading_Mutex, portMAX_DELAY);
      #if MUTEX_ACCESS_VERBOSE_DEBUG
        PRINT_MUTEX_TAKE("xlast_DHT11_reading_Mutex", "TaskReadDHT11Humidity");
      #endif

      if ( isnan(new_DHT11_reading.humidity) != true )
      {
        sensor_reading_struct.sensor_reading = new_DHT11_reading.humidity;
        last_DHT11_reading.humidity = new_DHT11_reading.humidity;
      }
      else
      {
        sensor_reading_struct.sensor_reading = last_DHT11_reading.humidity;
      }

      if( isnan(new_DHT11_reading.temperature) != true )
      {
        last_DHT11_reading.temperature = new_DHT11_reading.temperature;
      }

      #if MUTEX_ACCESS_VERBOSE_DEBUG
        PRINT_MUTEX_GIVE("xlast_DHT11_reading_Mutex", "TaskReadDHT11Humidity");
      #endif
      xSemaphoreGive(xlast_DHT11_reading_Mutex);
      // End of the critical section
    #endif
    
    if( xQueueSend ( coordinator_queue , 
                    (void *) &sensor_reading_struct , 
                    0) // xTicksToWait: The maximum amount of time the task should block waiting for space to become available on the queue.
                       // The call will return immediately if the queue is full and xTicksToWait is set to 0.
                    != pdPASS )    
    {
      /* Failed to post the message */
    }
                    

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
  
  TickType_t xLastActivationTime = xTaskGetTickCount();
  for (;;) 
  {
    vTaskDelayUntil(&xLastActivationTime, YL69SoilHumidityPeriod / portTICK_PERIOD_MS);

    #if DEBUG_SENSORS
      sensor_reading_struct.sensor_reading = sensorSim(100); // DEBUG: sensor simulation
    #else
      digitalWrite(YL69ACTIVATIONPIN, HIGH);
      vTaskDelay(YL69ACTIVATIONPIN / portTICK_PERIOD_MS);

      sensor_reading_struct.sensor_reading = 100 - (( analogRead(YL69PIN) / 4095.0 ) * 100 );  // actual sensor reading

      digitalWrite(YL69ACTIVATIONPIN, LOW);
    #endif
    
    if( xQueueSend ( coordinator_queue , 
                    (void *) &sensor_reading_struct , 
                    0) // xTicksToWait: The maximum amount of time the task should block waiting for space to become available on the queue.
                       // The call will return immediately if the queue is full and xTicksToWait is set to 0.
                    != pdPASS )    
    {
      /* Failed to post the message */
    }
                    

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
   
  TickType_t xLastActivationTime = xTaskGetTickCount();
  for (;;) 
  {
    vTaskDelayUntil(&xLastActivationTime, LuxReadingPeriod / portTICK_PERIOD_MS);

    #if DEBUG_SENSORS
      sensor_reading_struct.sensor_reading = sensorSim(100000); // DEBUG: sensor simulation
    #else

      if (luxsensor_online)
      {
        luxsensor.configure(BH1750::ONE_TIME_HIGH_RES_MODE_2);
        vTaskDelay( 200 / portTICK_PERIOD_MS );

        #if BH1750_LUX_SENSOR_MAX_READING_ATTEMPTS == 1

          if ( luxsensor.measurementReady(true) == true )
          {
            sensor_reading_struct.sensor_reading = luxsensor.readLightLevel();
          }
          else
          {
            sensor_reading_struct.sensor_reading = NAN;
          }

        #else

          if ( luxsensor.measurementReady(true) == true) // If the measurement is ready, read it immediately
          {
              sensor_reading_struct.sensor_reading = luxsensor.readLightLevel();
          }
          else // else wait for the refresh rate
          {
              vTaskDelay( 120 / portTICK_PERIOD_MS ); // the high res refresh rate is 120ms

              if ( luxsensor.measurementReady(true) == false) // If it still isn't ready give NaN value
              {
                sensor_reading_struct.sensor_reading = NAN;
              }
              else                                   // Else read the value
              {
                sensor_reading_struct.sensor_reading = luxsensor.readLightLevel();
              }
          }
            
        #endif

      }
      else
      {
        sensor_reading_struct.sensor_reading = NAN;
      }

    #endif
    
    if( xQueueSend ( coordinator_queue , 
                    (void *) &sensor_reading_struct , 
                    0) // xTicksToWait: The maximum amount of time the task should block waiting for space to become available on the queue.
                       // The call will return immediately if the queue is full and xTicksToWait is set to 0.
                    != pdPASS )    
    {
      /* Failed to post the message */
    }
                    

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
    vTaskDelay(IrrigatorActuationDuration / portTICK_PERIOD_MS);



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
    else if (mqtt.connected() == false)
    {
      connectMQTT();
    }
    else {}

    if (mqtt.connected() == true)
    {
      digitalWrite(connectionLED, HIGH);
    }
    else {}

    #if PRINT_TASK_MEMORY_USAGE
      printStackUsageInfo("TaskConnect");
    #endif

    vTaskSuspend(NULL);
  }
}


void TaskMQTTpublish( void* pvParameters )
{
  (void) pvParameters;

  struct sensor_msg sensor_reading_struct;
  TickType_t xLastActivationTime = xTaskGetTickCount();
  for (;;)
  {
    vTaskDelayUntil(&xLastActivationTime, MQTTPublishPeriod / portTICK_PERIOD_MS);
    if(mqtt.ping() == true)
    {
      for( uint8_t i = 0; i < (uint8_t) MQTT_PUBLISH_PER_EXECUTION ; i++)
      {
        if(xQueueReceive(MQTTpub_queue, &sensor_reading_struct, 0) != pdPASS)
        {
          break;
        }

        switch (sensor_reading_struct.sensor)
        {
          case Sensor_Id_DHT11Temperature:
            (void) MQTTPublishMessage(MQTTpub_temp, sensor_reading_struct.sensor_reading);
            break;
          
          case Sensor_Id_DHT11Humidity:
            (void) MQTTPublishMessage(MQTTpub_air_hum, sensor_reading_struct.sensor_reading);
            break;
          
          case Sensor_Id_YL69SoilHumidity:
            (void) MQTTPublishMessage(MQTTpub_soil_hum, sensor_reading_struct.sensor_reading);
            break;

          case Sensor_Id_Lux:
            (void) MQTTPublishMessage(MQTTpub_lux, sensor_reading_struct.sensor_reading);
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

  #if PRINT_TASK_MEMORY_USAGE
    printStackUsageInfo("TaskMQTTpublish");
  #endif

  }
}

void TaskMQTTfetchSubscriptions( void *pvParameters )
{
  (void) pvParameters;

  for(;;)
  {

    if(mqtt.ping() == true)
    {
      MQTTFetchSubscriptions(PREFERENCES_SETTINGS_SCOPE_NAME, MQTT_SUBSCRIPTION_READING_TIMEOUT);
    }
    else
    {
      mqtt.disconnect();
      vTaskResume(task_handle_Connect);
    }

    #if PRINT_TASK_MEMORY_USAGE
      printStackUsageInfo("TaskMQTTfetchSubscriptions");
    #endif

    vTaskDelay(MQTTSubscribePeriod / portTICK_PERIOD_MS);
  }
}
/*--------------------------------------------------*/
/*------------------- Functions --------------------*/
/*--------------------------------------------------*/
static char topic_keys[MAXSUBSCRIPTIONS][2];
void init_get_key_from_topic(void)
{
  /*
    Neccessary initialization for the 'get_key_from_topic' function
  */
  for(uint8_t i = 0; i < MAXSUBSCRIPTIONS; i++)
  {
    topic_keys[i][0] = (char) i;
    topic_keys[i][1] = '\0';
    int_to_charpointer[i] = (const char*) topic_keys[i];
  }
}

void init_topic_keys(void)
{
  IrrigatorActuationDuration_TopicKey = get_key_from_topic(IRRIG_ACT_DURATION_TOPIC); 
  IrrigatorBetweenActivationsDelay_TopicKey = get_key_from_topic(IRRIG_ACT_DELAY_TOPIC); 
  LightsActivationThreshold_TopicKey = get_key_from_topic(ON_LIGHTS_THR_TOPIC); 
  LightsDeactivationThreshold_TopicKey = get_key_from_topic(OFF_LIGHTS_THR_TOPIC); 
  MinSoilMoistureThreshold_TopicKey = get_key_from_topic(MIN_SOIL_MOIST_THR_TOPIC); 
  MaxSoilMoistureThreshold_TopicKey = get_key_from_topic(MAX_SOIL_MOIST_THR_TOPIC);
  MinAirMoistureThreshold_TopicKey = get_key_from_topic(MIN_AIR_MOIST_THR_TOPIC);
  MaxAirMoistureThreshold_TopicKey = get_key_from_topic(MAX_AIR_MOIST_THR_TOPIC);
  MinTemperatureThreshold_TopicKey = get_key_from_topic(MIN_TEMP_THR_TOPIC);
  MaxTemperatureThreshold_TopicKey = get_key_from_topic(MAX_TEMP_THR_TOPIC);
}

void get_settings_from_preferences(void)
{
  /*
    Loads the settings present in flash memory. 
    The settings not present in flash will receive the default value defined in "SGreenHouseConfig.h". 
  */
  preferences.begin(PREFERENCES_SETTINGS_SCOPE_NAME, true);
  IrrigatorActuationDuration = preferences.getUInt(IrrigatorActuationDuration_TopicKey, DEFAULT_IrrigatorActuationDuration); 
  IrrigatorBetweenActivationsDelay = preferences.getUInt(IrrigatorBetweenActivationsDelay_TopicKey, DEFAULT_IrrigatorBetweenActivationsDelay); 
  LightsActivationThreshold = preferences.getUInt(LightsActivationThreshold_TopicKey, DEFAULT_LightsActivationThreshold); 
  LightsDeactivationThreshold = preferences.getUInt(LightsDeactivationThreshold_TopicKey, DEFAULT_LightsDeactivationThreshold); 
  MinSoilMoistureThreshold = preferences.getUChar(MinSoilMoistureThreshold_TopicKey, DEFAULT_MinSoilMoistureThreshold); 
  MaxSoilMoistureThreshold = preferences.getUChar(MaxSoilMoistureThreshold_TopicKey, DEFAULT_MaxSoilMoistureThreshold);
  MinAirMoistureThreshold = preferences.getUChar(MinAirMoistureThreshold_TopicKey, DEFAULT_MinAirMoistureThreshold);
  MaxAirMoistureThreshold = preferences.getUChar(MaxAirMoistureThreshold_TopicKey, DEFAULT_MaxAirMoistureThreshold);
  MinTemperatureThreshold = preferences.getChar(MinTemperatureThreshold_TopicKey, DEFAULT_MinTemperatureThreshold);
  MaxTemperatureThreshold = preferences.getChar(MaxTemperatureThreshold_TopicKey, DEFAULT_MaxTemperatureThreshold);
  preferences.end();

  // Necessary inizializations for the protocol that sets thresholds with mqtt 

  init_setting_manage_struct_uint8_t(&soil_thr_manage, &MinSoilMoistureThreshold, &MaxSoilMoistureThreshold, "MinSoilMoistureThreshold", "MaxSoilMoistureThreshold", MinSoilMoistureThreshold_TopicKey, MaxSoilMoistureThreshold_TopicKey);
  init_setting_manage_struct_uint8_t(&air_thr_manage, &MinAirMoistureThreshold, &MaxAirMoistureThreshold, "MinAirMoistureThreshold", "MaxAirMoistureThreshold", MinAirMoistureThreshold_TopicKey, MaxAirMoistureThreshold_TopicKey);
  init_setting_manage_struct_uint32_t(&lights_thr_manage, &LightsActivationThreshold, &LightsDeactivationThreshold, "LightsActivationThreshold", "LightsDeactivationThreshold", LightsActivationThreshold_TopicKey, LightsDeactivationThreshold_TopicKey);
  init_setting_manage_struct_int8_t(&temp_thr_manage, &MinTemperatureThreshold, &MaxTemperatureThreshold, "MinTemperatureThreshold", "MaxTemperatureThreshold", MinTemperatureThreshold_TopicKey, MaxTemperatureThreshold_TopicKey);
}
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
}

void connectMQTT()
{
    #if MQTT_CONNECTION_VERBOSE_DEBUG
      Serial.println("Attempting to connect to the MQTT broker...");
    #endif

    int8_t ret;

    for ( uint8_t i = 0; i < (uint8_t) MAX_CONNECTION_ATTEMPTS; i++ )
    { 
      ret = mqtt.connect();
      if(ret == 0)
      {
        break;
      }
        
      #if MQTT_CONNECTION_VERBOSE_DEBUG
        Serial.println(mqtt.connectErrorString(ret));
      #endif
    
      vTaskDelay(MQTTConnectAttemptDelay / portTICK_PERIOD_MS);
    }

    #if MQTT_CONNECTION_VERBOSE_DEBUG
      if(ret == 0)
      {
        Serial.println("MQTT connected successfully!");
      }
      else
      {
        Serial.println("All MQTT connection attempts failed!");
      }
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
  for( uint8_t i = 0; i < (uint8_t) MQTT_MAX_PUBLISHING_ATTEMPTS; i++ )
  {  
    if(MQTT_topic_pub.publish(msg) == true){
      return true;
    }

    #if MQTT_PUBLISH_FAIL_VERBOSE_DEBUG
      Serial.print("Failed attempt number "); 
      Serial.print(i+(uint8_t) 1);
      Serial.println(" to publish a message!");
    #endif
  }
  return false;
}

void MQTTFetchSubscriptions(const char* preferencesScopeName, int16_t timeout)
{
  preferences.begin(preferencesScopeName, false);
  mqtt.processPackets(timeout);
  preferences.end();
}

void MQTTSetSubscriptions(void)
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

const char* get_key_from_topic(const char* topic)
{
  /* 
    Requires prior inizialization (call to 'init_get_key_from_topic()')

    Used for generating the preference key (that has to be max 15 characters long) using the name of the topic. 
    It returns the index relative to the position of the topic in the MQTT_subscription_array.
    The index returned is trasformed into a const char* variable, see the definition of the array 'int_to_charpointer' for more details.
  */
  for(uint8_t i = 0; i < MAXSUBSCRIPTIONS; i++)
  {
    if ( MQTT_subscription_array[i] && (MQTT_subscription_array[i]->callback) )
    {
      if(strcmp(MQTT_subscription_array[i]->sub_obj.topic, topic) == 0)
      {
        return int_to_charpointer[i];
      }
    }
  }
  #if DEBUG_PREFERENCES
    Serial.println("ERROR: function 'get_key_from_topic' got a non-existent topic as parameter.");
  #endif

  return NULL; 
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
  bool convertion_from_string_success;
  bool semantic_error = false;
  uint32_t effectiveIrrigatorActuationDuration = IrrigatorActuationDuration;

  convertion_from_string_success = MQTT_uint32_callbackCore(&IrrigatorActuationDuration, str, len);

  if(convertion_from_string_success == true)
  {
    if(IrrigatorActuationDuration >= (uint32_t) IRRIG_MIN_ACTUATION_DURATION)
    {
      preferences.putUInt(IrrigatorActuationDuration_TopicKey, IrrigatorActuationDuration);
    }
    else
    {
      semantic_error = true;
      IrrigatorActuationDuration = effectiveIrrigatorActuationDuration;
      #if MQTT_FETCH_SUB_VERBOSE_DEBUG
        Serial.print("Invalid value for setting IrrigatorActuationDuration: less than the permitted minimum of ");
        Serial.println(IRRIG_MIN_ACTUATION_DURATION);
      #endif
    }
  }

  #if MQTT_FETCH_SUB_VERBOSE_DEBUG
    if(convertion_from_string_success && !semantic_error)
    {
      printFetchedValue("IrrigatorActuationDuration", IrrigatorActuationDuration);
    }
    else
    {
      Serial.println("***__Ignoring an invalid value for: IrrigatorExecutionTime__***");
      MQTTpub_valid_irrig_act_duration.publish(IrrigatorActuationDuration);
    }
  #else
    if(!convertion_from_string_success && semantic_error)
    {
      MQTTpub_valid_irrig_act_duration.publish(IrrigatorActuationDuration);
    }
    else
    {
      // Do nothing, continue
    }
  #endif
}

void MQTTIrrigActuationDelay_callback(char* str, uint16_t len)
{
  bool convertion_from_string_success;
  bool semantic_error = false;
  uint32_t effectiveIrrigatorBetweenActivationsDelay = IrrigatorBetweenActivationsDelay;

  convertion_from_string_success = MQTT_uint32_callbackCore(&IrrigatorBetweenActivationsDelay, str, len);
  if(convertion_from_string_success == true)
  {
    if(IrrigatorBetweenActivationsDelay >= (uint32_t) IRRIG_MIN_ACTUATION_DELAY)
    {
      preferences.putUInt(IrrigatorBetweenActivationsDelay_TopicKey, IrrigatorBetweenActivationsDelay);
    }
    else
    {
      semantic_error = true;
      IrrigatorBetweenActivationsDelay = effectiveIrrigatorBetweenActivationsDelay;
      #if MQTT_FETCH_SUB_VERBOSE_DEBUG
        Serial.print("Invalid value for setting IrrigatorBetweenActivationsDelay: less than the permitted minimum of ");
        Serial.println(IRRIG_MIN_ACTUATION_DELAY);
      #endif
    }
  }
  #if MQTT_FETCH_SUB_VERBOSE_DEBUG
    if(convertion_from_string_success && !semantic_error)
    {
      printFetchedValue("IrrigatorBetweenActivationsDelay", IrrigatorBetweenActivationsDelay);
    }
    else
    {
      Serial.println("***__Ignoring an invalid value for: IrrigatorBetweenActivationsDelay__***");
      MQTTpub_valid_irrig_act_delay.publish(IrrigatorBetweenActivationsDelay);
    }
  #else
    if(!convertion_from_string_success && semantic_error)
    {
      MQTTpub_valid_irrig_act_delay.publish(IrrigatorBetweenActivationsDelay);
    }
    else
    {
      // Do nothing, continue
    }
  #endif
}

void MQTTOnLightThr_callback(char* str, uint16_t len)
{
  bool convertion_from_string_success;
  lights_thr_manage.effective_min_thr = LightsActivationThreshold;

  convertion_from_string_success = MQTT_uint32_callbackCore(&LightsActivationThreshold, str, len);
  if(convertion_from_string_success == true)
  {
    setMinThreshold_uint32_t(&lights_thr_manage);
  }
  #if MQTT_FETCH_SUB_VERBOSE_DEBUG
    if(convertion_from_string_success)
    {
      printFetchedValue("LightsActivationThreshold", LightsActivationThreshold);
    }
    else
    {
      Serial.println("***__Ignoring an invalid value for: LightsActivationThreshold__***");
      MQTTpub_valid_on_lights_thr.publish(LightsActivationThreshold);
    }
  #else
    if(!convertion_from_string_success)
    {
      MQTTpub_valid_on_lights_thr.publish(LightsActivationThreshold);
    }
    else
    {
      // Do nothing, continue
    }
  #endif
}

void MQTTOffLightThr_callback(char* str, uint16_t len)
{
  bool convertion_from_string_success;
  lights_thr_manage.effective_max_thr = LightsDeactivationThreshold;

  convertion_from_string_success = MQTT_uint32_callbackCore(&LightsDeactivationThreshold, str, len);
  if(convertion_from_string_success == true)
  {
    setMaxThreshold_uint32_t(&lights_thr_manage);
  }
  #if MQTT_FETCH_SUB_VERBOSE_DEBUG
    if(convertion_from_string_success)
    {
      printFetchedValue("LightsDeactivationThreshold", LightsDeactivationThreshold);
    }
    else
    {
      Serial.println("***__Ignoring an invalid value for: LightsDeactivationThreshold__***");
      MQTTpub_valid_off_lights_thr.publish(LightsDeactivationThreshold);
    }
  #else
    if(!convertion_from_string_success)
    {
      MQTTpub_valid_off_lights_thr.publish(LightsDeactivationThreshold);
    }
    else
    {
      // Do nothing, continue
    }
  #endif
}

// == uint8_t ==
bool MQTT_uint8_callbackCore(uint8_t *variable_ptr, char* str, uint16_t len)
{
  uint32_t uint32_value;

  if(SafeStrToUInt32(str, len, &uint32_value))
  {
    if(uint32_value <= (uint32_t ) UINT8_MAX)
    {
      *variable_ptr = (uint8_t) uint32_value;
      return true;
    }
  }
  return false;
}


void MQTTMaxSoilMoistThr_callback(char* str, uint16_t len)
{
  bool convertion_from_string_success;

  soil_thr_manage.effective_max_thr = MaxSoilMoistureThreshold;

  convertion_from_string_success = MQTT_uint8_callbackCore(&MaxSoilMoistureThreshold, str, len);
  
  if(convertion_from_string_success == true)
  {
    setMaxThreshold_uint8_t(&soil_thr_manage);
  }
  else
  {
    // Do nothing, continue
  }
  
  #if MQTT_FETCH_SUB_VERBOSE_DEBUG
    if(convertion_from_string_success)
    {
      printFetchedValue("MaxSoilMoistureThreshold", MaxSoilMoistureThreshold);
    }
    else
    {
      Serial.println("***__Ignoring an invalid value for: MaxSoilHumidityThreshold__***");
      MQTTpub_valid_max_soil_hum_thr.publish(MaxSoilMoistureThreshold);
    }
  #else
    if(!convertion_from_string_success)
    {
      MQTTpub_valid_max_soil_hum_thr.publish(MaxSoilMoistureThreshold);
    }
    else
    {
      // Do nothing, continue
    }
  #endif
}

void MQTTMinSoilMoistThr_callback(char* str, uint16_t len)
{
  
  bool convertion_from_string_success;

  soil_thr_manage.effective_min_thr = MinSoilMoistureThreshold;

  convertion_from_string_success = MQTT_uint8_callbackCore(&MinSoilMoistureThreshold, str, len);
  
  if(convertion_from_string_success == true)
  {
    setMinThreshold_uint8_t(&soil_thr_manage);
  }
  else
  {
    // Conversion failed
    // Do nothing, continue
  }
  
  #if MQTT_FETCH_SUB_VERBOSE_DEBUG
    if(convertion_from_string_success)
    {
      printFetchedValue("MinSoilMoistureThreshold", MinSoilMoistureThreshold);
    }
    else
    {
      Serial.println("***__Ignoring an invalid value for: MinSoilHumidityThreshold__***");
      MQTTpub_valid_min_soil_hum_thr.publish(MinSoilMoistureThreshold);
    }
  #else
    if(!convertion_from_string_success)
    {
      MQTTpub_valid_min_soil_hum_thr.publish(MinSoilMoistureThreshold);
    }
    else
    {
      // Do nothing, continue
    }
  #endif
}

void MQTTMaxAirMoistThr_callback(char* str, uint16_t len)
{
  bool convertion_from_str_success;
  air_thr_manage.effective_max_thr = MaxAirMoistureThreshold;

  convertion_from_str_success = MQTT_uint8_callbackCore(&MaxAirMoistureThreshold, str, len);
  
  if(convertion_from_str_success == true)
  {
    setMaxThreshold_uint8_t(&air_thr_manage);
  }
  else
  {
    // Do nothing, continue
  }

  #if MQTT_FETCH_SUB_VERBOSE_DEBUG
    if(convertion_from_str_success)
    {
      printFetchedValue("MaxAirMoistureThreshold", MaxAirMoistureThreshold);
    }
    else
    {
      Serial.println("***__Ignoring an invalid value for: MaxAirHumidityThreshold__***");
      MQTTpub_valid_max_air_hum_thr.publish(MaxAirMoistureThreshold);
    }
  #else
    if(!convertion_from_str_success)
    {
      MQTTpub_valid_max_air_hum_thr.publish(MaxAirMoistureThreshold);
    }
    else
    {
      // Do nothing, continue
    }
  #endif
}

void MQTTMinAirMoistThr_callback(char* str, uint16_t len)
{
  bool convertion_from_str_success;
  air_thr_manage.effective_min_thr = MinAirMoistureThreshold;

  convertion_from_str_success = MQTT_uint8_callbackCore(&MinAirMoistureThreshold, str, len);
  if(convertion_from_str_success == true)
  {
    setMinThreshold_uint8_t(&air_thr_manage);
  }
  else
  {
    // Do nothing, continue
  }
  #if MQTT_FETCH_SUB_VERBOSE_DEBUG
    if(convertion_from_str_success)
    {
      printFetchedValue("MinAirMoistureThreshold", MinAirMoistureThreshold);
    }
    else
    {
      Serial.println("***__Ignoring an invalid value for: MinAirHumidityThreshold__***");
      MQTTpub_valid_min_air_hum_thr.publish(MinAirMoistureThreshold);
    }
  #else
    if(!convertion_from_str_success)
    {
      MQTTpub_valid_min_air_hum_thr.publish(MinAirMoistureThreshold);
    }
    else
    {
      // Do nothing, continue
    }
  #endif
}
// == int8_t ==
bool MQTT_int8_callbackCore(int8_t *variable_ptr, char* str, uint16_t len)
{
  int32_t int32_value;

  if(SafeStrToInt32(str, len, &int32_value))
  {
    if( ( ((int32_t) INT8_MIN) <= int32_value) && ( int32_value <= ((int32_t) INT8_MAX )) )
    {
      *variable_ptr = (int8_t) int32_value;
      return true;
    }
  }
  return false;
}

void MQTTMaxTempThr_callback(char *str, uint16_t len)
{
  bool convertion_from_str_success;
  temp_thr_manage.effective_max_thr = MaxTemperatureThreshold;

  convertion_from_str_success = MQTT_int8_callbackCore(&MaxTemperatureThreshold, str, len);

  if(convertion_from_str_success == true)
  {
    setMaxThreshold_int8_t(&temp_thr_manage);
  }
  else
  {
    // Do nothing, continue
  }

  #if MQTT_FETCH_SUB_VERBOSE_DEBUG
    if(convertion_from_str_success)
    {
      printFetchedValue("MaxTemperatureThreshold", MaxTemperatureThreshold);
    }
    else
    {
      Serial.println("***__Ignoring an invalid value for: MaxTemperatureThreshold__***");
      MQTTpub_valid_max_temp_thr.publish(MaxTemperatureThreshold);
    }
  #else
    if(!convertion_from_str_success)
    {
      MQTTpub_valid_max_temp_thr.publish(MaxTemperatureThreshold);
    }
    else
    {
      // Do nothing, continue
    }
  #endif
}

void MQTTMinTempThr_callback(char *str, uint16_t len)
{
  bool convertion_from_str_success;
  temp_thr_manage.effective_min_thr = MinTemperatureThreshold;

  convertion_from_str_success = MQTT_int8_callbackCore(&MinTemperatureThreshold, str, len);

  if(convertion_from_str_success == true)
  {
    setMinThreshold_int8_t(&temp_thr_manage);
  }
  else
  {
    // Do nothing, continue
  }

  #if MQTT_FETCH_SUB_VERBOSE_DEBUG
    if(convertion_from_str_success)
    {
      printFetchedValue("MinTemperatureThreshold", MinTemperatureThreshold);
    }
    else
    {
      Serial.println("***__Ignoring an invalid value for: MinTemperatureThreshold__***");
      MQTTpub_valid_min_temp_thr.publish(MinTemperatureThreshold);
    }
  #else
    if(!convertion_from_str_success)
    {
      MQTTpub_valid_min_temp_thr.publish(MinTemperatureThreshold);
    }
    else
    {
      // Do nothing, continue
    }
  #endif
}

// ==== Safe conversions ====

bool isStrIntZero(char* str, uint16_t len){

  for ( uint16_t i = 0; i < len; i++ )
  {

    if (isSpace(str[i]) == true)
    {
      continue;
    }
    else if ((isDigit(str[i]) == true) && (str[i] == '0'))
    {
      return true;
    }
    else
    {
      return false;
    }
  
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
  errno = 0;
  int64_t int64_value = strtoll(str, nullptr, 10);
  if (errno != 0)
  {
    return false;
  }

  /* 
    Checking if the number is between the INT32 extremes
    guarantees it to be a 32 bit integer.
  */
  if ( ( ((int64_t) INT32_MIN) <= int64_value ) && ( int64_value <= ((int64_t) INT32_MAX) ) )
  {
    *int_value = (int32_t) int64_value;
    /*
      In case the value is zero, it is necessary to check
      if the value represents an actual integer zero or 
      it represents an input non convertible into an integer
    */
    if ( (*int_value) == 0 )
    {   
        return isStrIntZero(str, len);
    }
    else
    {
      return true;
    }
  }
  else
  {
    return false;
  }
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
  errno = 0;
  uint64_t uint64_value = strtoull(str, nullptr, 10);
  if(errno != 0)
  {
    return false;
  }
  /* 
    Checking if the number is under the UINT32 maximum
    value guarantees it to be a 32 bit unsigned int.
  */
  if (uint64_value <= (uint64_t) UINT32_MAX)
  {
    *uint_value = (uint32_t) uint64_value; 
    
    /*
      In case the value is zero, it is necessary to check
      if the value represents an actual integer zero or 
      it represents an input non convertible into an integer
    */
    if ( (*uint_value) == 0 )
    { 
        return isStrIntZero(str, len);
    }
    else
    {
      return true;
    }
  }
  else
  {
    return false;
  }
}

// ==== Functions for updating threshold with values fetched from MQTT protocol ====

bool validThresholdsSetMax(intmax_t min, intmax_t max)
{
  return min < max;
}

bool validThresholdsSetMin(intmax_t max, intmax_t min)
{
  return min < max;
}

// == uint8_t == 
void setMaxThreshold_uint8_t(struct threshold_setting_manage_uint8_t* threshold_setting_info)
{
  setThreshold_uint8_t(
    (uint8_t*) threshold_setting_info->max_thr,  
    threshold_setting_info->max_thr_readable_name,
    &threshold_setting_info->effective_max_thr,
    threshold_setting_info->min_thr_readable_name,
    (uint8_t*) threshold_setting_info->min_thr,
    &threshold_setting_info->pending_max_thr,
    &threshold_setting_info->pending_max_thr_flag,
    &threshold_setting_info->pending_min_thr,
    &threshold_setting_info->pending_min_thr_flag,
    threshold_setting_info->max_thr_topic_key,
    threshold_setting_info->min_thr_topic_key,
    validThresholdsSetMax);
}

void setMinThreshold_uint8_t(struct threshold_setting_manage_uint8_t* threshold_setting_info)
{
  setThreshold_uint8_t(
    (uint8_t*) threshold_setting_info->min_thr, 
    threshold_setting_info->min_thr_readable_name,
    &threshold_setting_info->effective_min_thr,
    threshold_setting_info->max_thr_readable_name,
    (uint8_t*) threshold_setting_info->max_thr,
    &threshold_setting_info->pending_min_thr,
    &threshold_setting_info->pending_min_thr_flag,
    &threshold_setting_info->pending_max_thr,
    &threshold_setting_info->pending_max_thr_flag,
    threshold_setting_info->min_thr_topic_key,
    threshold_setting_info->max_thr_topic_key,
    validThresholdsSetMin);
}
  
void setThreshold_uint8_t(uint8_t* new_thr, 
                          const char* thr_readable_name,
                          uint8_t* effective_thr,
                          const char* other_thr_readable_name,
                          uint8_t* other_thr,
                          uint8_t* pending_thr, 
                          bool* pending_thr_flag, 
                          uint8_t* pending_other_thr, 
                          bool* pending_other_thr_flag, 
                          const char* threshold_topic_key,
                          const char* other_threshold_topic_key,
                          bool (*validThresholdsFun) (intmax_t other_thr, intmax_t thr))
{
  bool discard = false;

  *pending_thr_flag = false;    
  if(validThresholdsFun(*other_thr, *new_thr) == true)
  {
    if(*pending_other_thr_flag == true)
    {
      if(validThresholdsFun(*pending_other_thr, *new_thr) == true)
      {
          
          #if MQTT_FETCH_SUB_VERBOSE_DEBUG
            Serial.print("Pending ");
            Serial.print(other_thr_readable_name);
            Serial.print(" accepted: ");
            Serial.println(*pending_other_thr);
          #else
            (void) other_thr_readable_name;
          #endif

          *pending_other_thr_flag = false;
          *other_thr = *pending_other_thr;
          preferences.putUChar(other_threshold_topic_key, *other_thr);
      }
      else
      {
        // Put in pending new threshold value
        discard = true;
        *pending_thr_flag = true;
        *pending_thr = *new_thr; 
        #if MQTT_FETCH_SUB_VERBOSE_DEBUG
          Serial.print("Threshold ");
          Serial.print(thr_readable_name); 
          Serial.print(" pending with value: ");
          Serial.println(*pending_thr);
        #else
          (void) thr_readable_name;
        #endif
      }
    }
    else{}

    if(discard == true)
    {
      *new_thr = *effective_thr;  // Resetting to valid value
    }
    else
    {
      // Valid new threshold value

      preferences.putUChar(threshold_topic_key, *new_thr);
    }
  }
  else
  {
    #if MQTT_FETCH_SUB_VERBOSE_DEBUG
      Serial.print("Not valid ");
      Serial.print(thr_readable_name);
      Serial.print(" threshold (with effective other threshold): ");
      Serial.println(*new_thr);
    #endif

    if( ( *pending_other_thr_flag == true ) && ( validThresholdsFun(*pending_other_thr, *new_thr) == true ) )
    {
      //*pending_thr_flag = false;
      *pending_other_thr_flag = false;

      #if MQTT_FETCH_SUB_VERBOSE_DEBUG
        Serial.print("Valid ");
        Serial.print(thr_readable_name);
        Serial.print(" (with pending other threshold): ");
        Serial.println(*new_thr);
        Serial.print("Pending other threshold accepted: ");
        Serial.println(*pending_other_thr);
      #endif
      *other_thr = *pending_other_thr;

      preferences.putUChar(other_threshold_topic_key, *other_thr);
      preferences.putUChar(threshold_topic_key, *new_thr);
  
    }
    else
    {
      // Put new thr in pending state (waiting for a valid other threshold)
      *pending_thr_flag = true;
      *pending_thr = *new_thr;
      #if MQTT_FETCH_SUB_VERBOSE_DEBUG
        Serial.print("Threshold ");
        Serial.print(thr_readable_name); 
        Serial.print(" pending with value: ");
        Serial.println(*pending_thr);
      #endif
      *new_thr = *effective_thr;  // Resetting to valid value
    }

  }
}

// == uint32_t == 
void setMaxThreshold_uint32_t(struct threshold_setting_manage_uint32_t* threshold_setting_info)
{
  setThreshold_uint32_t(
    (uint32_t*) threshold_setting_info->max_thr,  
    threshold_setting_info->max_thr_readable_name,
    &threshold_setting_info->effective_max_thr,
    threshold_setting_info->min_thr_readable_name,
    (uint32_t*) threshold_setting_info->min_thr,
    &threshold_setting_info->pending_max_thr,
    &threshold_setting_info->pending_max_thr_flag,
    &threshold_setting_info->pending_min_thr,
    &threshold_setting_info->pending_min_thr_flag,
    threshold_setting_info->max_thr_topic_key,
    threshold_setting_info->min_thr_topic_key,
    validThresholdsSetMax);
}

void setMinThreshold_uint32_t(struct threshold_setting_manage_uint32_t* threshold_setting_info)
{
  setThreshold_uint32_t(
    (uint32_t*) threshold_setting_info->min_thr, 
    threshold_setting_info->min_thr_readable_name,
    &threshold_setting_info->effective_min_thr,
    threshold_setting_info->max_thr_readable_name,
    (uint32_t*) threshold_setting_info->max_thr,
    &threshold_setting_info->pending_min_thr,
    &threshold_setting_info->pending_min_thr_flag,
    &threshold_setting_info->pending_max_thr,
    &threshold_setting_info->pending_max_thr_flag,
    threshold_setting_info->min_thr_topic_key,
    threshold_setting_info->max_thr_topic_key,
    validThresholdsSetMin);
}
  
void setThreshold_uint32_t(uint32_t* new_thr, 
                          const char* thr_readable_name,
                          uint32_t* effective_thr,
                          const char* other_thr_readable_name,
                          uint32_t* other_thr,
                          uint32_t* pending_thr, 
                          bool* pending_thr_flag, 
                          uint32_t* pending_other_thr, 
                          bool* pending_other_thr_flag, 
                          const char* threshold_topic_key,
                          const char* other_threshold_topic_key,
                          bool (*validThresholdsFun) (intmax_t other_thr, intmax_t thr))
{
  bool discard = false;

  *pending_thr_flag = false;    
  if(validThresholdsFun(*other_thr, *new_thr) == true)
  {
    if(*pending_other_thr_flag == true)
    {
      if(validThresholdsFun(*pending_other_thr, *new_thr) == true)
      {
          
          #if MQTT_FETCH_SUB_VERBOSE_DEBUG
            Serial.print("Pending ");
            Serial.print(other_thr_readable_name);
            Serial.print(" accepted: ");
            Serial.println(*pending_other_thr);
          #else
            (void) other_thr_readable_name;
          #endif

          *pending_other_thr_flag = false;
          *other_thr = *pending_other_thr;
          preferences.putUInt(other_threshold_topic_key, *other_thr);
      }
      else
      {
        // Put in pending new threshold value
        discard = true;
        *pending_thr_flag = true;
        *pending_thr = *new_thr; 
        #if MQTT_FETCH_SUB_VERBOSE_DEBUG
          Serial.print("Threshold ");
          Serial.print(thr_readable_name); 
          Serial.print(" pending with value: ");
          Serial.println(*pending_thr);
        #else
          (void) thr_readable_name;
        #endif
      }
    }
    else{}

    if(discard == true)
    {
      *new_thr = *effective_thr;  // Resetting to valid value
    }
    else
    {
      // Valid new threshold value

      preferences.putUInt(threshold_topic_key, *new_thr);
    }
  }
  else
  {
    #if MQTT_FETCH_SUB_VERBOSE_DEBUG
      Serial.print("Not valid ");
      Serial.print(thr_readable_name);
      Serial.print(" threshold (with effective other threshold): ");
      Serial.println(*new_thr);
    #endif

    if( ( *pending_other_thr_flag == true ) && ( validThresholdsFun(*pending_other_thr, *new_thr) == true ) )
    {
      //*pending_thr_flag = false;
      *pending_other_thr_flag = false;

      #if MQTT_FETCH_SUB_VERBOSE_DEBUG
        Serial.print("Valid ");
        Serial.print(thr_readable_name);
        Serial.print(" (with pending other threshold): ");
        Serial.println(*new_thr);
        Serial.print("Pending other threshold accepted: ");
        Serial.println(*pending_other_thr);
      #endif
      *other_thr = *pending_other_thr;

      preferences.putUInt(other_threshold_topic_key, *other_thr);
      preferences.putUInt(threshold_topic_key, *new_thr);
  
    }
    else
    {
      // Put new thr in pending state (waiting for a valid other threshold)
      *pending_thr_flag = true;
      *pending_thr = *new_thr;
      #if MQTT_FETCH_SUB_VERBOSE_DEBUG
        Serial.print("Threshold ");
        Serial.print(thr_readable_name); 
        Serial.print(" pending with value: ");
        Serial.println(*pending_thr);
      #endif
      *new_thr = *effective_thr;  // Resetting to valid value
    }

  }
}

// == int8_t ==
void setMaxThreshold_int8_t(struct threshold_setting_manage_int8_t* threshold_setting_info)
{
  setThreshold_int8_t(
    (int8_t*) threshold_setting_info->max_thr,  
    threshold_setting_info->max_thr_readable_name,
    &threshold_setting_info->effective_max_thr,
    threshold_setting_info->min_thr_readable_name,
    (int8_t*) threshold_setting_info->min_thr,
    &threshold_setting_info->pending_max_thr,
    &threshold_setting_info->pending_max_thr_flag,
    &threshold_setting_info->pending_min_thr,
    &threshold_setting_info->pending_min_thr_flag,
    threshold_setting_info->max_thr_topic_key,
    threshold_setting_info->min_thr_topic_key,
    validThresholdsSetMax);
}

void setMinThreshold_int8_t(struct threshold_setting_manage_int8_t* threshold_setting_info)
{
  setThreshold_int8_t(
    (int8_t*) threshold_setting_info->min_thr, 
    threshold_setting_info->min_thr_readable_name,
    &threshold_setting_info->effective_min_thr,
    threshold_setting_info->max_thr_readable_name,
    (int8_t*) threshold_setting_info->max_thr,
    &threshold_setting_info->pending_min_thr,
    &threshold_setting_info->pending_min_thr_flag,
    &threshold_setting_info->pending_max_thr,
    &threshold_setting_info->pending_max_thr_flag,
    threshold_setting_info->min_thr_topic_key,
    threshold_setting_info->max_thr_topic_key,
    validThresholdsSetMin);
}
 
void setThreshold_int8_t(int8_t* new_thr, 
                          const char* thr_readable_name,
                          int8_t* effective_thr,
                          const char* other_thr_readable_name,
                          int8_t* other_thr,
                          int8_t* pending_thr, 
                          bool* pending_thr_flag, 
                          int8_t* pending_other_thr, 
                          bool* pending_other_thr_flag, 
                          const char* threshold_topic_key,
                          const char* other_threshold_topic_key,
                          bool (*validThresholdsFun) (intmax_t other_thr, intmax_t thr))
{
  bool discard = false;

  *pending_thr_flag = false;    
  if(validThresholdsFun(*other_thr, *new_thr) == true)
  {
    if(*pending_other_thr_flag == true)
    {
      if(validThresholdsFun(*pending_other_thr, *new_thr) == true)
      {
          
          #if MQTT_FETCH_SUB_VERBOSE_DEBUG
            Serial.print("Pending ");
            Serial.print(other_thr_readable_name);
            Serial.print(" accepted: ");
            Serial.println(*pending_other_thr);
          #else
            (void) other_thr_readable_name;
          #endif

          *pending_other_thr_flag = false;
          *other_thr = *pending_other_thr;
          preferences.putChar(other_threshold_topic_key, *other_thr);
      }
      else
      {
        // Put in pending new threshold value
        discard = true;
        *pending_thr_flag = true;
        *pending_thr = *new_thr; 
        #if MQTT_FETCH_SUB_VERBOSE_DEBUG
          Serial.print("Threshold ");
          Serial.print(thr_readable_name); 
          Serial.print(" pending with value: ");
          Serial.println(*pending_thr);
        #else
          (void) thr_readable_name;
        #endif
      }
    }
    else{}

    if(discard == true)
    {
      *new_thr = *effective_thr;  // Resetting to valid value
    }
    else
    {
      // Valid new threshold value

      preferences.putChar(threshold_topic_key, *new_thr);
    }
  }
  else
  {
    #if MQTT_FETCH_SUB_VERBOSE_DEBUG
      Serial.print("Not valid ");
      Serial.print(thr_readable_name);
      Serial.print(" threshold (with effective other threshold): ");
      Serial.println(*new_thr);
    #endif

    if( ( *pending_other_thr_flag == true ) && ( validThresholdsFun(*pending_other_thr, *new_thr) == true ) )
    {
      //*pending_thr_flag = false;
      *pending_other_thr_flag = false;

      #if MQTT_FETCH_SUB_VERBOSE_DEBUG
        Serial.print("Valid ");
        Serial.print(thr_readable_name);
        Serial.print(" (with pending other threshold): ");
        Serial.println(*new_thr);
        Serial.print("Pending other threshold accepted: ");
        Serial.println(*pending_other_thr);
      #endif
      *other_thr = *pending_other_thr;

      preferences.putChar(other_threshold_topic_key, *other_thr);
      preferences.putChar(threshold_topic_key, *new_thr);
  
    }
    else
    {
      // Put new thr in pending state (waiting for a valid other threshold)
      *pending_thr_flag = true;
      *pending_thr = *new_thr;
      #if MQTT_FETCH_SUB_VERBOSE_DEBUG
        Serial.print("Threshold ");
        Serial.print(thr_readable_name); 
        Serial.print(" pending with value: ");
        Serial.println(*pending_thr);
      #endif
      *new_thr = *effective_thr;  // Resetting to valid value
    }

  }
}

void init_setting_manage_struct_uint8_t(struct threshold_setting_manage_uint8_t* threshold_setting_manage, 
                                        uint8_t* min_thr,
                                        uint8_t* max_thr,
                                        const char* minThresholdReadableName,
                                        const char* maxThresholdReadableName,
                                        const char* minThresholdTopicKey,
                                        const char* maxThresholdTopicKey)
{
  threshold_setting_manage->min_thr = min_thr;
  threshold_setting_manage->max_thr = max_thr;
  threshold_setting_manage->effective_min_thr = *min_thr;
  threshold_setting_manage->effective_max_thr = *max_thr;
  threshold_setting_manage->pending_max_thr_flag = false;
  threshold_setting_manage->pending_min_thr_flag = false;
  threshold_setting_manage->min_thr_readable_name = minThresholdReadableName;
  threshold_setting_manage->max_thr_readable_name = maxThresholdReadableName;
  threshold_setting_manage->min_thr_topic_key = minThresholdTopicKey;
  threshold_setting_manage->max_thr_topic_key = maxThresholdTopicKey;
}
void init_setting_manage_struct_uint32_t(struct threshold_setting_manage_uint32_t* threshold_setting_manage, 
                                        uint32_t* min_thr,
                                        uint32_t* max_thr,
                                        const char* minThresholdReadableName,
                                        const char* maxThresholdReadableName,
                                        const char* minThresholdTopicKey,
                                        const char* maxThresholdTopicKey)
{
  threshold_setting_manage->min_thr = min_thr;
  threshold_setting_manage->max_thr = max_thr;
  threshold_setting_manage->effective_min_thr = *min_thr;
  threshold_setting_manage->effective_max_thr = *max_thr;
  threshold_setting_manage->pending_max_thr_flag = false;
  threshold_setting_manage->pending_min_thr_flag = false;
  threshold_setting_manage->min_thr_readable_name = minThresholdReadableName;
  threshold_setting_manage->max_thr_readable_name = maxThresholdReadableName;
  threshold_setting_manage->min_thr_topic_key = minThresholdTopicKey;
  threshold_setting_manage->max_thr_topic_key = maxThresholdTopicKey;
}
void init_setting_manage_struct_int8_t(struct threshold_setting_manage_int8_t* threshold_setting_manage, 
                                        int8_t* min_thr,
                                        int8_t* max_thr,
                                        const char* minThresholdReadableName,
                                        const char* maxThresholdReadableName,
                                        const char* minThresholdTopicKey,
                                        const char* maxThresholdTopicKey)
{
  threshold_setting_manage->min_thr = min_thr;
  threshold_setting_manage->max_thr = max_thr;
  threshold_setting_manage->effective_min_thr = *min_thr;
  threshold_setting_manage->effective_max_thr = *max_thr;
  threshold_setting_manage->pending_max_thr_flag = false;
  threshold_setting_manage->pending_min_thr_flag = false;
  threshold_setting_manage->min_thr_readable_name = minThresholdReadableName;
  threshold_setting_manage->max_thr_readable_name = maxThresholdReadableName;
  threshold_setting_manage->min_thr_topic_key = minThresholdTopicKey;
  threshold_setting_manage->max_thr_topic_key = maxThresholdTopicKey;
}
