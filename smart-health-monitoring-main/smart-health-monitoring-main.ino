/*
IoT Final Project
Smart Health Monitoring System for Real-time Heart Rate and Body Temperature Tracking

Group B3
  Lauren Christy Tanudjaja				      (2106707870)
  Mikhael Morris Hapataran Siallagan		(2106731491)
  Rafi Fauzan Wirasena				        	(2106656320)
  Zalfy Putra Rezky					            (2106731453)
*/

#include <Wire.h>
#include <WiFi.h>
#include <ThingsBoard.h>
#include "Arduino_MQTT_Client.h"
#include "MAX30100_PulseOximeter.h"


// ThingsBoard configuration
#define TB_SERVER             "thingsboard.cloud"       // Enter ThingsBoard server instance
#define TB_TOKEN              "xSvHRJysdZ8Czi7RhBn2"    // Enter your Device Access Token
#define REPORTING_PERIOD_MS   1000                      // Increase reporting period to avoid sending data too frequently
#define ADC_VREF_mV           3300.0                    // in millivolt
#define ADC_RESOLUTION        4096.0                    // 12-bit ADC
#define PIN_LM35              35                        // ESP32 pin GPIO36 (ADC0) connected to LM35

float BPM, SpO2;

struct Temp {
  float tempC;
  float tempF;
};

QueueHandle_t tempQueue;
SemaphoreHandle_t oximeterMutex;

// Put your SSID & Password
const char* ssid = "";      // Enter SSID here
const char* password = "";  // Enter Password here

const uint16_t port = 1883U;

// Initialize the Ethernet client object
WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);

// Initialize ThingsBoard instance
ThingsBoard tb(mqttClient, 256U);

PulseOximeter pox;
uint32_t tsLastReport = 0;

TaskHandle_t TaskMAX;
TaskHandle_t TaskLM;
TaskHandle_t TaskWifi;
TaskHandle_t ThingsBoardTask;

void onBeatDetected() {
  Serial.println("Beat Detected!");
}

void setup() {
  Serial.begin(115200);
  pinMode(19, OUTPUT);
  delay(100);
  Serial.println("Hello, ESP32!");

  oximeterMutex = xSemaphoreCreateMutex();
  // Membuat antrian dengan kapasitas maksimum
  tempQueue = xQueueCreate(5, sizeof(Temp));

  if (tempQueue == NULL) {
    Serial.println("Gagal membuat antrian.");
    while (1);
  }

  // Initialize the PulseOximeter instance
  Serial.print("Initializing pulse oximeter..");
  if (!pox.begin()) {
    Serial.println("FAILED");
    for(;;);
  } else {
    Serial.println("SUCCESS");
  }
  pox.setOnBeatDetectedCallback(onBeatDetected);


  // Create task for reading data from MAX30100 sensor
  xTaskCreatePinnedToCore(
    MAX30100SensorReading,    /* Task function. */
    "MAX30100SensorReading",  /* Name of task. */
    10000,                    /* Stack size of task */
    NULL,                     /* Parameter of the task */
    1,                        /* Priority of the task */
    &TaskMAX,                 /* Task handle to keep track of created task */
    0                         /* Pin task to core 0 */
  );                  

   // Create task for reading data from LM35 sensor
  xTaskCreatePinnedToCore(
    LM35SensorReading,        /* Task function. */
    "LM35SensorReading",      /* Name of task. */
    10000,                    /* Stack size of task */
    NULL,                     /* Parameter of the task */
    1,                        /* Priority of the task */
    &TaskLM,                  /* Task handle to keep track of created task */
    1                         /* Pin task to core 1 */
  );                  

  // Create task for WiFi
  xTaskCreatePinnedToCore(
    WiFiTaskCode,             /* Task function. */
    "WiFiTask",               /* Name of task. */
    10000,                    /* Stack size of task */
    NULL,                     /* Parameter of the task */
    1,                        /* Priority of the task */
    &TaskWifi,                /* Task handle to keep track of created task */
    0                         /* Pin task to core 0 */
  );         

  // Create task for ThingsBoard
  xTaskCreatePinnedToCore(
    ThingsBoardTaskCode,      /* Task function. */
    "ThingsBoardTask",        /* Name of task. */
    10000,                    /* Stack size of task */
    NULL,                     /* Parameter of the task */
    1,                        /* Priority of the task */
    &ThingsBoardTask,         /* Task handle to keep track of created task */
    1                         /* Pin task to core 1 */
  );              
}

void loop() {
}

void WiFiTaskCode(void * parameter) {
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
      vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay for 1 second
      Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  for(;;) {
    // Check if WiFi is connected, if not, then reconnect
    while(WiFi.status() != WL_CONNECTED){
    Serial.println("Reconnecting WiFi...");
    WiFi.reconnect();
    vTaskDelay(5000 / portTICK_PERIOD_MS); // Delay for 5 seconds
    }
  vTaskDelay(3000 / portTICK_PERIOD_MS); // Delay for 3 seconds
  }
}

void ThingsBoardTaskCode(void * parameter) {
  for(;;) {
    // Connect to ThingsBoard
    while(!tb.connected()){
      Serial.println("Connecting to thingsboard...");
      tb.connect(TB_SERVER, TB_TOKEN);
      vTaskDelay(3000 / portTICK_PERIOD_MS); // Delay for 5 seconds
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay for 5 seconds

    if (xSemaphoreTake(oximeterMutex, portMAX_DELAY)){
      if (millis() - tsLastReport > REPORTING_PERIOD_MS){
        Serial.print("Heart Rate: ");
        Serial.print(BPM);
        Serial.print(" BPM");

        Serial.print(" | SpO2: ");
        Serial.print(SpO2);
        Serial.println("%");

        tsLastReport = millis();

        xSemaphoreGive(oximeterMutex);
      }
      vTaskDelay(2000 / portTICK_PERIOD_MS);
    }

    Temp receivedTemp;
    if (xQueueReceive(tempQueue, &receivedTemp, portMAX_DELAY) == pdPASS) {
      if (millis() - tsLastReport > REPORTING_PERIOD_MS){
        Serial.print("Temperature: ");
        Serial.print(receivedTemp.tempC);  // print the temperature in °C
        Serial.print("°C");
        Serial.print(" ~ "); // separator between °C and °F
        Serial.print(receivedTemp.tempF);  // print the temperature in °F
        Serial.println("°F");
        tsLastReport = millis();
      }
    }

    // Sending data to ThingsBoard
    tb.sendTelemetryData("temperature_c", receivedTemp.tempC);
    tb.sendTelemetryData("temperature_f", receivedTemp.tempF);
    tb.sendTelemetryData("heart_rate_bpm", BPM);
    tb.sendTelemetryData("heart_rate_oxy", SpO2);
    tb.loop();
  }
}


void MAX30100SensorReading(void * parameter) {
  while(1){
    if(xSemaphoreTake(oximeterMutex, portMAX_DELAY)){
      pox.update();
      BPM = pox.getHeartRate();
      SpO2 = pox.getSpO2();

      xSemaphoreGive(oximeterMutex);
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void LM35SensorReading(void * parameter) {
  for(;;) {
    int adcVal = analogRead(PIN_LM35); // Read the ADC value from sensor
    float milliVolt = adcVal * (ADC_VREF_mV / ADC_RESOLUTION); // Convert the ADC value to voltage in millivolt
    float tempC = milliVolt / 10; // Convert the voltage to the temperature in °C
    float tempF = tempC * 9 / 5 + 32; // convert the °C to °F

    Temp temp;
    temp.tempC = tempC;
    temp.tempF = tempF;

    // Mengirim data suhu ke dalam antrian
    if (xQueueSend(tempQueue, &temp, portMAX_DELAY) == pdPASS) {
      // Serial.println("Data suhu berhasil dikirim ke antrian.");
    } else {
      Serial.println("Gagal mengirim data suhu ke antrian.");
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}