/*
IoT Final Project
Smart Health Monitoring System for Real-time Heart Rate and Body Temperature Tracking

Group B3
Lauren Christy Tanudjaja				2106707870
Mikhael Morris Hapataran Siallagan		2106731491
Rafi Fauzan Wirasena					2106656320
Zalfy Putra Rezky					    2106731453
*/

#include <Wire.h>
#include <WiFi.h>
#include <ThingsBoard.h>
#include "MAX30100_PulseOximeter.h"

#define REPORTING_PERIOD_MS 1000 // Increase reporting period to avoid sending data too frequently
#define ADC_VREF_mV    3300.0 // in millivolt
#define ADC_RESOLUTION 4096.0 // 12-bit ADC
#define PIN_LM35       36 // ESP32 pin GPIO36 (ADC0) connected to LM35

float BPM, SpO2;

// Put your SSID & Password
const char* ssid = "REZKY_OXY"; // Enter SSID here
const char* password = "arjuna77"; // Enter Password here

// ThingsBoard configuration
const char* thingsboardServer = "https://thingsboard.cloud/"; // Enter ThingsBoard server instance
const char* accessToken = "dmei57uu26f21j1wmtv3"; // Enter your Device Access Token

// Initialize the Ethernet client object
WiFiClient espClient;
// Initialize ThingsBoard instance
ThingsBoard tb(espClient);

PulseOximeter pox;
uint32_t tsLastReport = 0;

void onBeatDetected() {
  Serial.println("Beat Detected!");
}

void setup() {
    Serial.begin(115200);
    pinMode(19, OUTPUT);
    delay(100);
    Serial.println("Hello, ESP32!");

    // Connect to Wi-Fi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");

    // Initialize the PulseOximeter instance
    Serial.print("Initializing pulse oximeter..");
    if (!pox.begin()) {
        Serial.println("FAILED");
        for(;;);
    } else {
        Serial.println("SUCCESS");
    }
    pox.setOnBeatDetectedCallback(onBeatDetected);

    // Connect to ThingsBoard
    tb.begin(thingsboardServer, accessToken);

    // Initialize the PulseOximeter instance
    pox.begin();
    pox.setOnBeatDetectedCallback(onBeatDetected);
    pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
}

void loop() {
    /* MAX30100 Sensor Reading */
    pox.update();
    BPM = pox.getHeartRate();
    SpO2 = pox.getSpO2();

    /* LM35 sensor reading */
    int adcVal = analogRead(PIN_LM35); // Read the ADC value from sensor
    float milliVolt = adcVal * (ADC_VREF_mV / ADC_RESOLUTION); // Convert the ADC value to voltage in millivolt
    float tempC = milliVolt / 10; // Convert the voltage to the temperature in °C
    float tempF = tempC * 9 / 5 + 32; // convert the °C to °F

    // Print data to the serial monitor
    if (millis() - tsLastReport > REPORTING_PERIOD_MS)
    {
        Serial.print("Temperature: ");
        Serial.print(tempC);   // print the temperature in °C
        Serial.print("°C");
        Serial.print("  ~  "); // separator between °C and °F
        Serial.print(tempF);   // print the temperature in °F
        Serial.println("°F");

        Serial.print(" | Heart Rate: ");
        Serial.print(BPM);
        Serial.print(" BPM");

        Serial.print(" | SpO2: ");
        Serial.print(SpO2);
        Serial.println("%");

        tsLastReport = millis();
    }

    // Sending data to ThingsBoard
    tb.sendTelemetryData("temperature", tempC);
    tb.sendTelemetryData("heart_rate", BPM);
    delay(500);
}