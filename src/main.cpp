#include <WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <ModbusMaster.h>
#include <Preferences.h>

#include "Config.h"



// MQTT Topics

#define MQTT_PUBLISH_TOPIC_SENSOR_1 "DMA/Energy/data"
#define MQTT_PUBLISH_TOPIC_SENSOR_2 "DMA/Energy/data2"

// Time-related variables
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 21600, 3600000);  // Update time every hour

// Globals for Wi-Fi, MQTT, and Preferences
WiFiClient espClient;
PubSubClient mqttClient(espClient);
Preferences preferences;
WiFiManager wifiManager;
volatile bool apModeTriggered = false;
bool debugMode = true;
bool apModeActive = false;


#define MAX485_DE_RE 27 // DE and RE combined on GPIO 27 for control
#define RS485_RX 16    // RO pin on RS-485 module to GPIO 16 (RX2)
#define RS485_TX 17    // DI pin on RS-485 module to GPIO 17 (TX2)

ModbusMaster node;

void preTransmission() {
  digitalWrite(MAX485_DE_RE, HIGH); // Enable Transmit mode
}

void postTransmission() {
  digitalWrite(MAX485_DE_RE, LOW); // Enable Receive mode
}

// all functions
int readModbusData(uint16_t reg_address);
void display();
void read_modbus();
String getFormattedDate();
String getFormattedTime();
void write_sd();
void sdCardWriteTask(void *pvParameters);

// Timing Control
unsigned long lastReadTime1 = 0;
unsigned long lastReadTime2 = 0;

// Utility Macros for Debugging
#define DEBUG_PRINT(x) if (debugMode) Serial.print(x)
#define DEBUG_PRINTLN(x) if (debugMode) Serial.println(x)

// MQTT Callback Function
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    String message;
    for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    DEBUG_PRINT("Message on topic: "); DEBUG_PRINT(topic);
    DEBUG_PRINT(". Message: "); DEBUG_PRINTLN(message);
    Serial.println(message);

    if (String(topic) == MQTT_SUBSCRIBE_TOPIC) {
        if (message == "ON") {
            DEBUG_PRINTLN("Turning on device...");
            // Add device control logic
        } else if (message == "OFF") {
            DEBUG_PRINTLN("Turning off device...");
            // Add device control logic
        }
    }
}

// Wi-Fi Connection Function
void connectToWiFi() {
    WiFi.begin();
    int retryCount = 0;
    while (WiFi.status() != WL_CONNECTED && retryCount < WIFI_MAX_RETRIES) {
        vTaskDelay(500 / portTICK_PERIOD_MS);
        retryCount++;
        DEBUG_PRINT(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
        DEBUG_PRINTLN("\nConnected to Wi-Fi.");
    } else {
        DEBUG_PRINTLN("\nFailed to connect to Wi-Fi. Use button for AP mode.");
    }
}

void connectToMQTT() {
    int mqttRetryCount = 0;
    int backoffDelay = MQTT_INITIAL_BACKOFF;

    while (!mqttClient.connected()) {
        DEBUG_PRINT("Attempting MQTT connection...");
        if (mqttClient.connect("ESP32Client", MQTT_USER, MQTT_PASSWORD)) {
            DEBUG_PRINTLN("Connected to MQTT broker.");
            mqttRetryCount = 0;

            // Subscribe to the topic after successful connection
            if (mqttClient.subscribe(MQTT_SUBSCRIBE_TOPIC)) {
                DEBUG_PRINT("Subscribed to topic: ");
                DEBUG_PRINTLN(MQTT_SUBSCRIBE_TOPIC);
            } else {
                DEBUG_PRINTLN("Subscription failed!");
            }

            backoffDelay = MQTT_INITIAL_BACKOFF;
            break;
        } else {
            DEBUG_PRINT("Failed to connect, MQTT State Code: ");
            DEBUG_PRINTLN(mqttClient.state());
            mqttRetryCount++;
            if (mqttRetryCount >= MQTT_MAX_RETRIES) ESP.restart();
            delay(backoffDelay);
            backoffDelay = min(backoffDelay * BACKOFF_MULTIPLIER, 10000);
        }
    }
}


// Task for Wi-Fi and MQTT Connection Management
void wifiMqttTask(void *pvParameters) {
    int wifi_attempts = preferences.getInt("wifi_attempts", 0);
    int mqtt_attempts = preferences.getInt("mqtt_attempts", 0);

    while (true) {
            // In AP mode, skip Wi-Fi and MQTT handling
        if (apModeTriggered) {vTaskDelay(1000 / portTICK_PERIOD_MS);continue;}

        // Reconnect to Wi-Fi if not connected
        if (WiFi.status() != WL_CONNECTED) {
            connectToWiFi();
            if (WiFi.status() == WL_CONNECTED) {
                wifi_attempts = 0;
                preferences.putInt("wifi_attempts", wifi_attempts);
            } else if (++wifi_attempts >= WIFI_MAX_RETRIES) {
                ESP.restart();
            }
            vTaskDelay(30000 / portTICK_PERIOD_MS); // Retry delay
        }

        // Reconnect to MQTT if Wi-Fi is connected but MQTT is not
        if (WiFi.status() == WL_CONNECTED && !mqttClient.connected()) {
            connectToMQTT();
            if (mqttClient.connected()) {
                mqtt_attempts = 0;
                preferences.putInt("mqtt_attempts", mqtt_attempts);
                mqttClient.setCallback(mqttCallback);
            } else if (++mqtt_attempts >= MQTT_MAX_RETRIES) {
                ESP.restart();
            }
            vTaskDelay(10000 / portTICK_PERIOD_MS); // Retry delay
        }

        // Regularly process incoming MQTT messages
        if (mqttClient.connected()) {
            mqttClient.loop();  // Call loop to process incoming messages
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay for task execution frequency
    }
}


// Button Task for AP Mode Triggering
void buttonTask(void *pvParameters) {
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    unsigned long pressStartTime = 0;
    bool longPressDetected = false;

    while (true) {
        if (digitalRead(BUTTON_PIN) == LOW) {
            if (pressStartTime == 0) pressStartTime = millis();
            if (millis() - pressStartTime >= LONG_PRESS_DURATION && !longPressDetected) {
                longPressDetected = true;
                apModeTriggered = true;
                DEBUG_PRINTLN("Long press detected. Triggering AP mode.");
            }
        } else {
            pressStartTime = 0;
            longPressDetected = false;
        }

        if (apModeTriggered) {
            DEBUG_PRINTLN("Starting Wi-Fi AP mode...");
            wifiManager.resetSettings();
            apModeActive = true;
            wifiManager.startConfigPortal("DMA ENERGY METER");
            apModeTriggered = false;
            ESP.restart();
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

// Sensor Reading and Publishing Task
void sensorTask(void *pvParameters) {
    while (true) {
        read_modbus();
        Serial.println(".....read modbus.......");
        display();
        /*
        unsigned long currentMillis = millis();
        

        // publish modbusdata
        if (currentMillis - lastReadTime1 >= 2000) {
            lastReadTime1 = currentMillis;
            // DEBUG_PRINT("Sensor 1 Value: "); DEBUG_PRINTLN(sensorValue1);

            if (WiFi.isConnected() && mqttClient.connected()) {
                mqttClient.publish(DEFAULT_MQTT_PUB_TOPIC, modbusdata.c_str());
                Serial.println("Published modbusdata");
                Serial.println(modbusdata);
            } else {
                Serial.println("Failed to Publish modbusdata");
            }
        }

        // Read and publish Sensor 2
        
        if (currentMillis - lastReadTime2 >= INTERVAL_SENSOR_2) {
            lastReadTime2 = currentMillis;
            int sensorValue2 = analogRead(SENSOR_PIN_2);
            DEBUG_PRINT("Sensor 2 Value: "); DEBUG_PRINTLN(sensorValue2);

            if (WiFi.isConnected() && mqttClient.connected()) {
                String payload = String(sensorValue2);
                mqttClient.publish(MQTT_PUBLISH_TOPIC_SENSOR_2, payload.c_str());
            }
        }
        */

        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}


// sd card task Task
void sdCardWriteTask(void *pvParameters) {
    while (true) {
        unsigned long currentMillis = millis();
        

        // publish modbusdata
        if (currentMillis - lastReadTime1 >= 8000) {
            lastReadTime1 = currentMillis;
            // DEBUG_PRINT("Sensor 1 Value: "); DEBUG_PRINTLN(sensorValue1);
            write_sd();
            Serial.println("Data write to sd card....");
            display();

            
            if (WiFi.isConnected() && mqttClient.connected()) {
                mqttClient.publish(DEFAULT_MQTT_PUB_TOPIC, modbusdata.c_str());
                Serial.println("Published modbusdata");
                Serial.println(modbusdata);
            } else {
                Serial.println("Failed to Publish modbusdata");
            }
            
        }

        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

void read_modbus() {

    rs485_inquiry_taeHigh = readModbusData(taeHigh_reg_addr);
    rs485_inquiry_taeLow = readModbusData(taeLow_reg_addr);
    rs485_inquiry_activePower = readModbusData(activePower_reg_addr);
    rs485_inquiry_pAvolt = readModbusData(pAvolt_reg_addr);
    rs485_inquiry_pBvolt = readModbusData(pBcurrent_reg_addr);
    rs485_inquiry_pCvolt = readModbusData(pCvolt_reg_addr);
    rs485_inquiry_lABvolt = readModbusData(lABvolt_reg_addr);
    rs485_inquiry_lBCvolt = readModbusData(lBCvolt_reg_addr);
    rs485_inquiry_lCAvolt = readModbusData(lCAvolt_reg_addr);
    rs485_inquiry_pAcurrent = readModbusData(pAcurrent_reg_addr);
    rs485_inquiry_pBcurrent = readModbusData(pBcurrent_reg_addr);
    rs485_inquiry_pCcurrent = readModbusData(pCcurrent_reg_addr);
    rs485_inquiry_frequency = readModbusData(frequency_reg_addr);
    rs485_inquiry_powerfactor = readModbusData(powerfactor_reg_addr);

    actual_taeHigh = rs485_inquiry_taeHigh/10.0;
    actual_taeLow = rs485_inquiry_taeLow/10.0;
    actual_activePower = rs485_inquiry_activePower/10.0;  
    actual_pAvolt = rs485_inquiry_pAvolt/10.0;
    actual_pBvolt = rs485_inquiry_pBvolt/10.0;  
    actual_pCvolt = rs485_inquiry_pCvolt/10.0;
    actual_lABvolt = rs485_inquiry_lABvolt/10.0; 
    actual_lBCvolt = rs485_inquiry_lBCvolt/10.0;
    actual_lCAvolt = rs485_inquiry_lCAvolt/10.0;
    actual_pAcurrent = rs485_inquiry_pAcurrent/10.0;
    actual_pBcurrent = rs485_inquiry_pBcurrent/10.0;
    actual_pCcurrent = rs485_inquiry_pCcurrent/10.0;
    actual_frequency = rs485_inquiry_frequency/100.0;
    actual_powerfactor = rs485_inquiry_powerfactor/1000.0;

      // Append values to dataString
    modbusdata += String(DEVICE_ID) + ",";
    modbusdata += String(actual_taeHigh, 2) + ",";
    modbusdata += String(actual_taeLow, 2) + ",";
    modbusdata += String(actual_activePower, 2) + ",";
    modbusdata += String(actual_pAvolt, 2) + ",";
    modbusdata += String(actual_pBvolt, 2) + ",";
    modbusdata += String(actual_pCvolt, 2) + ",";
    modbusdata += String(actual_lABvolt, 2) + ",";
    modbusdata += String(actual_lBCvolt, 2) + ",";
    modbusdata += String(actual_lCAvolt, 2) + ",";
    modbusdata += String(actual_pAcurrent, 2) + ",";
    modbusdata += String(actual_pBcurrent, 2) + ",";
    modbusdata += String(actual_pCcurrent, 2) + ",";
    modbusdata += String(actual_frequency, 2) + ",";
    modbusdata += String(actual_powerfactor, 2);

    vTaskDelay(1000 / portTICK_PERIOD_MS);

}
// Function to read a specific Modbus register and print the result
int readModbusData(uint16_t reg_address) {
  uint8_t result;
  uint16_t data;
  result = node.readHoldingRegisters(reg_address, 1); // Read 1 register from 'reg'
  if (result == node.ku8MBSuccess) {
    data = node.getResponseBuffer(0); // Get the data from the first register
    return data;
  } else {
    return result;
  }
}

void display() {
    Serial.print("TotalactiveEnergyHighByte: "); Serial.print(rs485_inquiry_taeHigh);     Serial.println(" kWh"); 
    Serial.print("TotalactiveEnergyLowByte:  "); Serial.print(rs485_inquiry_taeLow);      Serial.println(" kWh"); 
    Serial.print("Active power:              "); Serial.print(rs485_inquiry_activePower); Serial.println(" kW"); 
    Serial.print("Phase A voltage:           "); Serial.print(rs485_inquiry_pAvolt);      Serial.println(" volt"); 
    Serial.print("Phase B voltage:           "); Serial.print(rs485_inquiry_pBvolt);      Serial.println(" volt"); 
    Serial.print("Phase C voltage:           "); Serial.print(rs485_inquiry_pCvolt);      Serial.println(" volt"); 
    Serial.print("Line A-B voltage:          "); Serial.print(rs485_inquiry_lABvolt);     Serial.println(" volt"); 
    Serial.print("Line B-C voltage:          "); Serial.print(rs485_inquiry_lBCvolt);     Serial.println(" volt"); 
    Serial.print("Line C-A Voltage:          "); Serial.print(rs485_inquiry_lCAvolt);     Serial.println(" volt"); 
    Serial.print("phase A current:           "); Serial.print(rs485_inquiry_pAcurrent);   Serial.println(" A"); 
    Serial.print("phase B current:           "); Serial.print(rs485_inquiry_pBcurrent);   Serial.println(" A"); 
    Serial.print("phase C current:           "); Serial.print(rs485_inquiry_pCcurrent);   Serial.println(" A"); 
    Serial.print("Frequency:                 "); Serial.print(rs485_inquiry_frequency);   Serial.println(" Hz"); 
    Serial.print("PowerFactor:               "); Serial.print(rs485_inquiry_powerfactor); Serial.println("");
    Serial.println("");
    Serial.println(modbusdata);     Serial.println("");Serial.println("");
}


// Function to get the current date in "YYYY-MM-DD" format
String getFormattedDate() {
  if (!timeClient.isTimeSet()) return "1970-01-01";  // Default date if time not set
  time_t rawTime = timeClient.getEpochTime();
  struct tm * timeInfo = localtime(&rawTime);
  char buffer[11];
  strftime(buffer, sizeof(buffer), "%Y-%m-%d", timeInfo);
  return String(buffer);
}

// Function to get the current time in "HH:MM:SS" format
String getFormattedTime() {
  if (!timeClient.isTimeSet()) return "00:00:00";  // Default time if time not set
  time_t rawTime = timeClient.getEpochTime();
  struct tm * timeInfo = localtime(&rawTime);
  char buffer[9];
  strftime(buffer, sizeof(buffer), "%H:%M:%S", timeInfo);
  return String(buffer);
}

void write_sd() {

    // Retrieve current date and time
    String date = getFormattedDate();
    String time = getFormattedTime();

    // Open file in append mode
    File file = SD.open("/em_data.csv", FILE_APPEND);
    if (file) {
    // Write data row to file
    file.print(date); file.print(",");
    file.print(time); file.print(",");
    file.print(status); file.print(",");
    
    // Write each variable separated by commas
    file.print(actual_taeHigh); file.print(",");
    file.print(actual_taeLow); file.print(",");
    file.print(actual_activePower); file.print(",");
    file.print(actual_pAvolt); file.print(",");
    file.print(actual_pBvolt); file.print(",");
    file.print(actual_pCvolt); file.print(",");
    file.print(actual_lABvolt); file.print(",");
    file.print(actual_lBCvolt); file.print(",");
    file.print(actual_lCAvolt); file.print(",");
    file.print(actual_pAcurrent); file.print(",");
    file.print(actual_pBcurrent); file.print(",");
    file.print(actual_pCcurrent); file.print(",");
    file.print(actual_frequency); file.print(",");
    file.println(actual_powerfactor);  // End of line

    file.close();  // Close the file
    Serial.println("Data written to SD card.");
    } else {
        Serial.println("Error opening em_data.csv.");
    }
}




/****************************************************************************/
/*                               --  main  --                               */
/****************************************************************************/

void setup() {
    Serial.begin(115200);
    Serial2.begin(9600, SERIAL_8N1, RS485_RX, RS485_TX); // UART2 for RS-485

    pinMode(MAX485_DE_RE, OUTPUT);
    digitalWrite(MAX485_DE_RE, LOW);      // Set to Receive mode initially

    node.begin(1, Serial2);               // Modbus ID 1 and Serial2 for RS-485
    node.preTransmission(preTransmission);
    node.postTransmission(postTransmission);

    Serial.println("RS-485 Modbus communication initialized.");

    WiFi.mode(WIFI_STA);
    mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
    mqttClient.setCallback(mqttCallback);
    preferences.begin("connection", false);


    // Initialize time client and attempt to get the time
    timeClient.begin();
    if (timeClient.forceUpdate()) {
        Serial.println("Time synchronized with NTP server.");
    } else {
        Serial.println("Failed to sync time with NTP server. Using internal RTC if available.");
    }

    // Initialize SD card
    if (!SD.begin(chipSelect)) {
        Serial.println("SD card initialization failed!");
        return;
    }
    Serial.println("SD card initialized.");

    // Open or create file and add header if it doesn't exist
    File file = SD.open("/em_data.csv", FILE_APPEND);
    if (file.size() == 0) {
        file.println("Date,Time,Status,taeHigh,taeLow,activePower,pAvolt,pBvolt,pCvolt,lABvolt,lBCvolt,lCAvolt,pAcurrent,pBcurrent,pCcurrent,frequency,powerfactor"); // Header row
    }
    file.close();


    /*
    xTaskCreatePinnedToCore(wifiMqttTask, "wifiMqttTask", 4096, NULL, 2, NULL, 1); // Higher priority for MQTT
    xTaskCreatePinnedToCore(buttonTask, "buttonTask", 2048, NULL, 3, NULL, 1);     // Highest priority for real-time response
    xTaskCreatePinnedToCore(sensorTask, "sensorTask", 6144, NULL, 1, NULL, 0);     // Lowest priority for sensor task
    */

    xTaskCreatePinnedToCore(wifiMqttTask, "wifiMqttTask", 4096, NULL, 3, NULL, 1);    // Core 1: Medium priority, WiFi & MQTT connection, sufficient stack
    xTaskCreatePinnedToCore(buttonTask, "buttonTask", 2048, NULL, 1, NULL, 1);            // Core 1: Low priority, smaller stack for button handling
    xTaskCreatePinnedToCore(sensorTask, "sensorTask", 4096, NULL, 4, NULL, 0);            // Core 0: High priority, sensor / Modbus communication
    xTaskCreatePinnedToCore(sdCardWriteTask, "sdCardWriteTask", 4096, NULL, 2, NULL, 0);  // Core 0: Medium priority, SD card writing with adequate stack   
}

void loop() {}


/*
xTaskCreatePinnedToCore(wifiMqttTask, "wifiMqttTask", 4096, NULL, 3, NULL, 1);
xTaskCreatePinnedToCore(buttonTask, "buttonTask", 2048, NULL, 4, NULL, 1);
xTaskCreatePinnedToCore(modbusReadTask, "modbusReadTask", 4096, NULL, 2, NULL, 0);
xTaskCreatePinnedToCore(sdCardWriteTask, "sdCardWriteTask", 6144, NULL, 1, NULL, 0);
xTaskCreatePinnedToCore(dataProcessLedTask, "dataProcessLedTask", 3072, NULL, 2, NULL, 0);

xTaskCreatePinnedToCore(wifiMqttTask, "wifiMqttTask", 4096, NULL, 3, NULL, 1);    // Core 1: Medium priority, WiFi & MQTT connection, sufficient stack
xTaskCreatePinnedToCore(buttonTask, "buttonTask", 2048, NULL, 1, NULL, 1);       // Core 1: Low priority, smaller stack for button handling
xTaskCreatePinnedToCore(modbusReadTask, "modbusReadTask", 4096, NULL, 4, NULL, 0); // Core 0: High priority, Modbus communication
xTaskCreatePinnedToCore(sdCardWriteTask, "sdCardWriteTask", 4096, NULL, 2, NULL, 0); // Core 0: Medium priority, SD card writing with adequate stack

*/