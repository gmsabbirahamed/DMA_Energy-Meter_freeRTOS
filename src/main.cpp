#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <ModbusMaster.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "Config.h"

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

WiFiClient espClient;   // MQTT client
PubSubClient client(espClient);

WiFiManager wifiManager;    // WiFiManager Object

// Define constants for stack depth and task priority
constexpr uint16_t TASK_STACK_DEPTH = 1000;
constexpr UBaseType_t TASK_PRIORITY = 1;
constexpr BaseType_t CORE_ID = 0; // Core to pin tasks to

// Define stack arrays and task control block buffers for each task
StaticTask_t modbusTaskBuffer;
StaticTask_t wifi_mqtt_TaskBuffer;
StaticTask_t outputTaskBuffer;
StaticTask_t sdTaskBuffer;
StaticTask_t buttonTaskBuffer;

StackType_t modbusStack[TASK_STACK_DEPTH];
StackType_t wifi_mqtt_Stack[TASK_STACK_DEPTH];
StackType_t outputStack[TASK_STACK_DEPTH];
StackType_t sdStack[TASK_STACK_DEPTH];
StackType_t buttonStack[TASK_STACK_DEPTH];

// modbus data variable
int rs485_inquiry_taeHigh;
  int rs485_inquiry_taeLow;
  int rs485_inquiry_activePower;
  int rs485_inquiry_pAvolt;
  int rs485_inquiry_pBvolt;
  int rs485_inquiry_pCvolt;
  int rs485_inquiry_pABvolt;
  int rs485_inquiry_lBCvolt;
  int rs485_inquiry_lCAvolt;
  int rs485_inquiry_pAcurrent;
  int rs485_inquiry_pBcurrent;
  int rs485_inquiry_pCcurrent;
  int rs485_inquiry_frequency;
  int rs485_inquiry_powerfactor;

  float actual_taeHigh;
  float actual_teaLow;
  float actual_activePower;
  float actual_pAvolt;
  float actual_pBvolt;
  float actual_pCvolt;
  float actual_pABvolt;
  float actual_lBCvolt;
  float actual_lCAvolt;
  float actual_pAcurrent;
  float actual_pBcurrent;
  float actual_pCcurrent;
  float actual_frequency;
  float actual_powerfactor;



// Forward declarations of task functions
void modbusReadTask(void *parameter);
void wifi_mqtt_Task(void *parameter);
void outputControlTask(void *parameter);
void sdTask(void *parameter);
void wificonfigure_Task(void *parameter);

void callback(char* topic, byte* payload, unsigned int length);
int readModbusData(uint16_t reg_address);
void connectToMQTT();



/******************************************************************************/
/*----------------------------     setup     ---------------------------------*/
/******************************************************************************/

void setup() {
    Serial.begin(115200);
    Serial2.begin(9600, SERIAL_8N1, RS485_RX, RS485_TX); // UART2 for RS-485

    pinMode(MAX485_DE_RE, OUTPUT);
    digitalWrite(MAX485_DE_RE, LOW);      // Set to Receive mode initially

    node.begin(1, Serial2);               // Modbus ID 1 and Serial2 for RS-485
    node.preTransmission(preTransmission);
    node.postTransmission(postTransmission);

    Serial.println("RS-485 Modbus communication initialized.");

    // Set MQTT server and callback
    client.setServer(mqtt_broker, mqtt_port);
    client.setCallback(callback);

    // Create the Modbus read task pinned to core 0
    xTaskCreateStaticPinnedToCore(
        modbusReadTask,               // Task function
        "Modbus_Read_Task",           // Task name
        TASK_STACK_DEPTH,             // Stack depth
        NULL,                         // Parameter (no data passed)
        TASK_PRIORITY,                // Priority level
        modbusStack,                  // Stack array
        &modbusTaskBuffer,            // Task buffer
        CORE_ID                       // Core to run the task on
    );
    
    xTaskCreateStaticPinnedToCore(
        wifi_mqtt_Task,               // Task function
        "Modbus_Read_Task",           // Task name
        TASK_STACK_DEPTH,             // Stack depth
        NULL,                         // Parameter (no data passed)
        TASK_PRIORITY,                // Priority level
        modbusStack,                  // Stack array
        &modbusTaskBuffer,            // Task buffer
        CORE_ID                       // Core to run the task on
    );

    // Create the output control task pinned to core 0
    xTaskCreateStaticPinnedToCore(
        sdTask,            // Task function
        "Output_Control_Task",        // Task name
        TASK_STACK_DEPTH,             // Stack depth
        NULL,                         // Parameter (no data passed)
        TASK_PRIORITY,                // Priority level
        outputStack,                  // Stack array
        &outputTaskBuffer,            // Task buffer
        CORE_ID                       // Core to run the task on
    );
    // Start button detection task
    xTaskCreateStaticPinnedToCore(
        buttonPressTask,          // Task function
        "ButtonPressTask",        // Task name
        TASK_STACK_DEPTH,         // Stack depth
        NULL,                     // Parameter (no data passed)
        TASK_PRIORITY,            // Priority level
        buttonStack,              // Stack array
        &buttonTaskBuffer,        // Task buffer
        CORE_ID                   // Core to run the task on
    );
}

void loop() {
    // FreeRTOS handles task scheduling; loop() can be empty
}





// Modbus Read Task: Simulates reading data from a Modbus device
void modbusReadTask(void *parameter) {
    const TickType_t xDelay = pdMS_TO_TICKS(1000); // Delay time of 1 second

    while (true) {
        Serial.println("[Modbus Task] Reading data from Modbus...");

        rs485_inquiry_taeHigh = readModbusData(0x30);
        rs485_inquiry_taeLow = readModbusData(0x31);
        rs485_inquiry_activePower = readModbusData(0x1A);
        rs485_inquiry_pAvolt = readModbusData(0x14);
        rs485_inquiry_pBvolt = readModbusData(0x15);
        rs485_inquiry_pCvolt = readModbusData(0x16);
        rs485_inquiry_pABvolt = readModbusData(0x17);
        rs485_inquiry_lBCvolt = readModbusData(0x18);
        rs485_inquiry_lCAvolt = readModbusData(0x19);
        rs485_inquiry_pAcurrent = readModbusData(0x10);
        rs485_inquiry_pBcurrent = readModbusData(0x11);
        rs485_inquiry_pCcurrent = readModbusData(0x12);
        rs485_inquiry_frequency = readModbusData(0x1E);
        rs485_inquiry_powerfactor = readModbusData(0x1D);

        actual_taeHigh = rs485_inquiry_taeHigh/10.0;
        actual_teaLow = rs485_inquiry_taeLow/10.0;
        actual_activePower = rs485_inquiry_activePower/10.0;  
        actual_pAvolt = rs485_inquiry_pAvolt/10.0;
        actual_pBvolt = rs485_inquiry_pBvolt/10.0;  
        actual_pCvolt = rs485_inquiry_pCvolt/10.0;
        actual_pABvolt = rs485_inquiry_pABvolt/10.0; 
        actual_lBCvolt = rs485_inquiry_lBCvolt/10.0;
        actual_lCAvolt = rs485_inquiry_lCAvolt/10.0;
        actual_pAcurrent = rs485_inquiry_pAcurrent/10.0;
        actual_pBcurrent = rs485_inquiry_pBcurrent/10.0;
        actual_pCcurrent = rs485_inquiry_pCcurrent/10.0;
        actual_frequency = rs485_inquiry_frequency/100.0;
        actual_powerfactor = rs485_inquiry_powerfactor/1000.0;

        vTaskDelay(xDelay); // Wait 1 second before repeating
    }
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

// Output Control Task: Simulates control of an output device
void outputControlTask(void *parameter) {
    const TickType_t xDelay = pdMS_TO_TICKS(2000); // Delay time of 2 seconds

    while (true) {
        Serial.println("[Output Task] Controlling output device...");
        vTaskDelay(xDelay); // Wait 2 seconds before repeating
    }
}


void wifi_mqtt_Task(void *parameter) {
    WiFiManager wifiManager;
    const TickType_t wifiAttemptDelay = pdMS_TO_TICKS(30000); // 30 seconds Wi-Fi attempt delay
    const TickType_t restDelay = pdMS_TO_TICKS(90000);        // 1.5 minutes rest period
    const TickType_t taskDelay = pdMS_TO_TICKS(1000);         // 1-second delay for normal operations
    const TickType_t mqttRetryDelay = pdMS_TO_TICKS(5000);    // 5-second MQTT retry delay
    const TickType_t shortDelay = pdMS_TO_TICKS(5000);    // 5-second MQTT retry delay

    int wifiAttempts = 0;
    int maxWifiAttempts = 5;

    while (true) {
        // Check WiFi connection
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("Wi-Fi not connected. Attempting to reconnect...");
            WiFi.begin();
            wifiAttempts++;

            for (int attempt = 0; attempt < maxWifiAttempts && WiFi.status() != WL_CONNECTED; attempt++) {
            RedLed(); vTaskDelay(shortDelay); Serial.print("."); BlackLed(); vTaskDelay(shortDelay);
            }
            if (WiFi.status() == WL_CONNECTED) {
            BlackLed();
            Serial.println("\nConnected to Wi-Fi");
            Serial.print("IP Address: ");
            Serial.println(WiFi.localIP());
            return;
            } else {
            Serial.println("\nFailed to connect. Resting for 30 seconds...");
            vTaskDelay(restDelay);
            }
            // Check if max attempts reached
            if (wifiAttempts >= maxWifiAttempts) {
                Serial.println("Max Wi-Fi attempts reached. Restarting ESP...");
                ESP.restart(); // Restart ESP after max attempts
            } else {
                vTaskDelay(wifiAttemptDelay); // Wait before trying again
            }
        } else {
            wifiAttempts = 0; // Reset WiFi attempts counter when connected

            // Attempt MQTT connection if not connected
            if (!client.connected()) {
                connectToMQTT();
                vTaskDelay(mqttRetryDelay); // Delay for MQTT retry if needed
            }

            // Publish data if connected to MQTT
            if (client.connected()) {
                Serial.println("[WiFi_MQTT_Task] Reading data from Modbus and publishing...");
                client.loop();
            }

            vTaskDelay(taskDelay); // Regular delay between operations
        }
    }
}

void connectToMQTT() {
    client.setServer(mqtt_broker, mqtt_port);

    Serial.print("Connecting to MQTT...");
    if (client.connect("ESP32Client", mqtt_user, mqtt_pass)) {
        Serial.println("Connected to MQTT broker.");
    } else {
        Serial.print("Failed, rc=");
        Serial.print(client.state());
        Serial.println(". Will retry in 5 seconds.");
    }
}

void buttonPressTask(void *parameter) {
    TickType_t buttonPressStart = 0;  // Start time of the button press

    while (true) {
        // Check if button is pressed (active low)
        if (digitalRead(BUTTON_PIN) == LOW) {
            if (buttonPressStart == 0) {
                buttonPressStart = xTaskGetTickCount();  // Start timing
            }

            // Check if button has been held for the required time
            if ((xTaskGetTickCount() - buttonPressStart) >= BUTTON_HOLD_TIME) {
                Serial.println("Button held for 3 seconds. Enabling Wi-Fi configuration mode...");
                enableWiFiConfigurationMode();
                buttonPressStart = 0;  // Reset timing
            }
        } else {
            // Reset if button is released before reaching the hold time
            buttonPressStart = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(100));  // Check button state every 100 ms
    }
}
void enableWiFiConfigurationMode() {
    // Open WiFiManager configuration portal
    wifiManager.startConfigPortal("ESP32_Config_Portal");

    // Once Wi-Fi is connected, WiFiManager will store credentials
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("Wi-Fi connected successfully through configuration portal.");
    } else {
        Serial.println("Failed to connect through configuration portal.");
    }
}

// Output Control Task: Simulates control of an output device
void sdTask(void *parameter) {
    const TickType_t xDelay = pdMS_TO_TICKS(2000); // Delay time of 2 seconds

    while (true) {
        Serial.println("[sd Task] Controlling output device...");
        vTaskDelay(xDelay); // Wait 2 seconds before repeating
    }
}

/**************************************************************/

// MQTT callback for received messages
void callback(char* topic, byte* payload, unsigned int length) {
    String message;
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    Serial.print(message);
    Serial.println();
}