#include <WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <Preferences.h>

// MQTT and Wi-Fi Configuration
#define MQTT_SERVER "broker2.dma-bd.com"
#define MQTT_PORT 1883
#define MQTT_USER "broker2"
#define MQTT_PASSWORD "Secret!@#$1234"
#define WIFI_MAX_RETRIES 30
#define MQTT_MAX_RETRIES 10
#define MQTT_INITIAL_BACKOFF 1000 // ms
#define BACKOFF_MULTIPLIER 2

// GPIO Pins and Constants
#define BUTTON_PIN 23
#define SENSOR_PIN_1 39
#define SENSOR_PIN_2 36
#define LONG_PRESS_DURATION 3000 // 3 seconds

// Sensor Intervals (ms)
const unsigned long INTERVAL_SENSOR_1 = 5000;
const unsigned long INTERVAL_SENSOR_2 = 10000;

// MQTT Topics
#define MQTT_SUBSCRIBE_TOPIC "DMA/Energy/status"
#define MQTT_PUBLISH_TOPIC_SENSOR_1 "DMA/Energy/data"
#define MQTT_PUBLISH_TOPIC_SENSOR_2 "DMA/Energy/data2"

// Globals for Wi-Fi, MQTT, and Preferences
WiFiClient espClient;
PubSubClient mqttClient(espClient);
Preferences preferences;
WiFiManager wifiManager;
volatile bool apModeTriggered = false;
bool debugMode = true;

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
        unsigned long currentMillis = millis();

        // Read and publish Sensor 1
        if (currentMillis - lastReadTime1 >= INTERVAL_SENSOR_1) {
            lastReadTime1 = currentMillis;
            int sensorValue1 = analogRead(SENSOR_PIN_1);
            DEBUG_PRINT("Sensor 1 Value: "); DEBUG_PRINTLN(sensorValue1);

            if (WiFi.isConnected() && mqttClient.connected()) {
                String payload = String(sensorValue1);
                mqttClient.publish(MQTT_PUBLISH_TOPIC_SENSOR_1, payload.c_str());
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

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
    mqttClient.setCallback(mqttCallback);
    preferences.begin("connection", false);

    xTaskCreatePinnedToCore(wifiMqttTask, "wifiMqttTask", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(buttonTask, "buttonTask", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(sensorTask, "sensorTask", 2048, NULL, 1, NULL, 1);
}

void loop() {}
