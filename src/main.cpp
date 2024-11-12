#include <WiFi.h>
#include <WiFiManager.h>          // WiFiManager library
#include <PubSubClient.h>
#include <Preferences.h>

#define MQTT_SERVER "broker2.dma-bd.com"      // Replace with your secure broker's domain
#define MQTT_PORT 1883                // Secure MQTT port (usually 8883)
#define MQTT_USER "broker2"     // Replace with your MQTT username
#define MQTT_PASSWORD "Secret!@#$1234"      // Replace with your MQTT password
#define MAX_RETRIES 5
#define BUTTON_PIN 23               // Change as per your setup
#define SENSOR_PIN_1 39               // Change as per your setup
#define SENSOR_PIN_2 36               // Change as per your setup
#define LONG_PRESS_DURATION 3000   // 3 seconds

// Task intervals and delays
#define WIFI_RETRY_DELAY 30000     // Wi-Fi retry delay (30 seconds)
#define MQTT_RETRY_DELAY 10000     // MQTT retry delay (10 seconds)
#define WIFI_RETRY_DELAY_T 5000     // Wi-Fi retry delay (30 seconds)
#define MQTT_RETRY_DELAY_T 30000     // MQTT retry delay (10 seconds)
#define BACKOFF_MULTIPLIER 2       // Backoff multiplier for delay


#define MQTT_SUBSCRIBE_TOPIC "DMA/Energy/status"

// Define intervals for each sensor in milliseconds
const unsigned long interval1 = 5000; // 5 seconds for sensor 1
const unsigned long interval2 = 10000; // 10 seconds for sensor 2

// Variables to store the last time each sensor was read
unsigned long lastReadTime1 = 0;
unsigned long lastReadTime2 = 0;

// Wi-Fi and MQTT objects
WiFiClient espClient;
PubSubClient mqttClient(espClient);
Preferences preferences;
WiFiManager wifiManager;

bool debugMode = true;             // Set false to disable logging
volatile bool apModeTriggered = false; // Flag to indicate AP mode needed


// Callback function to handle incoming messages on subscribed topics
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message arrived on topic: ");
    Serial.print(topic);
    Serial.print(". Message: ");
    String message;
    for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    Serial.println(message);

    // Add logic to process specific messages if needed
    if (String(topic) == MQTT_SUBSCRIBE_TOPIC) {
        if (message == "ON") {
            Serial.println("Turning on device...");
            // Add logic to control hardware if needed
        } else if (message == "OFF") {
            Serial.println("Turning off device...");
            // Add additional actions based on the message
        }
    }
}

#define WIFI_MAX_RETRIES 30
#define MQTT_MAX_RETRIES 10
#define MQTT_INITIAL_BACKOFF 1000  // in ms
#define BACKOFF_MULTIPLIER_T 2

// Utility macro for debugging messages
#define DEBUG_PRINT(x) if (debugMode) Serial.print(x)
#define DEBUG_PRINTLN(x) if (debugMode) Serial.println(x)

// WiFi connection function
void connectToWiFi() {
    WiFi.begin();  // Attempt connection with saved credentials
    int retryCount = 0;

    while (WiFi.status() != WL_CONNECTED && retryCount < WIFI_MAX_RETRIES) {
        vTaskDelay(500 / portTICK_PERIOD_MS);
        retryCount++;
        DEBUG_PRINT(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
        DEBUG_PRINTLN("\nConnected to Wi-Fi.");
    } else {
        DEBUG_PRINTLN("\nFailed to connect to Wi-Fi. Please press the button for AP mode.");
        // Optional: Flag to enable AP mode via button press.
    }
}

// MQTT connection function with exponential backoff
void connectToMQTT() {
    if (!mqttClient.connected()) {
        int mqttRetryCount = 0;
        int backoffDelay = MQTT_INITIAL_BACKOFF;

        while (!mqttClient.connected() && mqttRetryCount < MQTT_MAX_RETRIES) {
            DEBUG_PRINT("Attempting MQTT connection...");
            
            if (mqttClient.connect("ESP32Client", MQTT_USER, MQTT_PASSWORD)) {
                DEBUG_PRINTLN("Connected to MQTT broker.");
                mqttRetryCount = 0;  // Reset after successful connection
                if (mqttClient.subscribe(MQTT_SUBSCRIBE_TOPIC)) {
                    DEBUG_PRINTLN("Subscribed to topic.");
                } else {
                    DEBUG_PRINTLN("Failed to subscribe to topic.");
                }
            } else {
                mqttRetryCount++;
                int errorCode = mqttClient.state();
                DEBUG_PRINT("Failed to connect, MQTT State Code: ");
                DEBUG_PRINTLN(errorCode);

                // Enhanced error handling
                switch (errorCode) {
                    case MQTT_CONNECTION_TIMEOUT:
                        DEBUG_PRINTLN("Connection timed out.");
                        break;
                    case MQTT_CONNECTION_LOST:
                        DEBUG_PRINTLN("Connection lost.");
                        break;
                    case MQTT_CONNECT_FAILED:
                        DEBUG_PRINTLN("Connect failed.");
                        break;
                    case MQTT_DISCONNECTED:
                        DEBUG_PRINTLN("Disconnected from broker.");
                        break;
                    default:
                        DEBUG_PRINTLN("Unknown error.");
                        break;
                }

                if (mqttRetryCount >= MQTT_MAX_RETRIES) {
                    DEBUG_PRINTLN("Max MQTT connection retries reached. Restarting ESP...");
                    ESP.restart();
                } else {
                    DEBUG_PRINT("Retrying MQTT connection in ");
                    DEBUG_PRINT(backoffDelay);
                    DEBUG_PRINTLN(" milliseconds...");
                    delay(backoffDelay);
                    backoffDelay *= BACKOFF_MULTIPLIER_T;  // Increment backoff delay
                    if (backoffDelay > 10000) { // Cap the backoff delay
                        backoffDelay = 10000;
                    }
                }
            }
        }
    }
}




void wifi_mqtt_task(void *pvParameters) {
    int wifi_attempts = preferences.getInt("wifi_attempts", 0);
    int mqtt_attempts = preferences.getInt("mqtt_attempts", 0);
    bool wifi_connected = false;
    bool mqtt_connected = false;
    int backoffDelay = 15000;  // Base delay for backoff

    while (true) {
        if (WiFi.status() != WL_CONNECTED) {
            connectToWiFi();
            wifi_connected = (WiFi.status() == WL_CONNECTED);

            if (wifi_connected) {
                wifi_attempts = 0;
                preferences.putInt("wifi_attempts", wifi_attempts); // Reset on success
            } else {
                wifi_attempts++;
                if (debugMode) Serial.printf("Wi-Fi attempt %d failed.\n", wifi_attempts);
                if (wifi_attempts >= MAX_RETRIES) ESP.restart();

                backoffDelay *= BACKOFF_MULTIPLIER_T;
                vTaskDelay(backoffDelay / portTICK_PERIOD_MS);
                continue;
            }
        }

        if (wifi_connected && !mqttClient.connected()) {
            connectToMQTT();
            mqtt_connected = mqttClient.connected();

            if (mqtt_connected) {
                mqtt_attempts = 0;
                preferences.putInt("mqtt_attempts", mqtt_attempts); // Reset on success
            } else {
                mqtt_attempts++;
                if (debugMode) Serial.printf("MQTT attempt %d failed.\n", mqtt_attempts);
                if (mqtt_attempts >= MAX_RETRIES) ESP.restart();

                backoffDelay *= BACKOFF_MULTIPLIER;
                vTaskDelay(backoffDelay / portTICK_PERIOD_MS);
                continue;
            }
        }

        if (mqtt_connected) mqttClient.loop();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void button_task(void *pvParameters) {
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    unsigned long pressStartTime = 0;
    bool longPressDetected = false;

    while (true) {
        if (digitalRead(BUTTON_PIN) == LOW) {  // Button is pressed
            if (pressStartTime == 0) pressStartTime = millis();

            if (millis() - pressStartTime >= LONG_PRESS_DURATION && !longPressDetected) {
                longPressDetected = true;
                apModeTriggered = true;  // Trigger AP mode
                if (debugMode) Serial.println("Long press detected. Triggering AP mode for Wi-Fi configuration.");
            }
        } else {  // Button is released
            pressStartTime = 0;
            longPressDetected = false;
        }

        // If AP mode is triggered, break out of the loop and start AP
        if (apModeTriggered) {
            if (debugMode) Serial.println("Starting Wi-Fi AP mode...");
            wifiManager.resetSettings();  // Reset saved credentials
            wifiManager.startConfigPortal("DMA ENERGY METER");
            // wifiManager.autoConnect("ESP32_AP");  // Start AP with default SSID
            apModeTriggered = false;  // Reset flag after configuration
            ESP.restart();  // Restart to apply new settings
        }

        vTaskDelay(100 / portTICK_PERIOD_MS); // Check button status every 100 ms
    }
}


/**************************************     ..........      ********************************************/
/*                                          : sensor :                                                 */
/**************************************     :........:       *******************************************/

// Sensor reading and publishing task
void sensor_task(void *pvParameters) {
    while (true) {
        unsigned long currentMillis = millis(); // Get current time

        // Check if it's time to read sensor 1
        if (currentMillis - lastReadTime1 >= interval1) {
            lastReadTime1 = currentMillis;
            int sensorValue1 = analogRead(SENSOR_PIN_1);
            Serial.printf("Sensor 1 Value: %d\n", sensorValue1);

            if (WiFi.isConnected() && mqttClient.connected()) {
                String payload = String(sensorValue1);
                if (mqttClient.publish("DMA/Energy/data", payload.c_str())) {
                    Serial.println("Sensor 1 data published to MQTT.");
                } else {
                    Serial.println("Failed to publish sensor 1 data.");
                }
            } else {
                Serial.println("WiFi or MQTT not connected. Skipping MQTT publish for sensor 1.");
            }
        }

        // Check if it's time to read sensor 2
        if (currentMillis - lastReadTime2 >= interval2) {
            lastReadTime2 = currentMillis;
            int sensorValue2 = analogRead(SENSOR_PIN_2);
            Serial.printf("Sensor 2 Value: %d\n", sensorValue2);

            if (WiFi.isConnected() && mqttClient.connected()) {
                String payload = String(sensorValue2);
                if (mqttClient.publish("DMA/Energy/data2", payload.c_str())) {
                    Serial.println("Sensor 2 data published to MQTT.");
                } else {
                    Serial.println("Failed to publish sensor 2 data.");
                }
            } else {
                Serial.println("WiFi or MQTT not connected. Skipping MQTT publish for sensor 2.");
            }
        }

        vTaskDelay(10 / portTICK_PERIOD_MS); // Short delay to allow FreeRTOS scheduler to run other tasks
    }
}

void display() {

}

void read_modbus() {

}

void write_sd() {

}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
    preferences.begin("connection", false);

    // Start the WiFi and MQTT management task
    xTaskCreatePinnedToCore(
        wifi_mqtt_task, "wifi_mqtt_task", 4096, NULL, 1, NULL, 0
    );

    // Start the button management task
    xTaskCreatePinnedToCore(
        button_task, "button_task", 2048, NULL, 1, NULL, 1
    );

    // Start the sensor management task
    xTaskCreatePinnedToCore(
        sensor_task, "sensor_task", 2048, NULL, 1, NULL, 1
    );

}

void loop() {}
