#include <Arduino.h>
#include <FastLED.h>

#define BUTTON_PIN 35               // Adjust to your button GPIO pin
#define BUTTON_HOLD_TIME 3000      // 3 seconds in milliseconds
unsigned long buttonPressTime = 0; // Tracks button press duration
bool isConfigMode = false;         // Tracks if Wi-Fi config mode is active


// MQTT settings   &&&   Topic
const char* mqtt_broker = "broker2.dma-bd.com";
const int mqtt_port = 1883;
const char* mqtt_user = "broker2";
const char* mqtt_pass = "Secret!@#$1234";

#define WORK_PACKAGE                                "1102"
#define DEVICE_TYPE                                 "00"
#define DEVICE_CODE_UPLOAD_DATE                     "240912"
#define DEVICE_SERIAL_ID                            "0000"

#define TEMPERATURE_SEND_INTERVAL                   5000

#define DEVICE_ID                                   WORK_PACKAGE DEVICE_TYPE DEVICE_CODE_UPLOAD_DATE DEVICE_SERIAL_ID

// MQTT topics
#define DEFAULT_MQTT_SUB_TOPIC                      ("DMA/EnergyMeter/" DEVICE_ID)
#define DEFAULT_MQTT_PUB_TOPIC					    ("DMA/EnergyMeter/PUB/" DEVICE_ID)



// Define the number of LEDs and the data pin
#define NUM_LEDS                                    1
#define DATA_PIN 4

CRGB leds[NUM_LEDS];// Create an array of LEDs

#define USE_FASTLED                                 1

#if USE_FASTLED
// FastLED control
void GreenBlink() { // green blink
  leds[0] = CRGB::Green;
  FastLED.show();
  delay(120);
  leds[0] = CRGB::Black;
  FastLED.show();
  delay(120);
  leds[0] = CRGB::Green;
  FastLED.show();
  delay(120);
  leds[0] = CRGB::Black;
  FastLED.show();
}

void BlueBlink() {
  leds[0] = CRGB::Blue; // blue blink
  FastLED.show();
  delay(120);
  leds[0] = CRGB::Black;
  FastLED.show();
  delay(120);
  leds[0] = CRGB::Blue;
  FastLED.show();
  delay(120);
  leds[0] = CRGB::Black;
  FastLED.show();
}

void YellowBlink() {
  leds[0] = CRGB::Yellow; // blue blink
  FastLED.show();
  delay(120);
  leds[0] = CRGB::Black;
  FastLED.show();
  delay(120);
  leds[0] = CRGB::Yellow;
  FastLED.show();
  delay(120);
  leds[0] = CRGB::Black;
  FastLED.show();
}

void RedBlink() {
  leds[0] = CRGB::Yellow; // blue blink
  FastLED.show();
  delay(120);
  leds[0] = CRGB::Black;
  FastLED.show();
  delay(120);
  leds[0] = CRGB::Yellow;
  FastLED.show();
  delay(120);
  leds[0] = CRGB::Black;
  FastLED.show();
}

void RedLed() {
  leds[0] = CRGB::Red;
  FastLED.show();
}

void GreenLed() {
  leds[0] = CRGB::Green;
  FastLED.show();
}

void BlueLed() {
  leds[0] = CRGB::Blue;
  FastLED.show();
}

void YellowLed() {
  leds[0] = CRGB::Yellow;
  FastLED.show();
}

void BlackLed() {
  leds[0] = CRGB::Black;
  FastLED.show();
}

#else
// RGB LED control via pins
#define RED_LED_PIN    2
#define GREEN_LED_PIN  26
#define BLUE_LED_PIN   25

void GreenBlink() {
  digitalWrite(GREEN_LED_PIN, LOW);    
  delay(120);
  digitalWrite(GREEN_LED_PIN, HIGH);
  delay(120);
  digitalWrite(GREEN_LED_PIN, LOW);
  delay(120);
  digitalWrite(GREEN_LED_PIN, HIGH);
}

void BlueBlink() {
  digitalWrite(BLUE_LED_PIN, LOW);
  delay(120);
  digitalWrite(BLUE_LED_PIN, HIGH);
  delay(120);
  digitalWrite(BLUE_LED_PIN, LOW);
  delay(120);
  digitalWrite(BLUE_LED_PIN, HIGH);
}
void YellowBlink() {
  digitalWrite(BLUE_LED_PIN, LOW);
  digitalWrite(RED_LED_PIN, LOW);
  delay(120);
  digitalWrite(BLUE_LED_PIN, HIGH);
  digitalWrite(RED_LED_PIN, HIGH);
  delay(120);
  digitalWrite(BLUE_LED_PIN, LOW);
  digitalWrite(RED_LED_PIN, LOW);
  delay(120);
  digitalWrite(BLUE_LED_PIN, HIGH);
  digitalWrite(RED_LED_PIN, HIGH);
}
void RedBlink() {
  digitalWrite(RED_LED_PIN, LOW);
  delay(120);
  digitalWrite(RED_LED_PIN, HIGH);
  delay(120);
  digitalWrite(RED_LED_PIN, LOW);
  delay(120);
  digitalWrite(RED_LED_PIN, HIGH);
}

void RedLed() {
  digitalWrite(RED_LED_PIN, LOW);
}

void GreenLed() {
  digitalWrite(GREEN_LED_PIN, LOW);
}

void BlueLed() {
  digitalWrite(BLUE_LED_PIN, LOW);
}

void YellowLed() {
  digitalWrite(RED_LED_PIN, LOW);
}

void BlackLed() {
  digitalWrite(RED_LED_PIN, HIGH);
  digitalWrite(GREEN_LED_PIN, HIGH);
  digitalWrite(BLUE_LED_PIN, HIGH);
}
#endif
