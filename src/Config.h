#include <Arduino.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <SD.h>
#include <SPI.h>

String modbusdata;
String status = "OK";  // Example status

#define WORK_PACKAGE                                "1146"
#define DEVICE_TYPE                                 "00"
#define DEVICE_CODE_UPLOAD_DATE                     "240912"
#define DEVICE_SERIAL_ID                            "0000"

#define DEVICE_ID                                   WORK_PACKAGE DEVICE_TYPE DEVICE_CODE_UPLOAD_DATE DEVICE_SERIAL_ID

// MQTT and Wi-Fi Configuration
#define MQTT_SERVER                                 "broker2.dma-bd.com"
#define MQTT_PORT                                   1883
#define MQTT_USER                                   "broker2"
#define MQTT_PASSWORD                               "Secret!@#$1234"
#define WIFI_MAX_RETRIES                            30
#define MQTT_MAX_RETRIES                            30
#define MQTT_INITIAL_BACKOFF                        1000 // ms
#define BACKOFF_MULTIPLIER                          2


#define DEFAULT_MQTT_PUB_TOPIC					    "DMA/EnergyMeter/PUB"
#define MQTT_PUBLISH_TOPIC_SENSOR_1					"DMA/EnergyMeter/PUB"
#define MQTT_SUBSCRIBE_TOPIC                        "DMA/EnergyMeter/" DEVICE_ID

// GPIO Pins and Constants
// const int chipSelect = 5;
#define chipSelect                                  5
#define BUTTON_PIN                                  35
#define SENSOR_PIN_1                                39
#define SENSOR_PIN_2                                36
#define LONG_PRESS_DURATION                         3000 // 3 seconds


// #define RE_PIN                                      23
// #define DE_PIN                                      23
// #define RO_PIN                                      5
// #define DI_PIN                                      4
// Core PCB Pins MAX485 to ESP2: RO=5, DI=4, DE,RE=23

#define ESP_BAUDRATE                                115200
#define MODBUS_BAUDRATE                             9600

#define SENSOR_DATA_SENDING_PERIOD                  5000       // 60000 // miliseconds
// Sensor Intervals (ms)
const unsigned long INTERVAL_SENSOR_1 = 5000;
const unsigned long INTERVAL_SENSOR_2 = 10000;




#define taeHigh_reg_addr                            0x30
#define taeLow_reg_addr                             0x31
#define activePower_reg_addr                        0x1A
#define pAvolt_reg_addr                             0x14
#define pBvolt_reg_addr                             0x15
#define pCvolt_reg_addr                             0x16
#define lABvolt_reg_addr                            0x17
#define lBCvolt_reg_addr                            0x18
#define lCAvolt_reg_addr                            0x19
#define pAcurrent_reg_addr                          0x10
#define pBcurrent_reg_addr                          0x11
#define pCcurrent_reg_addr                          0x12
#define frequency_reg_addr                          0x1E
#define powerfactor_reg_addr                        0x1D

// modbus data variable
int rs485_inquiry_taeHigh;
int rs485_inquiry_taeLow;
int rs485_inquiry_activePower;
int rs485_inquiry_pAvolt;
int rs485_inquiry_pBvolt;
int rs485_inquiry_pCvolt;
int rs485_inquiry_lABvolt;
int rs485_inquiry_lBCvolt;
int rs485_inquiry_lCAvolt;
int rs485_inquiry_pAcurrent;
int rs485_inquiry_pBcurrent;
int rs485_inquiry_pCcurrent;
int rs485_inquiry_frequency;
int rs485_inquiry_powerfactor;

float actual_taeHigh;
float actual_taeLow;
float actual_activePower;
float actual_pAvolt;
float actual_pBvolt;
float actual_pCvolt;
float actual_lABvolt;
float actual_lBCvolt;
float actual_lCAvolt;
float actual_pAcurrent;
float actual_pBcurrent;
float actual_pCcurrent;
float actual_frequency;
float actual_powerfactor;
