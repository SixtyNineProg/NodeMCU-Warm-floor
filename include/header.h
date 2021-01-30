#include <Arduino.h>
#include <BlynkSimpleEsp8266.h>
#include <Ticker.h>
#include <DHT.h>

#define ZERO_PIN D1
#define DIMMER_PIN D2
#define LED_PIN D8
#define DHT_PIN D5
#define INTERVAL_READ_SENSOR 2000
#define INTERVAL_PI_REGULATOR 2500

#define TIME_MAX_POWER_US 2500
#define TIME_MIN_POWER_US 46500
#define MIN_TEMP_LEVEL 0
#define MAX_TEMP_LEVEL 50

const float kp = 5.0;
const float ki = 0.3;
const float dt = 2.5;

Ticker ticker;
Ticker tickerPi;
DHT11 sensor;
volatile float temperature = 0;
volatile uint8_t error = 0;
volatile int8_t result = 0;

unsigned long dimmer = TIME_MIN_POWER_US;
byte tempSetup = MIN_TEMP_LEVEL;
boolean power = false;
boolean temperatureMaintenance = false;

char auth[] = "xZK47KRWZj0Rh6a1iZ2uqv3euLEOQR5h";
char ssid[] = "Keenetic-8669";
char pass[] = "299792458";