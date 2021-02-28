#include <Arduino.h>
#include <BlynkSimpleEsp8266.h>
#include <Ticker.h>

#define ZERO_PIN D1
#define DIMMER_PIN D2
#define LED_PIN D8
#define TEMP_SENSOR A0

#define INTERVAL_CALCULATE 2000
#define INTERVAL_UPDATE_TEMP 2000

// задаём значение 500us-9300us, где 500us максимум мощности, 9300us минимум!!!
// 5 coefficient timer, TIM_DIV16 = 1,  //5MHz (5 ticks/us - 1677721.4 us max)
// задаём новое значение 2500-46500
#define TIME_MAX_POWER_US 2500
#define TIME_MIN_POWER_US 46500

#define NUM_READINGS_SENSOR 30

#define MAX_TEMP_LEVEL 50  //Петля гистерезиса
#define MIN_TEMP_LEVEL 47
boolean temperatureIsAbove = false;

const float kp = 5.0;
const float ki = 0.3;
const float dt = 2.5;

Ticker tickerPi;
Ticker tickerGetTemp;
volatile float temperature = 0;

unsigned long dimmer = TIME_MIN_POWER_US;
byte tempSetup = MIN_TEMP_LEVEL;
boolean power = false;
boolean temperatureMaintenance = false;

char auth[] = "xZK47KRWZj0Rh6a1iZ2uqv3euLEOQR5h";
char ssid[] = "Keenetic-8669";
char pass[] = "299792458";