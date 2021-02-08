#include <header.h>

void readSensor()
{
  long sum = 0;
  float average;
  for (int i = 0; i < NUM_READINGS; i++)
    sum += analogRead(TEMP_SENSOR);
  average = sum / NUM_READINGS;
  temperature = (1150 - average) / 13;
  Blynk.virtualWrite(V5, temperature);
}

int computePID(float input, float setpoint, float kp, float ki, float dt, int minOut, int maxOut)
{
  float err = setpoint - input;
  static float integral = 0;
  integral = constrain(integral + (float)err * dt * ki, minOut, maxOut);
  return constrain(err * kp + integral, minOut, maxOut);
}

void calculatePID()
{
  int result_PID = computePID(temperature, tempSetup, kp, ki, dt, 0, 50);
  dimmer = map(result_PID, 0, 50, TIME_MIN_POWER_US, TIME_MAX_POWER_US);
}

void ICACHE_RAM_ATTR isr()
{
  digitalWrite(DIMMER_PIN, 0);
  timer1_write(dimmer);
}

void ICACHE_RAM_ATTR onTimerISR()
{
  digitalWrite(DIMMER_PIN, 1);
}

BLYNK_CONNECTED()
{
  Blynk.syncVirtual(V0);
  Blynk.syncVirtual(V1);
  Blynk.syncVirtual(V2);
  Blynk.syncVirtual(V3);
  Blynk.syncVirtual(V4);
  Blynk.syncVirtual(V5);
}

BLYNK_WRITE(V0)
{
  power = param.asInt();

  if (power == 1)
  {
    timer1_attachInterrupt(onTimerISR);
    digitalWrite(LED_PIN, HIGH);
  }
  else
  {
    if (temperatureMaintenance)
    {
      temperatureMaintenance = false;
      Blynk.virtualWrite(V2, temperatureMaintenance);
    }
    timer1_detachInterrupt();
    tickerPi.detach();
    digitalWrite(LED_PIN, LOW);
  }
}

BLYNK_WRITE(V1)
{
  // задаём значение 500-9300, где 500 максимум мощности, 9300 минимум!!!
  // 5 coefficient timer, TIM_DIV16 = 1,  //5MHz (5 ticks/us - 1677721.4 us max)
  // задаём новое значение 2500-46500
  if (!temperatureMaintenance)
    dimmer = map(param.asInt(), 1023, 0, TIME_MAX_POWER_US, TIME_MIN_POWER_US);
}

BLYNK_WRITE(V2)
{
  temperatureMaintenance = param.asInt();
  if (power && !temperatureMaintenance)
  {
    tickerPi.detach();
    Blynk.syncVirtual(V1);
  }
  if (power && temperatureMaintenance)
    tickerPi.attach_ms(INTERVAL_READ_SENSOR, calculatePID);
}

BLYNK_WRITE(V3)
{
  tempSetup = param.asInt();
  Blynk.virtualWrite(V4, tempSetup);
}

void setup()
{
  Blynk.begin(auth, ssid, pass);
  pinMode(ZERO_PIN, INPUT_PULLUP);
  pinMode(DIMMER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ZERO_PIN), isr, RISING);
  timer1_isr_init();
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
  if (power)
    timer1_attachInterrupt(onTimerISR);
  else
    timer1_detachInterrupt();
  ticker.attach_ms(INTERVAL_READ_SENSOR, readSensor);
}

void loop()
{
  Blynk.run();
}