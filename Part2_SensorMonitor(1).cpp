/**
 * Part 2 — Real-Time Monitoring (Sensor Read & Data)
 * Team10 — reads DHT22, DS18B20, MH-Z19, MQ-2, PIR, BH1750
 */
#include "sensor_manager.h"
#include "config.h"

#include <DHT.h>
#include <Wire.h>
#include <BH1750.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <MHZ19.h>

static DHT dht(PIN_DHT22, DHT22);
static BH1750 lightMeter;
static OneWire oneWire(PIN_DS18B20);
static DallasTemperature ds18b20(&oneWire);
static MHZ19 mhz19;

void SensorManager::begin() {
  pinMode(PIN_PIR, INPUT);
  pinMode(PIN_MQ2_ADC, INPUT);
  dht.begin();
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);
  ds18b20.begin();
  ds18b20.setResolution(12);
  Serial2.begin(9600, SERIAL_8N1, PIN_MHZ19_RX, PIN_MHZ19_TX);
  mhz19.begin(Serial2);
  mhz19.autoCalibration(true);
}

void SensorManager::calibrateMq2() {
  float sum = 0.0f;
  for (int i = 0; i < MQ2_BASELINE_SAMPLES; i++) {
    sum += analogRead(PIN_MQ2_ADC);
    delay(100);
  }
  mq2Baseline_ = sum / MQ2_BASELINE_SAMPLES;
  mq2Calibrated_ = true;
}

float SensorManager::readMq2Normalized() {
  int raw = analogRead(PIN_MQ2_ADC);
  if (!mq2Calibrated_ || mq2Baseline_ < 1.0f)
    return constrain(raw / 4095.0f, 0.0f, 1.0f);
  float ratio = (raw - mq2Baseline_) / (4095.0f - mq2Baseline_);
  return constrain(ratio, 0.0f, 1.0f);
}

SensorReadings SensorManager::read() {
  SensorReadings r{};
  r.temperatureC = dht.readTemperature();
  r.humidityPct = dht.readHumidity();
  r.validDht = !(isnan(r.temperatureC) || isnan(r.humidityPct));

  ds18b20.requestTemperatures();
  r.ambientTempC = ds18b20.getTempCByIndex(0);

  r.co2Ppm = mhz19.getCO2(false);
  if (r.co2Ppm < 0) { r.validCo2 = false; r.co2Ppm = 0.0f; }

  r.gasSmokeRatio = readMq2Normalized();

  bool pir = digitalRead(PIN_PIR) == HIGH;
  unsigned long now = millis();
  if (pir) { lastMotionMs_ = now; motionLatched_ = true; }
  else if (now - lastMotionMs_ > MOTION_COOLDOWN_MS) motionLatched_ = false;
  r.motionDetected = motionLatched_;

  r.lightLux = lightMeter.readLightLevel();
  return r;
}
