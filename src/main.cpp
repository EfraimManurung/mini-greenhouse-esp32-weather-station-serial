/*
  esp32-weather-station
  The purpose of this program is to monitor outside greenouse environment.
  
  Components:
  - CCS811 : CO2 Sensor
  - BH1750 : Lux Sensor
  - BME280 : Humidity, temperature and pressure
  
  Author: Efraim Manurung

  Information Technology Group, Wageningen University
  */

// include the libraries needed
#include <Arduino.h>
#include <Wire.h>

#include <BH1750.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "DFRobot_CCS811.h"

#define SEALEVELPRESSURE_HPA (1013.25)
#define LED 2

// make class objects
BH1750 bh1750_sensor;
Adafruit_BME280 bme280_sensor; // I2C
DFRobot_CCS811 ccs811_sensor;

// prototypes functions
void bh1750_read_sensor_value();
void bme280_read_sensor_values();
void ccs811_read_sensor_values();

void setup(){
  Serial.begin(9600);

  // Initialize the I2C bus (BH1750 library doesn't do this automatically)
  Wire.begin();

  bool status_bh1750, status_bme280, status_cc811;

  // Start sensors
  status_bh1750 = bh1750_sensor.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23);
  status_bme280 = bme280_sensor.begin(0x76); 
  status_cc811  = ccs811_sensor.begin();

  if (!status_bh1750) {
    Serial.println("Could not find a valid BH1750 sensor, check wiring!");
    while (1);
  }

  if (!status_bme280) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  if (status_cc811 != 0) {
    Serial.println("Could not find a valid CCS811 sensor, check wiring!");
    while (1000);
  }

  pinMode(LED, OUTPUT);
}

// Main loop/
void loop() {
  bh1750_read_sensor_value();
  bme280_read_sensor_values();
  ccs811_read_sensor_values();

  Serial.println("");
  delay(5000);
}

void bh1750_read_sensor_value() {
  float lux = bh1750_sensor.readLightLevel();

  Serial.print("lux: "); Serial.println(lux);
}

void bme280_read_sensor_values() {
  float temperature = bme280_sensor.readTemperature();
  float humidity = bme280_sensor.readHumidity();

  Serial.print("temp: "); Serial.println(temperature);
  Serial.print("hum: "); Serial.println(humidity);
}

void ccs811_read_sensor_values() {
  float co2, tvco2;
  if(ccs811_sensor.checkDataReady() == true){
    co2 = ccs811_sensor.getCO2PPM();
    tvco2 = ccs811_sensor.getTVOCPPB();
    Serial.print("co2: "); Serial.println(co2);
    Serial.print("tvco2: "); Serial.println(tvco2);

    } else {
      Serial.println("Data is not ready!");
    }

  ccs811_sensor.writeBaseLine(0x447B);

  delay(1000);
}