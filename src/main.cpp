/*
  ESP32 Weather Station 
  The purpose of this program is to monitor outside greenouse environment.

  Author: Efraim Manurung
  MSc Thesis in Information Technology Group, Wageningen University

  efraim.manurung@gmail.com
  
  Components:
  - ESP32  : Microcontroller 
  - CCS811 : CO2 Sensor
  - BH1750 : Lux Sensor
  - BME280 : Humidity, temperature and pressure
  */

// include the libraries needed
#include <Arduino.h>
#include <Wire.h>

#include <BH1750.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "DFRobot_CCS811.h"
#include <MHZ19.h>

#define SEALEVELPRESSURE_HPA (1013.25)
#define LED 2

// make class objects
BH1750 bh1750_sensor;
Adafruit_BME280 bme280_sensor; // I2C
DFRobot_CCS811 ccs811_sensor;
MHZ19 mhz(&Serial2);

// prototypes functions
void bh1750_read_sensor_value();
void bme280_read_sensor_values();
void ccs811_read_sensor_values();
void mhz19c_read_sensor_values();

void setup() 
{
  // Start the serial begin for MHZ19C
  Serial2.begin(9600);

  // Start the serial begin to the computer
  Serial.begin(9600);

  // Initialize the I2C bus (BH1750 library doesn't do this automatically)
  Wire.begin();

  bool status_bh1750, status_bme280, status_cc811;

  // Start sensors
  status_bh1750 = bh1750_sensor.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23);
  status_bme280 = bme280_sensor.begin(0x76); 
  status_cc811  = ccs811_sensor.begin();

  if (!status_bh1750) 
  {
    Serial.println("Could not find a valid BH1750 sensor, check wiring!");
    while (1);
  }

  if (!status_bme280) 
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  if (status_cc811 != 0) 
  {
    Serial.println("Could not find a valid CCS811 sensor, check wiring!");
    while (1000);
  }

  pinMode(LED, OUTPUT);
}

// Main loop/
void loop() 
{
  bh1750_read_sensor_value();
  bme280_read_sensor_values();
  ccs811_read_sensor_values();
  mhz19c_read_sensor_values();

  Serial.println("");
  delay(5000);
}

void bh1750_read_sensor_value() 
{
  float lux = bh1750_sensor.readLightLevel();

  Serial.print("lux: "); Serial.println(lux);
}

void bme280_read_sensor_values() 
{
  float temperature = bme280_sensor.readTemperature();
  float humidity = bme280_sensor.readHumidity();

  Serial.print("temp: "); Serial.println(temperature);
  Serial.print("hum: "); Serial.println(humidity);
}

void ccs811_read_sensor_values() 
{
  float ccs_co2, ccs_tvco2;
  if(ccs811_sensor.checkDataReady() == true){
    ccs_co2 = ccs811_sensor.getCO2PPM();
    ccs_tvco2 = ccs811_sensor.getTVOCPPB();
    Serial.print("ccs_co2: "); Serial.println(ccs_co2);
    Serial.print("ccs_tvco2: "); Serial.println(ccs_tvco2);

    } else {
      Serial.println("Data is not ready!");
    }

  ccs811_sensor.writeBaseLine(0x447B);

  delay(1000);
}

void mhz19c_read_sensor_values() 
{
  MHZ19_RESULT response = mhz.retrieveData();
  float co2, temp_co2;
  if (response == MHZ19_RESULT_OK) {
    co2 = mhz.getCO2();
    temp_co2 = mhz.getTemperature();
    Serial.print("co2: "); Serial.println(co2);
    Serial.print("temp_co2: "); Serial.print(temp_co2);
  }
  else
  {
    Serial.print(F("Error, code: "));;
    Serial.println(response);
  }
}