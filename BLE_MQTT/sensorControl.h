#include <Adafruit_DPS310.h>
#include "Adafruit_SHT4x.h"
#include "FastIMU.h"

// sht4x RH sensor
Adafruit_SHT4x sht4 = Adafruit_SHT4x();

// dps310 atmospheric pressure sensor
Adafruit_DPS310 dps;
Adafruit_Sensor *dps_temp = dps.getTemperatureSensor();
Adafruit_Sensor *dps_pressure = dps.getPressureSensor();

// icm-20689 6-axis imu sensor
#define IMU_ADDRESS 0x68    //Change to the address of the IMU
#define PERFORM_CALIBRATION //Comment to disable startup calibration
ICM20689 IMU;               //Change to the name of any supported IMU! 

// Currently supported IMUS: MPU9255 MPU9250 MPU6500 MPU6050 ICM20689 ICM20690 BMI055 BMX055 BMI160 LSM6DS3 LSM6DSL
calData calib = { 0 };  //Calibration data
AccelData accelData;    //Sensor data
GyroData gyroData;


void setSensorAlt(){
  if (dps.begin_I2C()) {
    Serial.println("DPS OK!");
    // Setup highest precision
    dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
    dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);

    dps_temp->printSensorDetails();
    dps_pressure->printSensorDetails();
  }else{
    Serial.println("Failed to find DPS");
    // while (1) yield();
  }
}
void setSensorRH(){
  //sht4x RH sensor initialization
  Serial.println("Adafruit SHT4x test");
  if (sht4.begin()) {
    Serial.println("Found SHT4x sensor");
    Serial.print("Serial number 0x");
    Serial.println(sht4.readSerial(), HEX);
    // You can have 3 different precisions, higher precision takes longer
    sht4.setPrecision(SHT4X_HIGH_PRECISION);
    switch (sht4.getPrecision()) {
      case SHT4X_HIGH_PRECISION: 
        Serial.println("High precision");
        break;
      case SHT4X_MED_PRECISION: 
        Serial.println("Med precision");
        break;
      case SHT4X_LOW_PRECISION: 
        Serial.println("Low precision");
        break;
    }
    // You can have 6 different heater settings
    // higher heat and longer times uses more power
    // and reads will take longer too!
    sht4.setHeater(SHT4X_NO_HEATER);
    switch (sht4.getHeater()) {
      case SHT4X_NO_HEATER: 
        Serial.println("No heater");
        break;
      case SHT4X_HIGH_HEATER_1S: 
        Serial.println("High heat for 1 second");
        break;
      case SHT4X_HIGH_HEATER_100MS: 
        Serial.println("High heat for 0.1 second");
        break;
      case SHT4X_MED_HEATER_1S: 
        Serial.println("Medium heat for 1 second");
        break;
      case SHT4X_MED_HEATER_100MS: 
        Serial.println("Medium heat for 0.1 second");
        break;
      case SHT4X_LOW_HEATER_1S: 
        Serial.println("Low heat for 1 second");
        break;
      case SHT4X_LOW_HEATER_100MS: 
        Serial.println("Low heat for 0.1 second");
        break;
    }
  }else{
    Serial.println("Couldn't find SHT4x");
    // while (1) delay(1);
  }
}


float getSensorPressure(){
  sensors_event_t temp_event, pressure_event;
  if (dps.pressureAvailable()) {
    dps_pressure->getEvent(&pressure_event);
    Serial.print(pressure_event.pressure);
  }
  return pressure_event.pressure;
  // return (float)12.345f;
}

void getSensorDataAlt(){
  sensors_event_t temp_event, pressure_event;
  if (dps.temperatureAvailable()) {
    dps_temp->getEvent(&temp_event);
    Serial.print(F("Temperature = "));
    Serial.print(temp_event.temperature);
    Serial.println(" *C");
  }
  // Reading pressure also reads temp so don't check pressure
  // before temp!
  if (dps.pressureAvailable()) {
    dps_pressure->getEvent(&pressure_event);
    Serial.print(F("Pressure = "));
    Serial.print(pressure_event.pressure);
    Serial.println(" hPa"); 
  }
}

void getSensorDataRH(){
  sensors_event_t humidity, temp;
  uint32_t timestamp = millis();
  timestamp = millis() - timestamp;
  sht4.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data

  Serial.print("Temperature: "); Serial.print(temp.temperature); Serial.println(" degrees C");
  Serial.print("Humidity: "); Serial.print(humidity.relative_humidity); Serial.println("% rH");
  Serial.print("Read duration (ms): ");
  Serial.println(timestamp);
}

void getSensorDataTRH(int8_t* dst_arr){
  // sensors_event_t humidity, temp;
  // uint32_t timestamp = millis();
  // timestamp = millis() - timestamp;
  // sht4.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data

  // Serial.print("Temperature: "); Serial.print(temp.temperature); Serial.println(" degrees C");
  // Serial.print("Humidity: "); Serial.print(humidity.relative_humidity); Serial.println("% rH");
  // Serial.print("Read duration (ms): ");
  // Serial.println(timestamp);

  int16_t buf[3] = {
    (int16_t)(20*10),
    (int16_t)(20*20),
  };
  memcpy(dst_arr,buf,3);
}

void setSensorIMU(){
    int err = IMU.init(calib, IMU_ADDRESS);
  if (err != 0) {
    Serial.print("Error initializing IMU: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }
  
  //err = IMU.setGyroRange(500);      //USE THESE TO SET THE RANGE, IF AN INVALID RANGE IS SET IT WILL RETURN -1
  //err = IMU.setAccelRange(2);       //THESE TWO SET THE GYRO RANGE TO ±500 DPS AND THE ACCELEROMETER RANGE TO ±2g
  
  if (err != 0) {
    Serial.print("Error Setting range: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }
}

void getSensorDataIMU(int8_t* dst_arr){
  IMU.update();
  IMU.getAccel(&accelData);
  Serial.println("imu:");
  Serial.print(accelData.accelX);
  Serial.print("\t");
  Serial.print(accelData.accelY);
  Serial.print("\t");
  Serial.print(accelData.accelZ);
  Serial.print("\t");
  Serial.println(IMU.getTemp());

  int8_t buf[3] = {
    (int8_t)(20*accelData.accelX),
    (int8_t)(20*accelData.accelY),
    (int8_t)(20*accelData.accelZ)
  };
  memcpy(dst_arr,buf,3);

  return;
}