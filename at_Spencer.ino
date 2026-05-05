#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP5xx.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_ISM330DHCX.h>
#include <SparkFun_u-blox_GNSS_v3.h>

#define LED_PIN 3
#define PPS_PIN 15
#define BUZZER_PIN 14

Adafruit_BMP5xx bmp; // temp / pres
Adafruit_LIS3MDL lis3mdl = Adafruit_LIS3MDL(); //
Adafruit_ISM330DHCX ism330dhcx;
SFE_UBLOX_GNSS myGNSS;

sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;

char I2C_transmit_buffer[98];
volatile uint8_t I2C_buffer_position = 6;
volatile short can_blink = 0;

volatile int time_of_last_pps = 0;
const int pps_delay = 0; //Only ever change this variable. Everything else should be set.
//It is the time (milliseconds) that Jacob's board waits from the last PPS

void saveGPSData(UBX_NAV_PVT_data_t *ubxDataStruct){
    int32_t *buf32 = (int32_t*)I2C_transmit_buffer;

    buf32[6] = ubxDataStruct->lat; //Yeah look this up it's magic
    buf32[7] = ubxDataStruct->lon;
    buf32[8] = ubxDataStruct->hMSL;
    buf32[9] = ubxDataStruct->velE;
    buf32[10] = ubxDataStruct->velN;
    buf32[11] = ubxDataStruct->velD;

    *((int64_t*)(I2C_transmit_buffer + 48)) = ubxDataStruct->iTOW; 
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(PPS_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PPS_PIN), handleInterrupt, RISING);

  memcpy(I2C_transmit_buffer, "KJ5NPP", 6);

  Wire.begin(400000);

  //BMP SETUP (Pressure+temp)
  while (!bmp.begin(BMP5XX_ALTERNATIVE_ADDRESS, &Wire)) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(30);
  }

  bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_16X);
  bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_16X);
  bmp.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP5XX_ODR_50_HZ); //Set this lower for slower speeds
  bmp.setPowerMode(BMP5XX_POWERMODE_NORMAL);
  bmp.enablePressure(true);
  bmp.configureInterrupt(BMP5XX_INTERRUPT_LATCHED, BMP5XX_INTERRUPT_ACTIVE_HIGH, BMP5XX_INTERRUPT_PUSH_PULL, BMP5XX_INTERRUPT_DATA_READY, true);

  while (!lis3mdl.begin_I2C()) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }

  lis3mdl.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_1000_HZ);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  lis3mdl.setIntThreshold(500);

  //ISM Setup (Accel + gyro)
  while(!ism330dhcx.begin_I2C()) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(1000);
  }
  ism330dhcx.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
  ism330dhcx.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  ism330dhcx.setAccelDataRate(LSM6DS_RATE_6_66K_HZ);
  ism330dhcx.setGyroDataRate(LSM6DS_RATE_6_66K_HZ);

  while (myGNSS.begin() == false) {
    digitalWrite(LED_PIN, HIGH);
    delay(1000);
    digitalWrite(LED_PIN, LOW);
    delay(500);
  }

  myGNSS.setAutoPVT(true);
  myGNSS.setAutoPVTcallbackPtr(saveGPSData);

  digitalWrite(LED_PIN, LOW);
}

void loop() {
  I2C_buffer_position = 6 + 50; //6 For callsign 32 for GPS data 

  if(bmp.performReading()){
    memcpy(I2C_transmit_buffer + I2C_buffer_position, &bmp.temperature, 4);
    I2C_buffer_position += 4;
    memcpy(I2C_transmit_buffer + I2C_buffer_position, &bmp.pressure, 4);
    I2C_buffer_position += 4;
  }
  else{
    I2C_buffer_position += 8;
  }
  lis3mdl.read();

  memcpy(I2C_transmit_buffer + I2C_buffer_position, &lis3mdl.x, 2);
  I2C_buffer_position += 2;
  memcpy(I2C_transmit_buffer + I2C_buffer_position, &lis3mdl.y, 2);
  I2C_buffer_position += 2;
  memcpy(I2C_transmit_buffer + I2C_buffer_position, &lis3mdl.z, 2);
  I2C_buffer_position += 2;

  ism330dhcx.getEvent(&accel, &gyro, &temp);

  memcpy(I2C_transmit_buffer + I2C_buffer_position, &temp.temperature, 4);
  I2C_buffer_position += 4;
  memcpy(I2C_transmit_buffer + I2C_buffer_position, &gyro.gyro.x, 4);
  I2C_buffer_position += 4;
  memcpy(I2C_transmit_buffer + I2C_buffer_position, &gyro.gyro.y, 4);
  I2C_buffer_position += 4;
  memcpy(I2C_transmit_buffer + I2C_buffer_position, &gyro.gyro.z, 4);
  I2C_buffer_position += 4;
  memcpy(I2C_transmit_buffer + I2C_buffer_position, &accel.acceleration.x, 4);
  I2C_buffer_position += 4;
  memcpy(I2C_transmit_buffer + I2C_buffer_position, &accel.acceleration.y, 4);
  I2C_buffer_position += 4;
  memcpy(I2C_transmit_buffer + I2C_buffer_position, &accel.acceleration.z, 4);

  Wire.beginTransmission(0xAA);
  Wire.write(I2C_transmit_buffer, 98);
  Wire.endTransmission();

  if (millis() - time_of_last_pps > 550 + pps_delay && can_blink){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    Wire.beginTransmission(0xBB);
    Wire.write(I2C_transmit_buffer, 98);
    Wire.endTransmission();
    can_blink = 0;
  }
}

void handleInterrupt(void) {
  time_of_last_pps = millis();
  can_blink = 1;
  digitalWrite(LED_PIN, LOW);
}