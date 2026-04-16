#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP5xx.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_ISM330DHCX.h>

#define LED_PIN 3

#define ALTITUDE_SIZE 24
#define MAG_SIZE 16
#define ACCEL_SIZE 96
#define GPS_SIZE 92
#define TEMP_SIZE 16
const uint8_t PACKET_SIZE = ALTITUDE_SIZE + MAG_SIZE + ACCEL_SIZE + GPS_SIZE + TEMP_SIZE;

struct TelemetryData {
  uint8_t altitude[ALTITUDE_SIZE];
  uint8_t magnetometer[MAG_SIZE];
  uint8_t accelerometer[ACCEL_SIZE];
  uint8_t gps[GPS_SIZE];
  uint8_t temperature[TEMP_SIZE];
};

struct SDData {
  uint8_t altitude[ALTITUDE_SIZE];
  uint8_t magnetometer[MAG_SIZE];
  uint8_t accelerometer[ACCEL_SIZE];
  uint8_t gps[GPS_SIZE];
  uint8_t temperature[TEMP_SIZE];
};

Adafruit_BMP5xx bmp;
Adafruit_LIS3MDL lis3mdl = Adafruit_LIS3MDL();
Adafruit_ISM330DHCX ism330dhcx;

sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;

volatile TelemetryData tx_packet;
volatile SDData sd_packet;

void setup() {
  Serial.begin(9600);
  delay(1000);

  setupBuzzer();

  while (1) {
    buzzAltitude(31672);
  }

  pinMode(LED_PIN, OUTPUT);

  Wire.begin();

  Serial.println("Start BMP setup");
  //BMP SETUP (Pressure+temp)
  while (!bmp.begin(BMP5XX_ALTERNATIVE_ADDRESS, &Wire)) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(10);
  }

  bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_16X);
  bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_16X);
  bmp.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP5XX_ODR_50_HZ);  //Set this lower for slower speeds
  bmp.setPowerMode(BMP5XX_POWERMODE_NORMAL);
  bmp.enablePressure(true);
  bmp.configureInterrupt(BMP5XX_INTERRUPT_LATCHED, BMP5XX_INTERRUPT_ACTIVE_HIGH, BMP5XX_INTERRUPT_PUSH_PULL, BMP5XX_INTERRUPT_DATA_READY, true);

  Serial.println("End BMP setup");

  //LIS Setup (Magnetometer)
  Serial.println("Start LIS setup");
  while (!lis3mdl.begin_I2C()) {  // Initializes using default 0x1C
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }

  lis3mdl.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_1000_HZ);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  lis3mdl.setIntThreshold(500);

  Serial.println("End LIS setup");

  //ISM Setup (Accel + gyro)
  Serial.println("Start accel setup");
  while (!ism330dhcx.begin_I2C()) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(1000);
  }
  ism330dhcx.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
  ism330dhcx.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  ism330dhcx.setAccelDataRate(LSM6DS_RATE_6_66K_HZ);
  ism330dhcx.setGyroDataRate(LSM6DS_RATE_6_66K_HZ);

  Serial.println("End accel setup");
}

void loop() {
  if (!bmp.dataReady()) { return; }

  if (!bmp.performReading()) { return; }

  Serial.print(bmp.temperature);
  Serial.print(" ");
  Serial.print(bmp.pressure);
  Serial.print(" ");

  lis3mdl.read();

  Serial.print(lis3mdl.x);
  Serial.print(" ");
  Serial.print(lis3mdl.y);
  Serial.print(" ");
  Serial.print(lis3mdl.z);
  Serial.print(" ");

  ism330dhcx.getEvent(&accel, &gyro, &temp);

  Serial.print(temp.temperature);
  Serial.print(" ");
  Serial.print(gyro.gyro.x);
  Serial.print(" ");
  Serial.print(gyro.gyro.y);
  Serial.print(" ");
  Serial.print(gyro.gyro.z);
  Serial.print(" ");
  Serial.print(accel.acceleration.x);
  Serial.print(" ");
  Serial.print(accel.acceleration.y);
  Serial.print(" ");
  Serial.print(accel.acceleration.z);
  Serial.print(" ");

  Serial.println();

  delay(10);  // Short delay since we're checking dataReady()
}