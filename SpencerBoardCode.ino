#include <Adafruit_BMP5xx.h>
#include <Adafruit_ISM330DHCX.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <Wire.h>

#define LED_PIN 3
#define PPS_PIN 15
#define BUZZER_PIN 14

const uint8_t TELEMETRY_PACKET_SIZE = 98;

struct __attribute__((packed)) GPSData {
  int32_t latitude;
  int32_t longitude;
  int32_t altitude;
  int32_t nedNorthVel;
  int32_t nedDownVel;
  int32_t nedEastVel;
  uint32_t unixEpoch;
};

struct __attribute__((packed)) BMPData {
  float temperature;
  float pressure;
};

struct __attribute__((packed)) MagnetometerData {
  int16_t x;
  int16_t y;
  int16_t z;
};

struct __attribute__((packed)) InertialData {
  float accelX;
  float accelY;
  float accelZ;
  float gyroZ;
  float gyroY;
  float gyroX;
  float temperature;
};

struct __attribute__((packed)) TelemetryData {
  char callsign[6];
  GPSData gps;
  BMPData bmp;
  MagnetometerData magnetometer;
  InertialData inertial;
  uint8_t reserved[TELEMETRY_PACKET_SIZE - 6 - sizeof(GPSData) -
                   sizeof(BMPData) - sizeof(MagnetometerData) -
                   sizeof(InertialData)];
};

static_assert(sizeof(TelemetryData) == TELEMETRY_PACKET_SIZE,
              "TelemetryData must remain 98 bytes");

Adafruit_BMP5xx bmp;
Adafruit_LIS3MDL lis3mdl = Adafruit_LIS3MDL();
Adafruit_ISM330DHCX ism330dhcx;
SFE_UBLOX_GNSS myGNSS;

TelemetryData telemetry;
volatile bool can_blink = false;

volatile unsigned long time_of_last_pps = 0;
const int pps_delay =
    0; // Only ever change this variable. Everything else should already be set.
// It is the time (milliseconds) that Jacob's board waits from the last PPS

uint8_t *telemetryBytes() { return reinterpret_cast<uint8_t *>(&telemetry); }

void sendTelemetryPacket(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(telemetryBytes(), sizeof(telemetry));
  Wire.endTransmission();
}

void saveGPSData(UBX_NAV_PVT_data_t *ubxDataStruct) {
  (void)ubxDataStruct;

  telemetry.gps.latitude = myGNSS.getLatitude();
  telemetry.gps.longitude = myGNSS.getLongitude();
  telemetry.gps.altitude = myGNSS.getAltitude();
  telemetry.gps.nedNorthVel = myGNSS.getNedNorthVel();
  telemetry.gps.nedDownVel = myGNSS.getNedDownVel();
  telemetry.gps.nedEastVel = myGNSS.getNedEastVel();
  telemetry.gps.unixEpoch = myGNSS.getUnixEpoch();
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(PPS_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PPS_PIN), handleInterrupt, RISING);

  memcpy(telemetry.callsign, "KJ5NPP", sizeof(telemetry.callsign));

  Wire.begin();
  Wire.setClock(400000);

  while (!bmp.begin(BMP5XX_ALTERNATIVE_ADDRESS, &Wire)) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(30);
  }

  bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_16X);
  bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_16X);
  bmp.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP5XX_ODR_50_HZ);
  bmp.setPowerMode(BMP5XX_POWERMODE_NORMAL);
  bmp.enablePressure(true);
  bmp.configureInterrupt(BMP5XX_INTERRUPT_LATCHED, BMP5XX_INTERRUPT_ACTIVE_HIGH,
                         BMP5XX_INTERRUPT_PUSH_PULL,
                         BMP5XX_INTERRUPT_DATA_READY, true);

  while (!lis3mdl.begin_I2C()) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }

  lis3mdl.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_1000_HZ);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  lis3mdl.setIntThreshold(500);

  while (!ism330dhcx.begin_I2C()) {
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
  if (bmp.performReading()) {
    telemetry.bmp.temperature = bmp.temperature;
    telemetry.bmp.pressure = bmp.pressure;
  }

  lis3mdl.read();
  telemetry.magnetometer.x = lis3mdl.x;
  telemetry.magnetometer.y = lis3mdl.y;
  telemetry.magnetometer.z = lis3mdl.z;

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  if (ism330dhcx.getEvent(&accel, &gyro, &temp)) {
    telemetry.inertial.accelX = accel.acceleration.x;
    telemetry.inertial.accelY = accel.acceleration.y;
    telemetry.inertial.accelZ = accel.acceleration.z;
    telemetry.inertial.gyroZ = gyro.gyro.z;
    telemetry.inertial.gyroY = gyro.gyro.y;
    telemetry.inertial.gyroX = gyro.gyro.x;
    telemetry.inertial.temperature = temp.temperature;
  }

  sendTelemetryPacket(0xAA);

  if (millis() - time_of_last_pps > 550 + pps_delay && can_blink) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    sendTelemetryPacket(0xBB);
    can_blink = 0;
  }
}

void handleInterrupt(void) {
  time_of_last_pps = millis();
  can_blink = 1;
  digitalWrite(LED_PIN, LOW);
}
