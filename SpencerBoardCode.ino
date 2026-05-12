#include <Adafruit_BMP5xx.h>
#include <Adafruit_ISM330DHCX.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <Wire.h>
#include <TelemetryData.h>

#define LED_PIN 3
#define PPS_PIN 15
// Set GPS_INT_PIN to the Teensy pin wired to the GPS INT/data-ready pin.
// Leave it at -1 while the INT line is not soldered; GPS servicing will then
// fall back to a timed non-blocking poll.
#define GPS_INT_PIN -1
#define BUZZER_PIN 14
#define BUILTIN_LED_PIN 13

const uint8_t I2C_ABRAHAM_ADDRESS = 0x09;
const uint8_t I2C_JACOB_ADDRESS = 0x08;
const uint8_t I2C_FRAME_MAX_SIZE = 32;
const uint8_t I2C_FRAME_HEADER_SIZE = 2;
const uint8_t I2C_FRAME_PAYLOAD_SIZE =
    I2C_FRAME_MAX_SIZE - I2C_FRAME_HEADER_SIZE;
const unsigned long GPS_VALID_TIMEOUT_MS = 2000;
const unsigned long GPS_POLL_INTERVAL_MS = 20;
const uint8_t GPS_NAVIGATION_FREQUENCY_HZ = 10;

// I2C frame header byte 0:
//   bit 0: frame is intended for the SD-card receiver
//   bit 1: frame is intended for the radio/antenna receiver
//   bit 7: frame starts a new telemetry packet; clear means continuation
// I2C frame header byte 1 is an 8-bit checksum of the payload bytes that
// follow the header. Receivers should drop the current packet on checksum
// failure and wait for the next frame with I2C_FRAME_START set.
const uint8_t I2C_FRAME_DESTINATION_SD = 1 << 0;
const uint8_t I2C_FRAME_DESTINATION_RADIO = 1 << 1;
const uint8_t I2C_FRAME_DESTINATION_BOTH =
    I2C_FRAME_DESTINATION_SD | I2C_FRAME_DESTINATION_RADIO;
const uint8_t I2C_FRAME_START = 1 << 7;

const uint8_t I2C_STATUS_SUCCESS = 0;
const uint8_t I2C_STATUS_SHORT_WRITE = 0xFE;

Adafruit_BMP5xx bmp;
Adafruit_LIS3MDL lis3mdl = Adafruit_LIS3MDL();
Adafruit_ISM330DHCX ism330dhcx;
SFE_UBLOX_GNSS myGNSS;

TelemetryData telemetry;
uint16_t nextPacketCounter = 0;
volatile bool can_blink = false;
volatile bool gps_service_requested = true;

volatile unsigned long time_of_last_pps = 0;
unsigned long time_of_last_gps_update = 0;
unsigned long time_of_last_gps_poll = 0;

// Only ever change this variable. Everything else should already be set.
const int pps_delay = 0;
// It is the time (milliseconds) that Jacob's board waits from the last PPS

uint8_t *telemetryBytes() { return reinterpret_cast<uint8_t *>(&telemetry); }

void setValidityFlag(uint8_t flag, bool valid) {
  if (valid) {
    telemetry.validity |= flag;
  } else {
    telemetry.validity &= ~flag;
  }
}

/** Clears stale GPS flags; saveGPSData sets them from fresh PVT. */
void refreshGPSValidity() {
  if (time_of_last_gps_update == 0 ||
      millis() - time_of_last_gps_update > GPS_VALID_TIMEOUT_MS) {
    setValidityFlag(VALIDITY_GPS, false);
    setValidityFlag(VALIDITY_GPS_UNIX_EPOCH, false);
  }
}

uint8_t checksumI2CPayload(const uint8_t *payload, size_t payloadSize) {
  uint8_t checksum = 0;
  for (size_t index = 0; index < payloadSize; index++) {
    checksum += payload[index];
  }
  return checksum;
}

/** Call once before one or more sendTelemetryPacket() calls for the same snapshot. */
void prepareTelemetryForI2CSend() {
  telemetry.wireLength = sizeof(TelemetryData);
  telemetry.packetCounter = nextPacketCounter++;
}

/** Send current telemetry buffer as framed chunks to one I2C slave (7-bit address). */
uint8_t sendTelemetryPacket(uint8_t slave7Address, uint8_t destinationFlags) {
  // Blink the LED if we're sending a packet.
  digitalWrite(LED_PIN, HIGH);

  uint8_t *bytes = telemetryBytes();
  size_t totalBytesWritten = 0;
  uint8_t finalStatus = I2C_STATUS_SUCCESS;

  for (size_t offset = 0; offset < sizeof(telemetry);
       offset += I2C_FRAME_PAYLOAD_SIZE) {
    size_t bytesRemaining = sizeof(telemetry) - offset;
    size_t payloadSize = bytesRemaining < I2C_FRAME_PAYLOAD_SIZE
                             ? bytesRemaining
                             : I2C_FRAME_PAYLOAD_SIZE;
    uint8_t frameFlags = destinationFlags;
    if (offset == 0) {
      frameFlags |= I2C_FRAME_START;
    }

    Wire.beginTransmission(slave7Address);
    size_t bytesWritten = Wire.write(frameFlags);
    uint8_t payloadChecksum = checksumI2CPayload(bytes + offset, payloadSize);
    bytesWritten += Wire.write(payloadChecksum);
    bytesWritten += Wire.write(bytes + offset, payloadSize);
    totalBytesWritten += bytesWritten;

    if (bytesWritten != payloadSize + I2C_FRAME_HEADER_SIZE) {
      Wire.endTransmission();
      finalStatus = I2C_STATUS_SHORT_WRITE;
      break;
    }

    finalStatus = Wire.endTransmission();
    if (finalStatus != I2C_STATUS_SUCCESS) {
      break;
    }
  }

  telemetry.lastI2CBytesWritten =
      totalBytesWritten > UINT8_MAX ? UINT8_MAX : totalBytesWritten;
  telemetry.lastI2CStatus = finalStatus;

  digitalWrite(LED_PIN, LOW);
  return finalStatus;
}

void saveGPSData(UBX_NAV_PVT_data_t *ubxDataStruct) {
  (void)ubxDataStruct;

  // Pulse the builtin LED if we get a callback from the GPS
  digitalWrite(BUILTIN_LED_PIN, HIGH);

  telemetry.gps.latitude = myGNSS.getLatitude(0);
  telemetry.gps.longitude = myGNSS.getLongitude(0);
  telemetry.gps.altitude = myGNSS.getAltitude(0);
  telemetry.gps.nedNorthVel = myGNSS.getNedNorthVel(0);
  telemetry.gps.nedDownVel = myGNSS.getNedDownVel(0);
  telemetry.gps.nedEastVel = myGNSS.getNedEastVel(0);

  // Unix: valid only when PVT marks UTC date+time valid and epoch is in range.
  const bool dateTimeOk =
      myGNSS.getDateValid(0) && myGNSS.getTimeValid(0);
  uint32_t epoch = 0;
  if (dateTimeOk) {
    epoch = myGNSS.getUnixEpoch(0);
  }
  telemetry.gps.unixEpoch = epoch;

  time_of_last_gps_update = millis();

  constexpr uint32_t kMinPlausibleUnix = 946684800UL;   // 2000-01-01 UTC
  constexpr uint32_t kMaxPlausibleUnix = 4102444800UL;  // ~2099
  const bool epochOk = dateTimeOk && epoch >= kMinPlausibleUnix &&
                       epoch <= kMaxPlausibleUnix;
  setValidityFlag(VALIDITY_GPS_UNIX_EPOCH, epochOk);

  // Position / nav block: receiver reports useful LLH (independent of unix flag).
  const uint8_t fixType = myGNSS.getFixType(0);
  const bool posOk = myGNSS.getGnssFixOk(0) ||
                     (fixType >= 2 && fixType <= 4);  // 2D/3D/GNSS+DR, not time-only
  setValidityFlag(VALIDITY_GPS, posOk);

  digitalWrite(BUILTIN_LED_PIN, LOW);
}

void serviceGPS(bool force = false) {
  unsigned long now = millis();
  bool pollIntervalElapsed =
      now - time_of_last_gps_poll >= GPS_POLL_INTERVAL_MS;

  if (!force && !gps_service_requested && !pollIntervalElapsed) {
    return;
  }

  gps_service_requested = false;
  time_of_last_gps_poll = now;

  // checkUblox reads any waiting bytes. checkCallbacks dispatches AutoPVT
  // callbacks. getPVT(0) is a non-blocking fallback for boards where the GPS
  // INT line is absent or no callback fires despite a fresh auto-PVT packet.
  myGNSS.checkUblox();
  myGNSS.checkCallbacks();
  if (myGNSS.getPVT(0)) {
    saveGPSData(nullptr);
  }
}

void configureGPSInterrupt() {
#if GPS_INT_PIN >= 0
  pinMode(GPS_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(GPS_INT_PIN), handleGPSInterrupt,
                  FALLING);
#endif
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(PPS_PIN, INPUT_PULLUP);
  pinMode(BUILTIN_LED_PIN, OUTPUT);

  digitalWrite(BUILTIN_LED_PIN, LOW);

  attachInterrupt(digitalPinToInterrupt(PPS_PIN), handleInterrupt, RISING);
  configureGPSInterrupt();

  telemetry.lastI2CStatus = I2C_STATUS_SUCCESS;

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

  myGNSS.setI2COutput(COM_TYPE_UBX);
  myGNSS.setNavigationFrequency(GPS_NAVIGATION_FREQUENCY_HZ);
  myGNSS.setAutoPVT(true, false);
  myGNSS.setAutoPVTcallbackPtr(saveGPSData);
  serviceGPS(true);

  digitalWrite(LED_PIN, LOW);
}

void loop() {
  if (bmp.performReading()) {
    telemetry.bmp.temperature = bmp.temperature;
    telemetry.bmp.pressure = bmp.pressure;
    setValidityFlag(VALIDITY_BMP, true);
  } else {
    setValidityFlag(VALIDITY_BMP, false);
  }

  lis3mdl.read();
  telemetry.magnetometer.x = lis3mdl.x;
  telemetry.magnetometer.y = lis3mdl.y;
  telemetry.magnetometer.z = lis3mdl.z;
  setValidityFlag(VALIDITY_MAGNETOMETER, true);

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
    setValidityFlag(VALIDITY_INERTIAL, true);
  } else {
    setValidityFlag(VALIDITY_INERTIAL, false);
  }

  serviceGPS();
  refreshGPSValidity();

  prepareTelemetryForI2CSend();
  uint8_t statusAbraham =
      sendTelemetryPacket(I2C_ABRAHAM_ADDRESS, I2C_FRAME_DESTINATION_SD);
  uint8_t statusJacob =
      sendTelemetryPacket(I2C_JACOB_ADDRESS, I2C_FRAME_DESTINATION_RADIO);
  telemetry.lastI2CStatus =
      (statusAbraham != I2C_STATUS_SUCCESS) ? statusAbraham : statusJacob;
}

void handleGPSInterrupt(void) { gps_service_requested = true; }

void handleInterrupt(void) {
  time_of_last_pps = millis();
  can_blink = 1;
  digitalWrite(LED_PIN, LOW);
}
