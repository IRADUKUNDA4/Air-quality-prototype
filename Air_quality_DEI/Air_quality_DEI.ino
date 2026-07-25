#include <Arduino.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_BME280.h>
#include <HardwareSerial.h>

#define GSM_RX_PIN 17
#define GSM_TX_PIN 18
HardwareSerial gsmSerial(2);

const char* GPRS_APN = "internet";
const char* GPRS_USER = "";
const char* GPRS_PASS = "";

const char* API_URL = "https://infertility-monitor-backend.vercel.app/api/readings";
const char* DEVICE_CODE = "ESP32-AIRQ-001";
const char* PATIENT_ID = "751c0190-02dd-4b20-866a-92f2f38d9bbd";

unsigned long lastGsmSendTime = 0;
const unsigned long GSM_SEND_INTERVAL = 30000;
bool gprsReady = false;

#define OLED_SDA_PIN 8
#define OLED_SCL_PIN 9

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_I2C_ADDR 0x3C
Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define BME280_I2C_ADDR 0x76
Adafruit_BME280 bme;
bool bmeFound = false;

HardwareSerial pmsSerial(1);
#define PMS_RX_PIN 10
#define PMS_TX_PIN 11

HardwareSerial gpsSerial(2);
#define GPS_RX_PIN 4
#define GPS_TX_PIN 5
TinyGPSPlus gps;

#define PIN_CJMCU_CO 1
#define PIN_CJMCU_NH3 3
#define PIN_CJMCU_NO2 2
#define PIN_SO2 6
#define PIN_CO2 7

#define ADC_MAX_VAL 4095.0
#define ADC_REF_VOLTAGE 3.3

#define SO2_MAX_PPM 20.0
#define CO2_MAX_PPM 5000.0

#define CALIBRATION_SAMPLES 100
#define CALIBRATION_SAMPLE_DELAY_MS 20

float R0_CO = 1.0;
float R0_NH3 = 1.0;
float R0_NO2 = 1.0;

struct BMEData {
  float tempC;
  float humidity;
  float pressureHPa;
};
BMEData bmeData;

struct PMSData {
  uint16_t pm10_env;
  uint16_t pm25_env;
  uint16_t pm100_env;
};
PMSData pmsData;

struct GasData {
  float ppb_CO;
  float ppb_NH3;
  float ppb_NO2;
  float ppb_SO2;
  float ppb_CO2;
};
GasData gasData;

unsigned long lastPageSwitch = 0;
const unsigned long PAGE_INTERVAL = 3000;
const uint8_t PAGE_COUNT = 3;
uint8_t currentPage = 0;

bool readPMSdata(Stream *s);
void readGasSensors();
void readBME280();
float calculateResist(int rawAdc);
float readAverageResist(int pin, uint16_t samples);
float ppmToPpb(float ppm);
void calibrateBaseline();
String sendATCommand(const String &cmd, unsigned long timeoutMs);
bool gsmInitGPRS();
int computeAQI(uint16_t pm25);
String aqiCategory(int aqi);
bool sendReadingGSM();
void updateOLED();
void renderPage1();
void renderPage2();
void renderPage3();

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);

  if (!display.begin(OLED_I2C_ADDR, true)) {
    Serial.println(F("OLED initialization failed!"));
    for (;;)
      ;
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 10);
  display.println(F("    AIR QUALITY    "));
  display.println(F("  TELEMETRY SYSTEM  "));
  display.setCursor(0, 40);
  display.println(F("Initializing..."));
  display.display();

  if (bme.begin(BME280_I2C_ADDR, &Wire)) {
    bmeFound = true;
    Serial.println(F("BME280 Sensor detected."));
  } else {
    Serial.println(F("Warning: BME280 not found at 0x76! Trying 0x77..."));
    if (bme.begin(0x77, &Wire)) {
      bmeFound = true;
      Serial.println(F("BME280 Sensor detected at 0x77."));
    }
  }

  pmsSerial.begin(9600, SERIAL_8N1, PMS_RX_PIN, PMS_TX_PIN);
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  analogReadResolution(12);

  display.clearDisplay();
  display.setCursor(0, 10);
  display.println(F("Calibrating gas"));
  display.println(F("sensor baseline..."));
  display.println(F("Keep in clean air"));
  display.display();

  calibrateBaseline();

  display.clearDisplay();
  display.setCursor(0, 10);
  display.println(F("Connecting GSM..."));
  display.display();

  gsmSerial.begin(9600, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN);
  delay(3000);
  gprsReady = gsmInitGPRS();

  Serial.println(gprsReady ? F("[GSM] GPRS bearer ready.") : F("[GSM] GPRS setup FAILED - will retry per send cycle."));

  delay(1000);
  Serial.println(F("System Initialized Successfully."));
}

void loop() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  if (readPMSdata(&pmsSerial)) {
    readGasSensors();
    readBME280();
  }

  if (millis() - lastPageSwitch >= PAGE_INTERVAL) {
    lastPageSwitch = millis();
    currentPage = (currentPage + 1) % PAGE_COUNT;
  }

  updateOLED();

  if (millis() - lastGsmSendTime >= GSM_SEND_INTERVAL) {
    lastGsmSendTime = millis();
    bool ok = sendReadingGSM();
    Serial.println(ok ? F("[GSM] Reading sent OK.") : F("[GSM] Send FAILED."));
  }
}

float readAverageResist(int pin, uint16_t samples) {
  float sum = 0;
  for (uint16_t i = 0; i < samples; i++) {
    sum += calculateResist(analogRead(pin));
    delay(CALIBRATION_SAMPLE_DELAY_MS);
  }
  return sum / samples;
}

void calibrateBaseline() {
  R0_CO = readAverageResist(PIN_CJMCU_CO, CALIBRATION_SAMPLES);
  R0_NH3 = readAverageResist(PIN_CJMCU_NH3, CALIBRATION_SAMPLES);
  R0_NO2 = readAverageResist(PIN_CJMCU_NO2, CALIBRATION_SAMPLES);

  if (R0_CO <= 0) R0_CO = 1.0;
  if (R0_NH3 <= 0) R0_NH3 = 1.0;
  if (R0_NO2 <= 0) R0_NO2 = 1.0;

  Serial.printf("Baseline R0 -> CO: %.3f  NH3: %.3f  NO2: %.3f\n", R0_CO, R0_NH3, R0_NO2);
}

float ppmToPpb(float ppm) {
  if (ppm <= 0) return 0.0;
  return ppm * 1000.0;
}

float calculateResist(int rawAdc) {
  if (rawAdc <= 0) return 1.0;
  float vOut = rawAdc * (ADC_REF_VOLTAGE / ADC_MAX_VAL);
  return (ADC_REF_VOLTAGE - vOut) / vOut;
}

void readBME280() {
  if (bmeFound) {
    bmeData.tempC = bme.readTemperature();
    bmeData.humidity = bme.readHumidity();
    bmeData.pressureHPa = bme.readPressure() / 100.0F;
  }
}

void readGasSensors() {
  int rawCO = analogRead(PIN_CJMCU_CO);
  int rawNH3 = analogRead(PIN_CJMCU_NH3);
  int rawNO2 = analogRead(PIN_CJMCU_NO2);
  int rawSO2 = analogRead(PIN_SO2);
  int rawCO2 = analogRead(PIN_CO2);

  float ratioCO = calculateResist(rawCO) / R0_CO;
  float ratioNH3 = calculateResist(rawNH3) / R0_NH3;
  float ratioNO2 = calculateResist(rawNO2) / R0_NO2;

  float ppmCO = 4.69 * pow(ratioCO, -1.18);
  float ppmNH3 = 0.81 * pow(ratioNH3, -2.75);
  float ppmNO2 = 0.15 * pow(ratioNO2, 1.02);

  float voltSO2 = rawSO2 * (ADC_REF_VOLTAGE / ADC_MAX_VAL);
  float voltCO2 = rawCO2 * (ADC_REF_VOLTAGE / ADC_MAX_VAL);

  float ppmSO2 = (voltSO2 / ADC_REF_VOLTAGE) * SO2_MAX_PPM;
  float ppmCO2 = (voltCO2 / ADC_REF_VOLTAGE) * CO2_MAX_PPM;

  gasData.ppb_CO = ppmToPpb(ppmCO);
  gasData.ppb_NH3 = ppmToPpb(ppmNH3);
  gasData.ppb_NO2 = ppmToPpb(ppmNO2);
  gasData.ppb_SO2 = ppmToPpb(ppmSO2);
  gasData.ppb_CO2 = ppmToPpb(ppmCO2);
}

void updateOLED() {
  display.clearDisplay();

  switch (currentPage) {
    case 0:
      renderPage1();
      break;
    case 1:
      renderPage2();
      break;
    case 2:
      renderPage3();
      break;
  }

  display.display();
}

void renderPage1() {
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.println(F("[1/3] ENV & DUST"));
  display.drawLine(0, 9, 128, 9, SH110X_WHITE);

  display.setCursor(0, 14);
  if (bmeFound) {
    display.printf("Temp : %.1f C\n", bmeData.tempC);
    display.printf("Hum  : %.1f %%\n", bmeData.humidity);
    display.printf("Press: %.0f hPa\n", bmeData.pressureHPa);
  } else {
    display.println(F("BME280: Not Found"));
  }

  display.printf("PM2.5: %d ug/m3\n", pmsData.pm25_env);
  display.printf("PM10 : %d ug/m3\n", pmsData.pm100_env);
}

void renderPage2() {
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.println(F("[2/3] GASES (ppb)"));
  display.drawLine(0, 9, 128, 9, SH110X_WHITE);

  display.setCursor(0, 12);
  display.printf("CO  : %8.1f\n", gasData.ppb_CO);
  display.setCursor(0, 21);
  display.printf("NH3 : %8.1f\n", gasData.ppb_NH3);
  display.setCursor(0, 30);
  display.printf("NO2 : %8.1f\n", gasData.ppb_NO2);
  display.setCursor(0, 39);
  display.printf("SO2 : %8.1f\n", gasData.ppb_SO2);
  display.setCursor(0, 48);
  display.printf("CO2 : %8.0f\n", gasData.ppb_CO2);
}

void renderPage3() {
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.println(F("[3/3] GPS TELEMETRY"));
  display.drawLine(0, 9, 128, 9, SH110X_WHITE);

  display.setCursor(0, 14);
  if (gps.location.isValid()) {
    display.printf("Lat: %.5f\n", gps.location.lat());
    display.printf("Lon: %.5f\n", gps.location.lng());
    display.printf("Alt: %.1f m\n", gps.altitude.meters());
    display.printf("Sats: %d", gps.satellites.value());
  } else {
    display.setCursor(0, 24);
    display.println(F(" Searching for "));
    display.println(F(" GPS Satellites..."));
  }
}

bool readPMSdata(Stream *s) {
  if (!s->available()) return false;
  if (s->peek() != 0x42) {
    s->read();
    return false;
  }
  if (s->available() < 32) return false;

  uint8_t buffer[32];
  uint16_t sum = 0;
  s->readBytes(buffer, 32);

  for (uint8_t i = 0; i < 30; i++) sum += buffer[i];
  uint16_t checksum = ((uint16_t)buffer[30] << 8) | buffer[31];

  if (sum != checksum) return false;

  pmsData.pm10_env = ((uint16_t)buffer[10] << 8) | buffer[11];
  pmsData.pm25_env = ((uint16_t)buffer[12] << 8) | buffer[13];
  pmsData.pm100_env = ((uint16_t)buffer[14] << 8) | buffer[15];

  return true;
}