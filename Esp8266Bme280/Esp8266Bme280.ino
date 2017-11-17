#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <ThingSpeak.h>

#include <OneWire.h>
#include <DallasTemperature.h>

#include <ESP8266WiFi.h>
/*
   #define MY_WIFI_SSID "WIFI_SSID"
   #define MY_WIFI_PASSWORD "WIFI_PASS"

   #define MY_THING_SPEAK_CHANNEL_ID 99999ul
   #define MY_THING_SPEAK_WRITE_API_KEY "012345678abcdef"
*/
#include "user_definitions.h"

// Wire Settings
/*
   Bosh BMP280 Datasheet https://cdn-shop.adafruit.com/datasheets/BST-BMP280-DS001-11.pdf
   The 7-bit device address is 111011x. The 6 MSB bits are fixed. The last bit is changeable by
   SDO value and can be changed during operation. Connecting SDO to GND results in slave
   address 1110110 (0x76); connection it to VDDIO results in slave address 1110111 (0x77),
   which s the same as BMP180’s I²C address. The SDO pin cannot be left floating; if left floating, the
   I²C address will be undefined.

   Pin connections ESP8266 -> BMP280
   GPIO0 (D3) -> SDA (SDI)
   GPIO2 (D4)  -> SCL (SCK)
   3V -> VCC (VDDIO)
   G -> GND
   SD0 -> VCC (VDDIO; for slave address '1' - 0x77)
   CSB -> VCC (VDDIO) or GND [then poweroff-poweron esp8633]
*/
Adafruit_BMP280 bme; // I2C
const float seaLevelPressureConst = 1016.38;
#define PA_MMHG (101325.0/760.0)
#define SDA 0 //GPIO0 D3
#define SCL 2 //GPIO2 D4

//DALLAS DS18B20 - Temp senspr Settings
#define ONE_WIRE_BUS D2
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress insideThermometer;
bool dallasInitialized;

//Notify Led settings
#define LED_PIN D1

// Wi-Fi Settings
const char* ssid = MY_WIFI_SSID;
const char* password = MY_WIFI_PASSWORD;

WiFiClient client;

// ThingSpeak Settings
unsigned long channelID = MY_THING_SPEAK_CHANNEL_ID;
const char *writeAPIKey = MY_THING_SPEAK_WRITE_API_KEY;
const int postingInterval = 20 * 1000;

void setup() {
  Serial.begin(115200);
  delay(10);

  Serial.println("HELLO!");

  Wire.begin(SDA, SCL);

  if (!bme.begin()) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    ESP.deepSleep(999999999 * 999999999U, WAKE_NO_RFCAL); //deep sleep
  }
  Serial.println("BMP280 connected");

  sensors.begin();
  byte dallasDeviceCount = sensors.getDeviceCount();
  if (dallasDeviceCount == 0) {
    Serial.println("Dallas devices: absent");
  } else {
    Serial.println(dallasDeviceCount, DEC);
    if (sensors.getAddress(insideThermometer, 0)) {
      sensors.setResolution(insideThermometer, 9);

      dallasInitialized = true;

      Serial.print("Dallas device 0 connected. Address: ");
      for (byte i = 0; i < 8; i++)
      {
        if (insideThermometer[i] < 16) {
          Serial.print("0");
        }
        Serial.print(insideThermometer[i], HEX);
      }
      Serial.println();
    } else {
      Serial.println("Dallas devices: unable to find address for Device 0");
    }
  }

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  Serial.println("Wi-Fi connected");

  ThingSpeak.begin(client);
  Serial.println("ThingSpeak connected");

  pinMode(LED_PIN, OUTPUT);
}

float readSeaLevelPressureGPa() {
  return seaLevelPressureConst;
}

void readBMP280Value(float seaLevelPressureGPA, float &temperatureCelsius, float &pressureMmHg, float &altitudeMeters) {
  temperatureCelsius = bme.readTemperature();
  float pressurePa = bme.readPressure();
  pressureMmHg = pressurePa / PA_MMHG;
  altitudeMeters = bme.readAltitude(seaLevelPressureGPA);

  Serial.print("Temperature = ");
  Serial.print(temperatureCelsius);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(pressureMmHg);
  Serial.println(" mm Hg");

  Serial.print("Approx altitude = ");
  Serial.print(altitudeMeters);
  Serial.println(" m");
}

void readDallasDeviceTemp(float &temperatureCelsius) {
  sensors.requestTemperatures();
  temperatureCelsius = sensors.getTempC(insideThermometer);

  Serial.print("Dallas device temperature = ");
  Serial.print(temperatureCelsius);
  Serial.println(" *C");
}

void loop() {
  digitalWrite(LED_PIN, HIGH);
  Serial.println();

  float temperatureCelsius, pressureMmHg, altitudeMeters, dallasTemperatureCelsius;
  float seaLevelPressureGPA = readSeaLevelPressureGPa();

  readBMP280Value(seaLevelPressureGPA, temperatureCelsius, pressureMmHg, altitudeMeters);

  if (dallasInitialized) {
    readDallasDeviceTemp(dallasTemperatureCelsius);
  }

  digitalWrite(LED_PIN, LOW);

  //  delay(postingInterval);
  //return;
  ThingSpeak.setField(1, temperatureCelsius);
  ThingSpeak.setField(2, pressureMmHg);
  ThingSpeak.setField(3, altitudeMeters);
  ThingSpeak.setField(4, dallasTemperatureCelsius);
  ThingSpeak.writeFields(channelID, writeAPIKey);

  delay(postingInterval);
}
