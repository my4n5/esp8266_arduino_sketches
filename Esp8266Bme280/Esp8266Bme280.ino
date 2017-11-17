#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <ThingSpeak.h>

#include <ESP8266WiFi.h>

// Wire Settings
Adafruit_BMP280 bme; // I2C
const float seaLevelPressureConst = 1016.38;
#define PA_MMHG (101325.0/760.0)
#define SDA 0 //gpio0 d3
#define SCL 2 //gpio2 d4

// Wi-Fi Settings
const char* ssid = "*"; // HERE Wi-Fi SSID
const char* password = "*"; // HERE Wi-Fi password

WiFiClient client;

// ThingSpeak Settings
unsigned long channelID = *; //HERE ThingSpeak channel ID
const char *writeAPIKey = "*"; //HERE ThingSpeak write API key
const int postingInterval = 20 * 1000;

void setup() {
  Serial.begin(115200);

  Wire.begin(SDA, SCL); //SDA,SCL
  if (!bme.begin()) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    ESP.deepSleep(999999999*999999999U, WAKE_NO_RFCAL); //sleep
  }
  Serial.println("BMP280 connected");

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  Serial.println("Wi-Fi connected");

  ThingSpeak.begin(client);
  Serial.println("ThingSpeak connected");
}

float readSeaLevelPressureGPa(){
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

  Serial.println();
}

void loop() {
  float temperatureCelsius, pressureMmHg, altitudeMeters;
  float seaLevelPressureGPA = readSeaLevelPressureGPa();
  readBMP280Value(seaLevelPressureGPA, temperatureCelsius, pressureMmHg, altitudeMeters);

  delay(postingInterval);
return ;
  ThingSpeak.setField(1, temperatureCelsius);
  ThingSpeak.setField(2, pressureMmHg);
  ThingSpeak.setField(3, altitudeMeters);
  ThingSpeak.writeFields(channelID, writeAPIKey);

  delay(postingInterval);
}
