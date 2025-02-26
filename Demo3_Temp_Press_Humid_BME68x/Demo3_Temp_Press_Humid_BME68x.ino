// Copyright 2025 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Dependencies:
// https://github.com/adafruit/Adafruit_Sensor
// https://github.com/adafruit/Adafruit_BME680

// This Matter Application will create 3 endpoints: Temperature, Humidity and Pressure
// It uses a BME680 or BME688 sensor with I2C (M5Stack ENV PRO)
// WiFi will connect using hardcoded WiFi credential (defined in this code)
// In case that also fails, it will turn the Builtin LED RED
// After sucess WiFi connection, the sketch will wait for Matter Provisioning
// Once it is done, the Matter endpoints will start to be visible in the Matter Controller or APP
//
// on board RGB LED will show WiFi/Matter Status:
// OFF -- Not Started
// Blinking RED -- BME680 has failed to start
// Binking BLUE -- BME680 OK + Processing WiFi connection
// Solid RED -- Failed Connecting to WiFi
// Solid BLUE -- WiFi is connected
// Blinking Green -- Waiting for commisssioning
// Solid Green -- Commissioned and Ready to work

// Arduino ESP32 Matter Library v3.1.3 or lower requires
// the WiFi credentials to be defined in the code
// WiFi is manually set and started - for the C5 you may select a 5GHz WiFi SSID
const char *ssid = "your-ssid";          // Change this to your WiFi SSID
const char *password = "your-password";  // Change this to your WiFi password

// set your board USER BUTTON pin here - decommissioning button
const uint8_t buttonPin = BOOT_PIN;  // Set your pin here. Using BOOT Button.

// set your board RGB LED pin here - displays WiFi/Matter Status
#ifdef RGB_BUILTIN
const uint8_t statusLedPin = RGB_BUILTIN;
#else
const uint8_t statusLedPin = 2;  // Set your pin here if your board has not defined LED_BUILTIN
#warning "Do not forget to set the Matter Status RGB LED pin"
#endif

// decommissioning Board Button control
uint32_t button_time_stamp = 0;                // debouncing control
bool button_state = false;                     // false = released | true = pressed
const uint32_t decommissioningTimeout = 5000;  // keep the button pressed for 5s, or longer, to decommission

#include <Matter.h>
#include <WiFi.h>
#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BME680.h"

// BME680 Sensor
// default I2C pins communication
Adafruit_BME680 bme;

// List of Matter Endpoints for this Node
// Matter Temperature Sensor Endpoint
MatterTemperatureSensor TemperatureSensor;
// Matter Pressure Sensor Endpoint
MatterPressureSensor PressureSensor;
// Matter Humidity Sensor Endpoint
MatterHumiditySensor HumiditySensor;

// BME68x temperature, humidity and pressure sensor
bool updateTempHumidityPressure() {
  float bm_t = NAN;
  float bm_h = NAN;
  float bm_p = NAN;
  bool retCode = false;

  if (bme.performReading()) {
    bm_t = bme.temperature;
    bm_h = bme.humidity;
    bm_p = bme.pressure;
  }
  if (!isnan(bm_t) && !isnan(bm_h) && !isnan(bm_p)) {
    Serial.print("BME680  Temp: "); Serial.print(bm_t, 2); Serial.print(" °C"); Serial.print(" Hum: "); Serial.print(bm_h); Serial.print("%"); Serial.print(" Pressure: "); Serial.print(bm_p / 100.0); Serial.println("hPa");
    // Set Matter internal values
    TemperatureSensor.setTemperature(bm_t);
    HumiditySensor.setHumidity(bm_h);
    PressureSensor.setPressure(bm_p / 100.0);
    retCode = true;
  }
  return retCode;
}

void setup() {
  // Initialize the USER BUTTON (Boot button) that will be used to decommission the Matter Node
  pinMode(buttonPin, INPUT_PULLUP);
  // Status RGB LED initially Off
  pinMode(statusLedPin, OUTPUT);
  rgbLedWrite(statusLedPin, 0, 0, 0); // OFF
  Serial.begin(115200);

  // default Wire pins and I2C Addr 0x77
  if (!bme.begin()) {
    while (1) {
      rgbLedWrite(statusLedPin, 0, 0, 0); // OFF
      delay(500);
      rgbLedWrite(statusLedPin, 64, 0, 0); // RED
      delay(500);
      Serial.println("Could not find a valid BME680 sensor, check wiring!");
    }
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_1X);
  bme.setHumidityOversampling(BME680_OS_1X);
  bme.setPressureOversampling(BME680_OS_1X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_0);
  bme.setGasHeater(0, 0); // Not used
  // perform initiam BME680 reading
  float bm_t = NAN;
  float bm_h = NAN;
  float bm_p = NAN;
  while (!bme.performReading()) {
    delay(100);
  }
  bm_t = bme.temperature;
  bm_h = bme.humidity;
  bm_p = bme.pressure;
  if (!isnan(bm_t) && !isnan(bm_h) && !isnan(bm_p)) {
    Serial.print("BME680  Temp: "); Serial.print(bm_t, 2); Serial.print(" °C"); Serial.print(" Hum: "); Serial.print(bm_h); Serial.print("%"); Serial.print(" Pressure: "); Serial.print(bm_p / 100.0); Serial.println("hPa");
  }

  // Manually connect to WiFi
  rgbLedWrite(statusLedPin, 0, 0, 64); // Blinking Blue - connecting
  Serial.print("Trying to connect to WiFi SSID [");
  Serial.print(ssid);
  Serial.println("]");
  uint16_t timeout = 2 * 15; // 500ms * 2 * 15 = 15 seconds
  WiFi.begin(ssid, password);
  while (timeout && WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (!(--timeout % 30)) {
      Serial.println();
    }
    if (timeout & 1) {
      rgbLedWrite(statusLedPin, 0, 0, 0); // OFF
    } else {
      rgbLedWrite(statusLedPin, 0, 0, 64); // BLUE
    }
  }
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.STA.end();
    rgbLedWrite(statusLedPin, 64, 0, 0); // SOLID RED
    while (1) {
      Serial.println("Failed to connect to WiFi. Check WiFi Credentials.");
      delay(2000);
    }
  }
  Serial.println();
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  // Solid Blue - WiFi connected
  rgbLedWrite(statusLedPin, 0, 0, 64);
  delay(500);

  // start all sensor endpoints with initial readings
  TemperatureSensor.begin(bm_t);
  HumiditySensor.begin(bm_h);
  PressureSensor.begin(bm_p / 100.0);

  // Matter beginning - Last step, after all EndPoints are initialized
  Matter.begin();
  
  // Check Matter Accessory Commissioning state, which may change during execution of loop()
  if (!Matter.isDeviceCommissioned()) {
    Serial.println("");
    Serial.println("Matter Node is not commissioned yet.");
    Serial.println("Initiate the device discovery in your Matter environment.");
    Serial.println("Commission it to your Matter hub with the manual pairing code or QR code");
    Serial.printf("Manual pairing code: %s\r\n", Matter.getManualPairingCode().c_str());
    Serial.printf("QR code URL: %s\r\n", Matter.getOnboardingQRCodeUrl().c_str());
    // waits for Matter Commissioning.
    uint32_t timeCount = 0;
    while (!Matter.isDeviceCommissioned()) {
      delay(500);
      // Blinking Green - Waiting for Matter
      if (timeCount & 1) {
        rgbLedWrite(statusLedPin, 0, 0, 0);
      } else {
        rgbLedWrite(statusLedPin, 0, 64, 0);
      }
      if ((timeCount++ % 10) == 0) {  // 10*500ms = 5 sec
        Serial.println("Matter Node not commissioned yet. Waiting for commissioning.");
      }
    }
    Serial.println("Matter Node is commissioned and connected to Wi-Fi. Ready for use.");
  }
  rgbLedWrite(statusLedPin, 0, 64, 0); // Solid Green - Matter Commissioned
}

void loop() {
  static uint32_t timeCounter = 0;

  // Print the current temperature value every 5s
  if (!(timeCounter++ % 10)) {  // delaying for 500ms x 10 = 5s
    // Update Temperature, Humidity and Pressure from BME68x
    // Matter APP shall display the updated information
    updateTempHumidityPressure();

    log_i("Current Temperature is %.02fC\r\n", TemperatureSensor.getTemperature());
    log_i("Current Humidity is %.02f%%\r\n", HumiditySensor.getHumidity());
    log_i("Current Pressure is %.02fhPa\r\n", PressureSensor.getPressure());
  }

  // Check if the board button (decommissioning) has been pressed
  if (digitalRead(buttonPin) == LOW && !button_state) {
    // deals with button debouncing
    button_time_stamp = millis();  // record the time while the button is pressed.
    button_state = true;           // pressed.
  }

  if (digitalRead(buttonPin) == HIGH && button_state) {
    button_state = false;  // released
  }

  // Onboard User Button is kept pressed for longer than 5 seconds in order to decommission matter node
  uint32_t time_diff = millis() - button_time_stamp;
  if (button_state && time_diff > decommissioningTimeout) {
    Serial.println("Decommissioning this Matter Accessory. It shall be commissioned again.");
    Matter.decommission();
    button_time_stamp = millis();  // avoid running decommissining again, reboot takes a second or so
  }

  delay(500);
}
