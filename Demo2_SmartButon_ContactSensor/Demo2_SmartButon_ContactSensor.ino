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
// https://github.com/adafruit/Adafruit_NeoPixel
//
// This Matter Application will create 2 endpoints: Smart Button + Contact Sensor
// It uses M5Stack Mechanical Key that has 1 WS2812 LED to light the key
// Contact Sensor is a simple Magnetic Reed Switch
// WiFi will connect using hardcoded WiFi credential (defined in this code)
// In case that also fails, it will turn the Builtin LED RED
// After sucess WiFi connection, the sketch will wait for Matter Provisioning
// Once it is done, the Matter endpoints will start to be visible in the Matter Controller or APP
//
// on board RGB LED will show WiFi/Matter Status:
// OFF -- Not Started
// Binking BLUE -- Processing WiFi connection
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

const uint8_t keyPin = 2;    // M5Stack grove IN  pin = 1, 2, 4 and 7
const uint8_t LEDkeyPin = 3; // M5Stack grove OUT pin = 0, 3, 5 and 6
const uint8_t reedPin = 4;   // M5Stack grove IN  pin = 1, 2, 4 and 7

#include <Matter.h>
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>

// List of Matter Endpoints for this Node
// Generic Switch Endpoint - works as a smart button with a single click
MatterGenericSwitch SmartButton;
// Matter Contact Sensor Endpoint
MatterContactSensor ContactSensor;

// This will control the RGB Key backlight
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(1, LEDkeyPin, NEO_GRB + NEO_KHZ800);

void monitorMagneticReed() {
  static bool pinState = false;
  static uint32_t timeStamp = 0;
  const uint32_t debouceTime = 500;

  uint32_t time_diff = millis() - timeStamp;

  if (digitalRead(reedPin) == LOW && time_diff > debouceTime && !pinState) {
    // deals with button debouncing
    timeStamp = millis();  // record the time that the Reed changed state.
    pinState = true;       // Contact CLOSED
    // CLOSED means that the magnet is close enough to make both reed ends connected
    log_i("Magnetic Reed is CLOSED. Sending event to the Matter Controller!");
    ContactSensor.setContact(true);
  }

  if (pinState && time_diff > debouceTime && digitalRead(reedPin) == HIGH) {
    timeStamp = millis();  // record the time that the Reed changed state.
    pinState = false;  // Contact OPEN
    // OPEN means that the magnet is far enough to make both reed ends disconnected
    log_i("Magnetic Reed is OPEN. Sending event to the Matter Controller!");
    ContactSensor.setContact(false);
  }
}

void monitorMechanicalKeyState() {
  static bool pinState = false;
  static uint32_t timeStamp = 0;
  const uint32_t debouceTime = 200;

  // Check if the button has been pressed
  if (digitalRead(keyPin) == LOW && !pinState) {
    // deals with button debouncing
    timeStamp = millis();  // record the time while the key is pressed.
    pinState = true;    // pressed.
    // Key backlight goes green
    pixels.setPixelColor(0, pixels.Color(0, 64, 0));
    pixels.show();
  }

  uint32_t time_diff = millis() - timeStamp;
  if (pinState && time_diff > debouceTime && digitalRead(keyPin) == HIGH) {
    pinState = false;  // released
    log_i("Key released. Sending Click to the Matter Controller!");
    // Matter Controller will receive an event and, if programmed, it will trigger an action
    SmartButton.click();
    // Key backlight off
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    pixels.show();
  }
}


void setup() {
  // Initialize the USER BUTTON (Boot button) that will be used to decommission the Matter Node
  pinMode(buttonPin, INPUT_PULLUP);
  // Initialize the Key and Reed pins
  pinMode(keyPin, INPUT_PULLUP);
  pinMode(reedPin, INPUT_PULLUP);
  // set Key backlight to off
  pixels.begin();    // Init the NeoPixel library
  pixels.setPixelColor(0, pixels.Color(0, 0, 0));
  pixels.show();
  // Initialize the Status RGB LED
  pinMode(statusLedPin, OUTPUT);
  rgbLedWrite(statusLedPin, 0, 0, 0); // OFF
  Serial.begin(115200);

  rgbLedWrite(statusLedPin, 0, 0, 64); // Blinking Blue - connecting
  Serial.print("Trying to connect to WiFi SSID [");
  Serial.print(ssid);
  Serial.println("]");
  uint16_t timeout = 2 * 15 * 1; // 15 seconds
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

  // Initialize the Matter EndPoint
  SmartButton.begin();
  ContactSensor.begin();

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
    // waits for Matter Temperature Sensor Commissioning.
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
  // monitor Key and Magnetic Reed states
  monitorMagneticReed();
  monitorMechanicalKeyState();

  // Check if the board button (decommissioning) has been pressed
  if (digitalRead(buttonPin) == LOW && !button_state) {
    // deals with button debouncing
    button_time_stamp = millis();  // record the time while the button is pressed.
    button_state = true;           // pressed.
  }

  if (button_state && digitalRead(buttonPin) == HIGH) {
    button_state = false;  // released
  }

  // Onboard User Button is kept pressed for longer than 5 seconds in order to decommission matter node
  uint32_t time_diff = millis() - button_time_stamp;
  if (button_state && time_diff > decommissioningTimeout) {
    Serial.println("Decommissioning the Generic Switch Matter Accessory. It shall be commissioned again.");
    Matter.decommission();
    button_time_stamp = millis();  // avoid running decommissining again, reboot takes a second or so
  }

  delay(50);
}
