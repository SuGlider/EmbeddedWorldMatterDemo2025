// Copyright 2025 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// This Matter Application will create 1 endpoint: White Dimmable Light
// It uses a device with 1 White LED - M5Stack FlashLight
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

// PWM Controlled LED pin from M5Stack FlashLight
const uint8_t whiteLEDPin = 2; // 0, 2, 4, 6 (M5Stack IN Pin)

#include <Matter.h>
#include <WiFi.h>
#include <Preferences.h>

// List of Matter Endpoints for this Node
// Dimmable Light Endpoint
MatterDimmableLight DimmableLight;

// it will keep last OnOff & Brightness state stored, using Preferences
Preferences matterPref;
const char *onOffPrefKey = "OnOff";
const char *brightnessPrefKey = "Brightness";

// Set the Flash Light based on the current state of the Dimmable Light
bool setLightState(bool state, uint8_t brightness) {
  if (state) {
    // set pin ad LEDC
    analogWrite(whiteLEDPin, brightness);
  } else {
    pinMode(whiteLEDPin, OUTPUT);
    digitalWrite(whiteLEDPin, LOW);
  }
  // store last Brightness and OnOff state for when the Light is restarted / power goes off
  matterPref.putUChar(brightnessPrefKey, brightness);
  matterPref.putBool(onOffPrefKey, state);
  // This callback must return the success state to Matter core
  return true;
}

void setup() {
  // Initialize the USER BUTTON (Boot button) that will be used to decommission the Matter Node
  pinMode(buttonPin, INPUT_PULLUP);
  // Initialize the Status RGB LED
  pinMode(statusLedPin, OUTPUT);
  rgbLedWrite(statusLedPin, 0, 0, 0); // OFF
  // Configure the FlashLight pin (digital/PWM output)
  pinMode(whiteLEDPin, OUTPUT);
  digitalWrite(whiteLEDPin, LOW); // Off
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

  // Initialize Matter EndPoint
  matterPref.begin("MatterPrefs", false);
  // default OnOff state is ON if not stored before
  bool lastOnOffState = matterPref.getBool(onOffPrefKey, true);
  // default brightness ~= 6% (15/255)
  uint8_t lastBrightness = matterPref.getUChar(brightnessPrefKey, 15);
  DimmableLight.begin(lastOnOffState, lastBrightness);
  // set the callback function to handle the Light state change
  DimmableLight.onChange(setLightState);

  // lambda functions are used to set the attribute change callbacks
  DimmableLight.onChangeOnOff([](bool state) {
    Serial.printf("Light OnOff changed to %s\r\n", state ? "ON" : "OFF");
    return true;
  });
  DimmableLight.onChangeBrightness([](uint8_t level) {
    Serial.printf("Light Brightness changed to %d\r\n", level);
    return true;
  });

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
  DimmableLight.updateAccessory();
}

void loop() {
  // Check if the board button (decommissioning) has been pressed
  if (digitalRead(buttonPin) == LOW && !button_state) {
    // deals with button debouncing
    button_time_stamp = millis();  // record the time while the button is pressed.
    button_state = true;           // pressed.
  }

  if (digitalRead(buttonPin) == HIGH && button_state) {
    button_state = false;    // released
  }

  // Onboard User Button is kept pressed for longer than 5 seconds in order to decommission matter node
  uint32_t time_diff = millis() - button_time_stamp;
  if (button_state && time_diff > decommissioningTimeout) {
    Serial.println("Decommissioning this Matter Accessory. It shall be commissioned again.");
    DimmableLight = false;  // turn the light off
    Matter.decommission();
    button_time_stamp = millis();  // avoid running decommissining again, reboot takes a second or so
  }
}
