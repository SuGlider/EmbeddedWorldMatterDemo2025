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
// Dependencies:
// https://github.com/adafruit/Adafruit_NeoPixel
//
// This Matter Application will create 2 endpoints: 
//   Color RGB+CWW Dimmable Light and White Dimmable Light
// It uses a device with 3 WS2812 LEDs (M5Stack RGB LED Unit) got the RGB Light
// and a device with 1 White LED - M5Stack FlashLight for the White Light
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

// set your M5Stack RGB LED Unit Light pin (3 Leds)
#define LIGHT_PIN  (0)   // Use 0, 3, 5, 6 (M5Stack OUT Pin)
#define NUM_PIXELS (3)   // M5Stack RGB_SK_6812 has 3 LEDs

// PWM Controlled LED pin from M5Stack FlashLight
const uint8_t whiteLEDPin = 3; // Use 0, 3, 5, 6 (M5Stack OUT Pin)

#include <Matter.h>
#include <WiFi.h>
#include <Preferences.h>
#include <Adafruit_NeoPixel.h>

// List of Matter Endpoints for this Node
// CWW + Color Light Endpoint
MatterEnhancedColorLight EnhancedColorLight;
// Dimmable Light Endpoint
MatterDimmableLight DimmableLight;

// It will use HSV color to control all Matter Attribute Changes
HsvColor_t currentHSVColor = {0, 0, 0};

// it will keep last OnOff & HSV Color state stored, using Preferences
Preferences matterPref;
const char *onOffColorLightPrefKey = "OnOff";
const char *hsvColorLightPrefKey = "HSV";
const char *onOffWhiteLightPrefKey = "OnOff";
const char *brightnessWhiteLightPrefKey = "Brightness";

// This will control the RGB Light
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUM_PIXELS, LIGHT_PIN, NEO_GRB + NEO_KHZ800);

// Set the Flash Light based on the current state of the Dimmable Light
bool setWhiteLightState(bool state, uint8_t brightness) {
  if (state) {
    // set pin ad LEDC
    analogWrite(whiteLEDPin, brightness);
  } else {
    pinMode(whiteLEDPin, OUTPUT);
    digitalWrite(whiteLEDPin, LOW);
  }
  // store last Brightness and OnOff state for when the Light is restarted / power goes off
  matterPref.putUChar(brightnessWhiteLightPrefKey, brightness);
  matterPref.putBool(onOffWhiteLightPrefKey, state);
  // This callback must return the success state to Matter core
  return true;
}

// Set the RGB LED Light based on the current state of the Enhanced Color Light
bool setColorLightState(bool state, espHsvColor_t colorHSV, uint8_t brighteness, uint16_t temperature_Mireds) {
  log_i("State:%s HSV(%d,%d,%d), Brightness(%d) Temperature(%d)", state ? "ON" : "OFF", colorHSV.h, colorHSV.s, colorHSV.v, brighteness, temperature_Mireds);
  if (state) {
    // currentHSVColor keeps final color result
    espRgbColor_t rgbColor = espHsvColorToRgbColor(currentHSVColor);
    // set the Light Color
    for (uint8_t i = 0; i < NUM_PIXELS; i++) {
      pixels.setPixelColor(i, pixels.Color(rgbColor.r, rgbColor.g, rgbColor.b));
    }
  } else {
    // Turn off
    for (uint8_t i = 0; i < NUM_PIXELS; i++) {
      pixels.setPixelColor(i, pixels.Color(0, 0, 0)); // OFF
    }
  }
  pixels.show();
  // store last HSV Color and OnOff state for when the Light is restarted / power goes off
  matterPref.putBool(onOffColorLightPrefKey, state);
  matterPref.putUInt(hsvColorLightPrefKey, currentHSVColor.h << 16 | currentHSVColor.s << 8 | currentHSVColor.v);
  // This callback must return the success state to Matter core
  return true;
}

void setup() {
  // Initialize the USER BUTTON (Boot button) that will be used to decommission the Matter Node
  pinMode(buttonPin, INPUT_PULLUP);
  // Status RGB LED initially Off
  pinMode(statusLedPin, OUTPUT);
  rgbLedWrite(statusLedPin, 0, 0, 0); // OFF
  // M5Stack RGB Unit initialization
  pixels.begin();    // Init the NeoPixel library
  for (uint8_t i = 0; i < NUM_PIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(0, 0, 0)); // OFF
  }
  pixels.show();
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

  // gets Color RGB CWW Light data stored state in NVS
  // default OnOff state is ON if not stored before
  bool lastColorOnOffState = matterPref.getBool(onOffColorLightPrefKey, true);
  // default HSV color is (21, 216, 25) - Warm White Color at 10% intensity
  uint32_t prefHsvColor = matterPref.getUInt(hsvColorLightPrefKey, 21 << 16 | 216 << 8 | 25);
  currentHSVColor = {uint8_t(prefHsvColor >> 16), uint8_t(prefHsvColor >> 8), uint8_t(prefHsvColor)};
  EnhancedColorLight.begin(lastColorOnOffState, currentHSVColor);
  // set the callback function to handle the Light state change
  EnhancedColorLight.onChange(setColorLightState);

  // lambda functions are used to set the attribute change callbacks
  EnhancedColorLight.onChangeOnOff([](bool state) {
    Serial.printf("Color Light OnOff changed to %s\r\n", state ? "ON" : "OFF");
    return true;
  });
  EnhancedColorLight.onChangeColorTemperature([](uint16_t colorTemperature) {
    Serial.printf("Color Light Temperature changed to %d\r\n", colorTemperature);
    // get correspondent Hue and Saturation of the color temperature
    HsvColor_t hsvTemperature = espRgbColorToHsvColor(espCTToRgbColor(colorTemperature));
    // keep previous the brightness and just change the Hue and Saturation
    currentHSVColor.h = hsvTemperature.h;
    currentHSVColor.s = hsvTemperature.s;
    return true;
  });
  EnhancedColorLight.onChangeBrightness([](uint8_t brightness) {
    Serial.printf("Color Light brightness changed to %d\r\n", brightness);
    // change current brightness (HSV value)
    currentHSVColor.v = brightness;
    return true;
  });
  EnhancedColorLight.onChangeColorHSV([](HsvColor_t hsvColor) {
    Serial.printf("Color Light HSV Color changed to (%d,%d,%d)\r\n", hsvColor.h, hsvColor.s, hsvColor.v);
    // keep the current brightness and just change Hue and Saturation
    currentHSVColor.h = hsvColor.h;
    currentHSVColor.s = hsvColor.s;
    return true;
  });

  // gets White Light data stored state in NVS
  // default OnOff state is ON if not stored before
  bool lastWhiteOnOffState = matterPref.getBool(onOffWhiteLightPrefKey, true);
  // default brightness ~= 6% (15/255)
  uint8_t lastBrightness = matterPref.getUChar(brightnessWhiteLightPrefKey, 15);
  DimmableLight.begin(lastWhiteOnOffState, lastBrightness);
  // set the callback function to handle the Light state change
  DimmableLight.onChange(setWhiteLightState);

  // lambda functions are used to set the attribute change callbacks
  DimmableLight.onChangeOnOff([](bool state) {
    Serial.printf("White Dimmable Light OnOff changed to %s\r\n", state ? "ON" : "OFF");
    return true;
  });
  DimmableLight.onChangeBrightness([](uint8_t level) {
    Serial.printf("White Dimmable Light Brightness changed to %d\r\n", level);
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
  // configure the Light based on initial state
  EnhancedColorLight.updateAccessory();
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
    button_state = false;  // released
  }

  // Onboard User Button is kept pressed for longer than 5 seconds in order to decommission matter node
  uint32_t time_diff = millis() - button_time_stamp;
  if (button_state && time_diff > decommissioningTimeout) {
    Serial.println("Decommissioning this Matter Accessory. It shall be commissioned again.");
    EnhancedColorLight = false;  // turn the light off
    Matter.decommission();
    button_time_stamp = millis();  // avoid running decommissining again, reboot takes a second or so
  }

  delay(500);
}
