# Embedded World 2025 - Arduino Matter Demo

This 4 applications are designed for the ESP32-C5 using 5GHz WiFi Matter.

It is based on the ESP32 Arduino Matter Library that can be found at
https://github.com/espressif/arduino-esp32/tree/master/libraries/Matter

Each application uses M5Stack accessories:

- **Demo1:** RGB CWW Light ([M5Stack RGB LED Unit](https://shop.m5stack.com/products/rgb-unit) with 3 LEDs) and Dimmable Light ([M5Stack Unit FlashLight](https://docs.m5stack.com/en/unit/FlashLight))
- **Demo2:** Contact Sensor (Magnetic Reed Switch) + Single Press Smart Button with backlight feedback ([M5Stack Unit Key](https://docs.m5stack.com/en/unit/key))
- **Demo3:** Temperature, Humidity and Pressure BME688 Sensor ([M5Stack ENV PRO](https://docs.m5stack.com/en/unit/ENV%20Pro%20Unit))

## Building the applications

Those 3 examples were built using Arduino IDE with ESP32 Arduino v3.1.3
https://github.com/espressif/arduino-esp32/releases/tag/3.1.3
https://github.com/espressif/arduino-esp32/tree/release/v3.1.x

It is recommended to use the Huge APP Partition scheme beacuse the binary has more than 1.2MB.\
It is strongly recommended that the flash is erased before uploading in order to avoid using previous NVS Matter stored data.\
Some useful log information may be seen in the Console UART0 when using Core Debug Level as Info.

--------
This is mounted using a board with grove connectors like this one:

![image](https://github.com/user-attachments/assets/1cc6ce14-49f4-4774-a112-6e10d236dcc8)

