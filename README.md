# Embedded World 2025 - Arduino Matter Demo

This 4 applications are designed for the ESP32-C5 using 5GHz WiFi Matter.

It is based on the ESP32 Arduino Matter Library that can be found at
https://github.com/espressif/arduino-esp32/tree/master/libraries/Matter

Each application uses M5Stack accessories:

- Temperature, Humidity and Pressure BME688 Sensor ([M5Stack ENV PRO](https://docs.m5stack.com/en/unit/ENV%20Pro%20Unit))
- Dimmable Light ([M5Stack Unit FlashLight](https://docs.m5stack.com/en/unit/FlashLight))
- RGB CWW Light ([M5Stack RGB LED Unit](https://shop.m5stack.com/products/rgb-unit) with 3 LEDs)
- Contact Sensor (Magnetic Reed Switch) + Single Press Smart Button with backlight feedback ([M5Stack Unit Key](https://docs.m5stack.com/en/unit/key))

This is mounted using a board with grove connectors like this one:

![image](https://github.com/user-attachments/assets/1cc6ce14-49f4-4774-a112-6e10d236dcc8)

