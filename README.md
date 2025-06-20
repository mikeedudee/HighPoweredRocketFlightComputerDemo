# High-Powered Rocket Flight Computer Demo

[![License](https://img.shields.io/badge/license-MIT-green.svg)]()

This is a sample or demonstration of a working flight computer for a high-powered rocket using Arduino IDE. It supports multiple microcontrollers. This program has been tested and is considered ready for flight and is 100% working. Though I recommend changing some parameters to suit your mission profile, always test after you have made changes. 

> This software was ran and tested using ESP32. Some libraries may or may not be needed on your end, always check which library supports your working environment.

---

## Key variables to change:
 - `ALT_THRESHOLD_GREEN`  = Detection of launch.
 - `ALT_THRESHOLD_RED`    = Detection of Apogee (theoretically based on your calculation).
 - `ALT_RESET_THRESHOLD`  = Detection of prominent touchdown.

## Micro-Controller Compatible
Any of the following:
  - Teensy
  - Arduino
  - ESP32

## Usage
1. **Dependencies**
   - `FS.h` — File System
   - `SPIFFS.h` — SPI Flash File System
   - `Wire.h` — For I2C communication
   - `MS5611.h` — For this, you need to use the older version of the library, specifically `1.1.0`
   - `esp_system.h` — required only for those who uses ESP32 micro-controllers since in this code we utilize comprehensive contingecy and reporting methods to debug.
   - `SimpleKalmanFilter.h` — For altitude smoothing (moving average)
