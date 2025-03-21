# ESP32-SIM7000G-Tracker

## Overview
I built this GPS Tracker project using a LilyGo T-SIM7000G development board to capture real-time GPS coordinates and monitor battery levels, then send that data to my Adafruit IO dashboard via MQTT. You'll need to update your own GPRS credentials—APN, username, and password—as well as your Adafruit IO username, key, and MQTT topics to match your feeds. I used libraries: TinyGsmClient for modem communication, Adafruit_MQTT for managing the MQTT connection, and vfs_api for file system support, with an optional StreamDebugger to help with AT command debugging if needed. Also, make sure to double-check and adjust the pin assignments (for TX, RX, LED, and battery measurement) to fit your specific hardware setup before running the tracker.
