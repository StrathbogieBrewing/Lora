#!/bin/bash
set -e
arduino-cli compile --fqbn esp32:esp32:esp32 rfm95
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32 rfm95
socat  /dev/ttyUSB0,b115200,raw -