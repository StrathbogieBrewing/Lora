#!/bin/bash

set -e

arduino-cli config init
arduino-cli config set directories.user /Arduino

# socat  /dev/ttyUSB0,b115200,raw -

# arduino-cli compile --fqbn esp32:esp32:esp32 rfm95

# arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32 rfm95

arduino-cli compile --fqbn STMicroelectronics:stm32:LoRa --board-options "pnum=LORA_E5_MINI" stm32wl

arduino-cli upload --fqbn STMicroelectronics:stm32:LoRa --board-options "pnum=LORA_E5_MINI" stm32wl



# docker run -it --privileged -v /dev/:/dev/ -v $PWD:/Arduino -w /Arduino arduino-cli:1.0.0


