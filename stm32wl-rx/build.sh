#!/bin/bash
set -e
arduino-cli compile --fqbn STMicroelectronics:stm32:LoRa --board-options "pnum=LORA_E5_MINI" stm32wl-rx
arduino-cli upload --fqbn STMicroelectronics:stm32:LoRa --board-options "pnum=LORA_E5_MINI" stm32wl-rx
# socat  /dev/ttyUSB0,b115200,raw -