#!/bin/bash
set -e

export PATH="/usr/local/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin:$PATH"

arduino-cli compile --fqbn STMicroelectronics:stm32:LoRa --board-options "pnum=LORA_E5_MINI" stm32wl-tx
arduino-cli upload --fqbn STMicroelectronics:stm32:LoRa --board-options "pnum=LORA_E5_MINI" stm32wl-tx
# socat  /dev/ttyUSB0,b115200,raw -