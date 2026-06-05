arduino-cli compile --build-path build --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc XiaoS3

arduino-cli upload --build-path build --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc -p /dev/ttyACM0 XiaoS3

socat /dev/ttyACM0,raw,b115200 -

# /root/.arduino15/packages/esp32/tools/esp-x32/2405/bin/xtensa-esp32s3-elf-addr2line -pfiaC -e build/xiao_lora.ino.elf 0x42002099
