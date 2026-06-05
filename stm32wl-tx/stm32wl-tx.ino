#include "STM32LowPower.h"
#include <RadioLib.h>

#include <STM32RTC.h>

#define PIN_PWR PB9
#define PIN_TRG PA0
#define PIN_ADC_INPUT PB14

#define DEVICE_TYPE 0xEF // Electric Fence

#define BUFFER_SIZE 16

// #define SLEEP_MS (10000UL)
#define SLEEP_MS (60000UL * 5UL)

/* Get the rtc object */
STM32RTC &rtc = STM32RTC::getInstance();

STM32WLx radio = new STM32WLx_Module();

HardwareSerial mySerial(USART2);

static const uint32_t rfswitch_pins[] = {PA4, PA5, RADIOLIB_NC, RADIOLIB_NC,
                                         RADIOLIB_NC};
static const Module::RfSwitchMode_t rfswitch_table[] = {
    {STM32WLx::MODE_IDLE, {LOW, LOW}},
    {STM32WLx::MODE_RX, {HIGH, LOW}},
    // {STM32WLx::MODE_TX_HP, {LOW, HIGH}},  // for LoRa-E5 mini
    {STM32WLx::MODE_TX_LP, {HIGH, HIGH}}, // for LoRa-E5-LE mini
    END_OF_MODE_TABLE,
};

/* callback function once the Alarm A matched */
void alarmMatch(void *data) { UNUSED(data); }

uint16_t getFence_V(uint16_t battery_mv) {
    pinMode(PIN_TRG, INPUT_PULLUP);

    pinMode(PIN_PWR, OUTPUT);
    digitalWrite(PIN_PWR, HIGH);

    // wait for trigger to be inactive without timeout
    while (digitalRead(PIN_TRG) == LOW) {
        ;
    }

    unsigned long trigger_us = 0;
    unsigned long timeout = micros() + 5000000L;

    while (micros() < timeout) {
        if (digitalRead(PIN_TRG) == LOW) {
            trigger_us = micros();
            break;
        }
    }

    // wait for trigger to be inactive without timeout
    while (digitalRead(PIN_TRG) == LOW) {
        ;
    }

    int sensorValue = 0;
    if (trigger_us) {
        trigger_us = micros() - trigger_us;
        analogReadResolution(12);
        sensorValue = (uint16_t)(((uint32_t)analogRead(PIN_ADC_INPUT) *
                                  (uint32_t)battery_mv) /
                       4096ULL) * ((4700ULL + 4700ULL + 48ULL + 2ULL) / 2200ULL);
    }

    digitalWrite(PIN_PWR, LOW);
    pinMode(PIN_TRG, INPUT_PULLDOWN);
    return sensorValue;
}

uint16_t getBattery_mV(void) {
    analogReadResolution(12);
    uint16_t battery_mv = 0;
    int vrefint_raw_adc = analogRead(AVREF);
    uint16_t vref_cal = *(uint16_t *)VREFINT_CAL_ADDR;
    if (vrefint_raw_adc > 0) {
        battery_mv = __HAL_ADC_CALC_VREFANALOG_VOLTAGE(vrefint_raw_adc,
                                                       ADC_RESOLUTION_12B);
    }
    return battery_mv;
}

int16_t getTemperature_C(void) {
    analogReadResolution(12);
    uint16_t adcVrefRaw = analogRead(AVREF);
    uint32_t vdda =
        __HAL_ADC_CALC_VREFANALOG_VOLTAGE(adcVrefRaw, ADC_RESOLUTION_12B);
    uint16_t adcTempRaw = analogRead(ATEMP);
    int32_t temperature_c =
        __HAL_ADC_CALC_TEMPERATURE(vdda, adcTempRaw, ADC_RESOLUTION_12B);

    // uint16_t temperature_c = analogRead(ATEMP) / 10;
    return temperature_c;
}

void setup() {
    // Serial.begin(115200);
    // Serial.print("Power up ");
    SystemClock_ConfigFromStop();

    radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);
    radio.begin(916.0);

    radio.setBandwidth(125.0);
    radio.setSpreadingFactor(10);
    radio.setCodingRate(RADIOLIB_SX126X_LORA_CR_4_8);
    radio.setOutputPower(14);
    radio.setTCXO(1.7);

    int16_t temperature_c = getTemperature_C();
    uint16_t battery_mv = getBattery_mV();
    uint16_t fence_v = getFence_V(battery_mv);

    uint8_t buffer[BUFFER_SIZE] = {0};

    uint32_t device_id = HAL_GetUIDw0();
    buffer[0] = 0; // reserved for routing
    buffer[1] = DEVICE_TYPE;
    buffer[2] = device_id >> 24;
    buffer[3] = device_id >> 16;
    buffer[4] = device_id >> 8;
    buffer[5] = device_id >> 0;

    buffer[6] = battery_mv >> 8;
    buffer[7] = battery_mv & 0xff;

    buffer[8] = fence_v >> 8;
    buffer[9] = fence_v & 0xff;

    buffer[10] = temperature_c >> 8;
    buffer[11] = temperature_c & 0xff;

    size_t packet_size = 12;

    radio.transmit(buffer, packet_size);
    delay(100);

    radio.sleep(true);

    // Configure low power
    LowPower.begin();
    LowPower.enableWakeupFrom(&rtc, alarmMatch);
    LowPower.shutdown(SLEEP_MS);
}

void loop() {}

// uint32_t getDistance_mm(void) {
//     mySerial.begin(9600);
//     mySerial.write(0x01);
//     delay(50);
//     if (mySerial.available()) {
//         uint32_t distance_mm;
//         uint8_t startByte, h_data, l_data, sum = 0;
//         uint8_t buf[3];

//         startByte = (byte)mySerial.read();
//         if (startByte == 255) {
//             mySerial.readBytes(buf, 3);
//             h_data = buf[0];
//             l_data = buf[1];
//             sum = buf[2];
//             distance_mm = (h_data << 8) + l_data;
//             if (((h_data + l_data - 1) & 0xFF) == sum) {
//                 return distance_mm;
//             } else {
//                 return 0;
//             }
//         }
//     }
//     return 0;
// }

// void clock_reinit(void) {
//     RCC_OscInitTypeDef osc = {};
//     RCC_ClkInitTypeDef clk = {};

//     osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
//     osc.HSIState = RCC_HSI_ON;
//     osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
//     osc.PLL.PLLState = RCC_PLL_NONE;

//     HAL_RCC_OscConfig(&osc);

//     clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
//                     RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;

//     clk.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
//     clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
//     clk.APB1CLKDivider = RCC_HCLK_DIV1;
//     clk.APB2CLKDivider = RCC_HCLK_DIV1;

//     HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_0);
// }