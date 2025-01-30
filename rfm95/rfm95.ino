/*
 * Copyright (c) PeeWee Labs All rights reserved.
 * Licensed under the MIT License.
 * See LICENSE file in the project root for full license information.
 */

/*
 * TX Example for pwl_rfm9X library.
 */

#include <Arduino.h>
#include <SPI.h>
#include <pwl_rfm9X.h>

#define RFM_MISO 19
#define RFM_MOSI 18
#define RFM_SCLK 5
#define RFM_NSEL 17
#define RFM_NRST 16

#define LED 2

#define RADIO_FREQ_HZ 916000000 // US ISM => 902M to 928M
#define RADIO_POWER_dBm 10     // Set between 5 and 20 dBm

#define SERIAL_BAUD 115200

// The pwl_rfm9X library requires us to provide the register read
// and write functions.

int rf95_reg_read(uint8_t reg_addr, uint8_t *data, uint32_t len) {
    uint32_t idx;
    digitalWrite(RFM_NSEL, LOW);
    SPI.transfer(reg_addr);
    for (idx = 0; idx < len; idx++) {
        data[idx] = SPI.transfer(0);
    }
    digitalWrite(RFM_NSEL, HIGH);
    return 0;
}

int rf95_reg_write(uint8_t reg_addr, uint8_t *data, uint32_t len) {
    uint32_t idx;
    digitalWrite(RFM_NSEL, LOW);
    SPI.transfer(reg_addr);
    for (idx = 0; idx < len; idx++) {
        SPI.transfer(data[idx]);
    }
    digitalWrite(RFM_NSEL, HIGH);
    return 0;
}

// Instantiate the RFM9X driver here.  Pass the write/read/delay
// functions.
PWL_RFM9X radio_driver(rf95_reg_write, rf95_reg_read, delay);

void setup() {
    delay(1000);

    // Configure the serial port
    Serial.begin(SERIAL_BAUD);
    while (!Serial)
        ;
    Serial.println("It's alive!");

    // set LED as output
    pinMode(LED, OUTPUT);
    digitalWrite(LED, LOW);

    // Reset the radio
    pinMode(RFM_NRST, OUTPUT);
    digitalWrite(RFM_NRST, LOW);
    delay(10);
    digitalWrite(RFM_NRST, INPUT_PULLUP);
    delay(10);

    // Configure the Radio SPI Slave Select pin
    pinMode(RFM_NSEL, OUTPUT);
    digitalWrite(RFM_NSEL, HIGH);
    SPI.begin(RFM_SCLK, RFM_MISO, RFM_MOSI, RFM_NSEL);
    SPI.setClockDivider(SPI_CLOCK_DIV8);

    // Initialize the radio
    radio_driver.init(
        RADIO_FREQ_HZ, RADIO_POWER_dBm, PWL_RFM9X::RFM9X_LORA_BW_125k,
        PWL_RFM9X::RFM9X_LORA_CR_4_5, PWL_RFM9X::RFM9X_LORA_SF_128);
}

void loop() {
    static int x = 0;
    uint8_t radio_data[64];
    int rval;

    ++x;
    Serial.print("TX: ");
    Serial.println(x);

    sprintf((char *)radio_data, "ESP: %d", x);

    rval = radio_driver.send(radio_data, strlen((char *)radio_data) + 1);
    if (rval)
        Serial.println("Send Error");

    rval = radio_driver.wait_packet_tx();
    if (rval)
        Serial.println("TX Timeout");

    digitalWrite(LED, x & 0x01);


    int pcount = 0;
    uint8_t data_len = 64;

    while (++pcount < 100) {
        if (radio_driver.receive(radio_data, &data_len)) {
            radio_data[data_len] = 0;
            Serial.println();
            Serial.println((char *)&radio_data[0]);
            Serial.print("RSSI: ");
            Serial.println(radio_driver.get_rssi());
        }

        delay(100);
    }
}
