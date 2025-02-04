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
#include <rfm.h>

#define RFM_MISO 19
#define RFM_MOSI 18
#define RFM_SCLK 5
#define RFM_NSEL 17
#define RFM_NRST 16

#define LED 2

#define RADIO_FREQ_HZ 916000000 // US ISM => 902M to 928M
#define RADIO_POWER_dBm 5      // Set between 5 and 20 dBm

#define SERIAL_BAUD 115200

void setup()
{
    delay(1000);

    // Configure the serial port
    Serial.begin(SERIAL_BAUD);

    // set LED as output
    pinMode(LED, OUTPUT);
    digitalWrite(LED, LOW);

    // Initialize the radio
    RFM_Init(RFM_MISO,
             RFM_MOSI,
             RFM_SCLK,
             RFM_NSEL,
             RFM_NRST);
}

void loop()
{
    static int x = 0;
    uint8_t radio_data[64];
    int rval;

    ++x;
    // Serial.print("TX: ");
    // Serial.println(x);

    sprintf((char *)radio_data, "ESP: %d", x);

    // digitalWrite(LED, HIGH);
    // RFM_Send_Package(radio_data, strlen((char *)radio_data) + 1);

    // rval = radio_driver.send(radio_data, strlen((char *)radio_data) + 1);
    // if (rval)
    //     Serial.println("Send Error");

    // rval = radio_driver.wait_packet_tx();
    // if (rval)
    //     Serial.println("TX Timeout");

    // delay(1000);
    // digitalWrite(LED, LOW);
    // delay(5000);

    int pcount = 0;
    uint8_t data_len = 64;

    while (++pcount < 50)
    {
        // if (radio_driver.receive(radio_data, &data_len))
        if (RFM_Get_Package(radio_data, &data_len) == CRC_OK)
        {
            radio_data[data_len] = 0;
            Serial.println();
            Serial.println((char *)&radio_data[0]);
            Serial.print("RSSI: ");
            Serial.println(RFM_Get_Rssi());
        }
        else
        {
            // digitalWrite(LED, HIGH);
            // // delay(100);
            // digitalWrite(LED, LOW);
            // delay(100);
        }
        digitalWrite(LED, pcount & 0x01);
    }
}
