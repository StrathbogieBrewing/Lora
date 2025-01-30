# Simple RFM95 LoRa Driver

Copyright © PeeWee Labs, All Rights Reserved.  Licensed under the MIT License, see the LICENSE file in the root of this repository.

## About
A simple, single purpose, driver for the HopeRF LoRa module.  This is not a swiss army knife of radio drivers.  It is intended to be small, and simple to use;  to receive and transmit packet data using the radio's LoRa mode.

This driver is polled and does not directly support interrupts.  If you need support for interrupts, it should be easy enough to derive a class from this that adds the interrupt support.

For Arduino based platforms, see the examples.

## Instantiation
### Include the header:

    #include <pwl_rfm9X.h>
### Provide three functions that are used by the driver:

    int spi_read_register(uint8_t reg_addr, uint8_t *reg_data, uint32_t len);
    int spi_write_register(uint8_t reg_addr, uint8_t *reg_data, uint32_t len);
    void delay_milliseconds(uint32_t ms);

### Instantiate the class:

    PWL_RFM9X radio_driver(spi_read_register, spi_write_register, delay_milliseconds);
## Initialization
IMPORTANT NOTE:  This driver uses the SPI read write functions provided above.  If you need to initialize your SPI hardware/software, then do so before initializing the radio driver.

        radio_driver.init(uint32_t  RADIO_FREQ,
                          uint8_t   RADIO_POWER,
                          lora_bw_t LORA_BANDWIDTH,
                          lora_cr_t LORA_CODE_RATE,
                          lora_sf_t LORA_SPREADING_FACTOR);
#### The initialization parameters:
##### RADIO_FREQ
The carrier (center) frequency on which the radio will transmit and receive packet data.  Please see your radio's documentation for the valid range of this parameter.
##### RADIO_POWER
The number of dBm boost the power amp should give when transmitting data.  This number must be between 5 and 20 for this driver.
##### LORA_BANDWIDTH
Provide one of the values enumerated by the lora_bw_t type.  See the pwl_rfm9X for the possible values.
##### LORA_CODE_RATE
Provide one of the values enumerated by the lora_cr_t type.  See the pwl_rfm9X for the possible values.
##### LORA_SPREADING_FACTOR
Provide one of the values enumerated by the lora_sf_t type.  See the pwl_rfm9X for the possible values.

---
**NOTE:** For the **LORA_XXX** parameters, see one of numerous LoRa primers on the web.  Generally these will tradeoff between data rate and reliability.
## Transmit

        int send(uint8_t* data, uint8_t len);
Returns PWL_RFM9X_STATUS_GOOD if the packet was transferred to the radio.

WARNING:  This function returns before the radio has finished transmitted the data.  Changing the radio's mode while the radio is transmitting the packet will cause the ongoing transmission to be aborted.  Calling "receive," "send," or "set_mode" while the radio is transmitting will all cause this abort.

To wait for the packet transmission to complete call this function:

    int wait_packet_tx(int max_wait_ms=1000);
## Receive

    uint8_t receive(uint8_t* buf, uint8_t* len);
This function places the radio into receive mode if it is not already in receive mode.  If the radio is in receive mode then this function checks if a packet has been received and copies the packet into the provided buffer.

When this function is called, the *len parameter must contain the length of the provided *buf buffer to prevent overwriting.

If no packet is available when this function is called, the return value is zero (0) and the *len field is **not** modified.

If a packet is received and copied, then *len is updated to reflect the number of bytes that were copied into *buf.  The return value is also the number of bytes received.

NOTE:  This function is non-blocking.  It will not wait for a packet to be received.  It must be called repeatedly until it returns a non-zero value.
## Tested With
This driver has been tested on the following platforms:

 - Arduino Nano + RFM95 module
 - Moteino with RFM95
 - ATTiny 1614 + RFM95w module
