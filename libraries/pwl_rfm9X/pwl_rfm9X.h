/*
 * Copyright (c) PeeWee Labs All rights reserved.
 * Licensed under the MIT License.
 * See LICENSE file in the project root for full license information.
 */

/*
 * pwl_rfm9X is a Simple driver for the HopeRF LoRa RFM9X Modules
 *
 * It has been tested with the RFM95 module, but should work with
 * any of the LoRa modules.
 *
 * Short, simple and cute like a Corgi butt.
 *
 */

#ifndef _PWL_RFM9X_H
#define _PWL_RFM9X_H

#if defined(__AVR__) || defined(ESP32)
#include <Arduino.h>
#else
    #error You need to provide the "uintXX_t" data types
#endif

// Note that interrupts are not directly supported by this driver.
// They can be implemented by an upper layer.  If support is needed,
// implement the ISR and call poll() from that ISR.  Make sure to
// account for the fact that the poll function uses your user
// supplied SPI read and write functions in the poll function
// (i.e. in the ISR).

// Prototype definition for the SPI register read/write
// functions that are user supplied.
typedef int (*pwl_rfm9X_reg_rwr_fptr_t)(uint8_t reg_addr, uint8_t *reg_data, uint32_t len);

// Prototype definition for the required delay function
typedef void (*pwl_rfm9X_ms_delay_t)(uint32_t ms_count);


// The radio can handle packet sizes up to 255 bytes long
// If you don't intend to send packets that long you can
// adjust/define this to a smaller value that will save
// some memory space.
#if !defined(PWL_RFM9X_RX_BUFFER_LEN)
#define PWL_RFM9X_RX_BUFFER_LEN 256
#endif

// Generally this doesn't need to be adjusted, but just in
// case someone has a radio with a different clock.
#if !defined(PWL_RFM9X_BASE_CLOCK_FREQENCY)
#define PWL_RFM9X_BASE_CLOCK_FREQENCY 32000000.0
#endif

// RX_ERROR is an RX that didn't meet the radio's requirements
// If RX_ERROR is returned the radio has been placed back into
// RX_CONTINUOUS mode in anticipation of another packet.
#define PWL_RFM9X_POLL_RX_ERROR   -1
#define PWL_RFM9X_POLL_NO_STATUS   0
#define PWL_RFM9X_POLL_RX_READY    1
#define PWL_RFM9X_POLL_TX_DONE     2

#define PWL_RFM9X_STATUS_GOOD      0
#define PWL_RFM9X_STATUS_BAD      -1


class PWL_RFM9X
{
protected:
    int _write_byte(uint8_t addr, uint8_t data);
    uint8_t _read_byte(uint8_t addr);
    pwl_rfm9X_reg_rwr_fptr_t _write_func;
    pwl_rfm9X_reg_rwr_fptr_t _read_func;
    pwl_rfm9X_ms_delay_t     _delay_func;

    uint8_t  _buffer[PWL_RFM9X_RX_BUFFER_LEN];
    uint8_t  _rxlength;
    volatile uint8_t  _mode;
    volatile int16_t  _rssi;
    uint32_t _freq;
    bool     _rx_valid;

    enum {
        PWL_RFM9X_INTERRUPT_STATUS_MASK  = 0b01101000,
        PWL_RFM9X_RX_DONE_INTERRUPT_FLAG = 0b01000000,
        PWL_RFM9X_RX_CRC_ERROR_FLAG      = 0b00100000,
        PWL_RFM9X_TX_INTERRUPT_MASK      = 0b00001000
    };

    enum {
        RFM9X_LORA_BW_BitPos = 0x04,
        RFM9X_LORA_CR_BitPos = 0x01,
        RFM9X_LORA_SF_BitPos = 0x04,
    };


    enum {
        RFM9X_REG_Fifo                = 0x00,
        RFM9X_REG_OpMode              = 0x01,
        RFM9X_REG_FrfMsb              = 0x06,
        RFM9X_REG_FrfMid              = 0x07,
        RFM9X_REG_FrfLsb              = 0x08,
        RFM9X_REG_PaConfig            = 0x09,
        RFM9X_REG_PaRamp              = 0x0A,
        RFM9X_REG_Ocp                 = 0x0B,
        RFM9X_REG_Lna                 = 0x0C,
        RFM9X_REG_FifoAddrPtr         = 0x0D,
        RFM9X_REG_FifoTxBaseAddr      = 0x0E,
        RFM9X_REG_FifoRxBaseAddr      = 0x0F,
        RFM9X_REG_FifoRxCurrentAddr   = 0x10,
        RFM9X_REG_IrqFlagsMask        = 0x11,
        RFM9X_REG_IrqFlags            = 0x12,
        RFM9X_REG_RxNbBytes           = 0x13,
        RFM9X_REG_RxHeaderCntValueMsb = 0x14,
        RFM9X_REG_RxHeaderCntValueLsb = 0x15,
        RFM9X_REG_RxPacketCntValueMsb = 0x16,
        RFM9X_REG_RxPacketCntValueLsb = 0x17,
        RFM9X_REG_ModemStat           = 0x18,
        RFM9X_REG_PktSnrValue         = 0x19,
        RFM9X_REG_PktRssiValue        = 0x1A,
        RFM9X_REG_RssiValue           = 0x1B,
        RFM9X_REG_HopChannel          = 0x1C,
        RFM9X_REG_ModemConfig1        = 0x1D,
        RFM9X_REG_ModemConfig2        = 0x1E,
        RFM9X_REG_SymbTimeoutLsb      = 0x1F,
        RFM9X_REG_PreambleMsb         = 0x20,
        RFM9X_REG_PreambleLsb         = 0x21,
        RFM9X_REG_PayloadLength       = 0x22,
        RFM9X_REG_MaxPayloadLength    = 0x23,
        RFM9X_REG_HopPeriod           = 0x24,
        RFM9X_REG_FifoRxByteAddr      = 0x25,
        RFM9X_REG_ModemConfig3        = 0x26,
        RFM9X_REG_PpmCorrection       = 0x27,
        RFM9X_REG_FeiMsb              = 0x28,
        RFM9X_REG_FeiMid              = 0x29,
        RFM9X_REG_FeiLsb              = 0x2A,
        RFM9X_REG_RssiWideband        = 0x2C,
        RFM9X_REG_IfFreq1             = 0x2F,
        RFM9X_REG_IfFreq2             = 0x30,
        RFM9X_REG_DetectOptimize      = 0x31,
        RFM9X_REG_InvertIQ            = 0x33,
        RFM9X_REG_HighBwOptimize1     = 0x36,
        RFM9X_REG_DetectionThreshold  = 0x37,
        RFM9X_REG_SyncWord            = 0x39,
        RFM9X_REG_HighBwOptimize2     = 0x3A,
        RFM9X_REG_InvertIQ2           = 0x3B,
        RFM9X_REG_DioMapping1         = 0x40,
        RFM9X_REG_DioMapping2         = 0x41,
        RFM9X_REG_Version             = 0x42,
        RFM9X_REG_Tcxo                = 0x4B,
        RFM9X_REG_PaDac               = 0x4D,
        RFM9X_REG_FormerTemp          = 0x5B,
        RFM9X_REG_AgcRef              = 0x61,
        RFM9X_REG_AgcThresh1          = 0x62,
        RFM9X_REG_AgcThresh2          = 0x63,
        RFM9X_REG_AgcThresh3          = 0x64,
        RFM9X_REG_Pll                 = 0x70
    };


public:
    enum lora_bw_t {
        RFM9X_LORA_BW_7p8k,            // 0x00
        RFM9X_LORA_BW_10p4k,           // 0x01
        RFM9X_LORA_BW_15p6k,           // 0x02
        RFM9X_LORA_BW_20p8k,           // 0x03
        RFM9X_LORA_BW_31p25k,          // 0x04
        RFM9X_LORA_BW_41p7k,           // 0x05
        RFM9X_LORA_BW_62p5k,           // 0x06
        RFM9X_LORA_BW_125k,            // 0x07
        RFM9X_LORA_BW_250k,            // 0x08
        RFM9X_LORA_BW_500k,            // 0x09
    };


    enum lora_cr_t {
        RFM9X_LORA_CR_4_5,             // 0x00
        RFM9X_LORA_CR_4_6,             // 0x01
        RFM9X_LORA_CR_4_7,             // 0x02
        RFM9X_LORA_CR_4_8,             // 0x03
    };


    enum lora_sf_t {
        // Currently this driver supports only explicit header mode
        // Setting the spreading factor to 6 is no compatible with 
        // explicit header mode... so no support.
        // RFM9X_LORA_SF_64 = 6,       // 0x06
        RFM9X_LORA_SF_128 = 7,         // 0x07
        RFM9X_LORA_SF_256,             // 0x08
        RFM9X_LORA_SF_512,             // 0x09
        RFM9X_LORA_SF_1024,            // 0x0A
        RFM9X_LORA_SF_2048,            // 0x0B
        RFM9X_LORA_SF_4096,            // 0x0C
    };

    enum lora_mode_t {
        RFM9X_LORA_MODE_SLEEP = 0x00,  // 0x00
        RFM9X_LORA_MODE_STDBY,         // 0x01
        RFM9X_LORA_MODE_FSTX,          // 0x02  // No support in driver
        RFM9X_LORA_MODE_TX,            // 0x03
        RFM9X_LORA_MODE_FSRX,          // 0x04  // No support in driver
        RFM9X_LORA_MODE_RX_CONTINUOUS, // 0x05
        RFM9X_LORA_MODE_RX,            // 0x06  // No support in driver
        RFM9X_LORA_MODE_CAD,           // 0x07  // No support in driver
        RFM9X_LORA_MODE_INVALID = 0xFF
    };

    // When the class is instantiated the register read and write
    // functions, and the delay function must be provided.  These
    // functions provide the low level access via SPI to the RFM9X
    // registers.
    PWL_RFM9X(pwl_rfm9X_reg_rwr_fptr_t write_function,
              pwl_rfm9X_reg_rwr_fptr_t read_function,
              pwl_rfm9X_ms_delay_t     delay_function) :
                  _write_func(write_function),
                  _read_func(read_function),
                  _delay_func(delay_function),
                  _rx_valid(0),
                  _mode(RFM9X_LORA_MODE_INVALID),
                  _rxlength(0),
                  _rssi(0),
                  _freq(0) { }

    // Call init with the following LoRa radio parameters.  If you don't
    // know what these are please read a primer on LoRa before using this
    // driver:
    // Hz is the target carrier/center frequency.  (i.e. 915000000)
    // dBm is the TX power.  This must be between 5 and 20.
    // bw is the Channel bandwidth.  See lora_bw_t above.
    // cr is the Coding Rate.  See lora_cr_t above.
    // sf is the Spreading Factor.  See lora_sf_t above.
    // Return 0 for success, -1 if the radio isn't returning expected mode data
    int init(uint32_t  Hz,
             int       dBm,
             lora_bw_t bw,
             lora_cr_t cr,
             lora_sf_t sf);

    // Use set_center_frequency to adjust the carrier frequency after initialization if desired
    // Return 0 for success, non-zero for error
    int set_center_frequency(uint32_t Hz);

    // Use set_power_amp to adjust the TX power after initialization if desired
    // dBm must be between 5 and 20
    // Return 0 for success, non-zero for error
    int set_power_amp(int dBm);

    // Use set_mode to select one of the operating modes given by lora_mode_t
    // Return 0 for success, non-zero for error
    int set_mode(lora_mode_t mode);

    // Read back the current mode from the radio
    // Returns the current mode value
    uint8_t get_mode(void);

    // Instead of interrupt driven, this driver is polled.
    // poll() must be called regularly when waiting for RX data in RFM9X_LORA_MODE_RX_CONTINUOUS
    // mode.  It can also be called regularly during TX to determine when TX is complete.
    // Once RX data is identified by polling call get_rx_data to copy the data to
    // your local buffer.
    // Returns one of the defined "PWL_RFM9X_POLL_..." status values
    int poll();

    // rx_data_ready() returns true if there is RX data ready that hasn't been copied out of the buffer.
    // Returns true if there is RX data ready
    bool rx_data_ready();

    // receive() sets the mode to RX, if data has been received it copies the received data
    // to user's buffer and clears the rx_data_ready status.
    // This function can be "polled" in place of the poll function if waiting for RX data
    // Returns the number of bytes received or 0 if no data is available
    uint8_t receive(uint8_t* buf, uint8_t* len);

    // send() Transmit the given data
    // Return 0 for success, non-zero for error
    int send(uint8_t* data, uint8_t len);

    // Send returns before the data has transmitted.  Call wait_packet_tx to block until done.
    // Starting another TX or RX before it is done will cause a loss of TX data.
    // Return 0 for success (TX done), non-zero for error (timeout)
    int wait_packet_tx(int max_wait_ms=1000);

    // Return the Receive Signal Strength Indicator (approximated by the radio)
    int16_t get_rssi(void) { return _rssi; }
};

#endif // _PWL_RFM9X_H