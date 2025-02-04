/******************************************************************************************
 * Copyright 2017 Ideetron B.V.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 ******************************************************************************************/
/****************************************************************************************
 * File:     RFM95.cpp
 * Author:   Gerben den Hartog
 * Compagny: Ideetron B.V.
 * Website:  http://www.ideetron.nl/LoRa
 * E-mail:   info@ideetron.nl
 ****************************************************************************************/
/****************************************************************************************
 * Created on:         06-01-2017
 * Supported Hardware: ID150119-02 Nexus board with RFM95
 ****************************************************************************************/

#include <Arduino.h>
#include <SPI.h>
#include "rfm.h"

// #define DEBUG

/*
*****************************************************************************************
* REGISTER DEFINITIONS
*****************************************************************************************
*/

typedef enum
{
    RFM_REG_FIFO = 0x00,
    RFM_REG_OP_MODE = 0x01,
    RFM_REG_FR_MSB = 0x06,
    RFM_REG_FR_MID = 0x07,
    RFM_REG_FR_LSB = 0x08,
    RFM_REG_OCP = 0x0b,
    RFM_REG_PA_CONFIG = 0x09,
    RFM_REG_LNA = 0x0C,
    RFM_REG_FIFO_ADDR_PTR = 0x0D,
    RFM_REG_IRQ_FLAGS = 0x12,
    RFM_REG_LAST_RSSI = 0x1A,
    RFM_REG_MODEM_CONFIG1 = 0x1D,
    RFM_REG_MODEM_CONFIG2 = 0x1E,
    RFM_REG_SYM_TIMEOUT_LSB = 0x1F,
    RFM_REG_PREAMBLE_MSB = 0x20,
    RFM_REG_PREAMBLE_LSB = 0x21,
    RFM_REG_PAYLOAD_LENGTH = 0x22,
    RFM_REG_MODEM_CONFIG3 = 0x26,
    RFM_REG_INVERT_IQ = 0x33,
    RFM_REG_INVERT_IQ2 = 0x3b,
    RFM_REG_SYNC_WORD = 0x39,
    RFM_REG_DIO_MAPPING1 = 0x40,
    RFM_REG_DIO_MAPPING2 = 0x41,
    RFM_REG_PA_DAC = 0x4d

} rfm_register_t;

typedef enum
{
    RFM_MODE_SLEEP = 0b000,
    RFM_MODE_STANDBY = 0b001,
    RFM_MODE_FSTX = 0b010,
    RFM_MODE_TX = 0b011,
    RFM_MODE_FSRX = 0b100,
    RFM_MODE_RXCONT = 0b101,
    RFM_MODE_RXSINGLE = 0b110,
    RFM_MODE_CAD = 0b111,
    RFM_MODE_LORA = 0b10000000
} frm_mode_t;

typedef enum
{
    IRQ_RX_TIMEOUT_MASK = 0b10000000,
    IRQ_RX_DONE_MASK = 0b01000000,
    IRQ_TX_DONE_MASK = 0b00001000
} irq_mask;

static uint8_t rfm_nsel = 0;
static uint8_t rfm_nrst = 0;

/*
*****************************************************************************************
* Description : Function that reads a register from the RFM and returns the value
*
* Arguments   : address Address of register to be read
*
* Returns   : Value of the register
*****************************************************************************************
*/
static unsigned char RFM_Read(unsigned char address)
{
    unsigned char data;

    // Add transactions in Read and Write methods
    SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));

    // Set NSS pin low to start SPI communication
    digitalWrite(rfm_nsel, LOW);

    // Send Address
    SPI.transfer(address);
    // Send 0x00 to be able to receive the answer from the RFM
    data = SPI.transfer(0x00);

    // Set NSS high to end communication
    digitalWrite(rfm_nsel, HIGH);

    // End the transaction so that other hardware can use the bus
    SPI.endTransaction();

#ifdef DEBUG
    Serial.print("SPI Read ADDR: ");
    Serial.print(address, HEX);
    Serial.print(" DATA: ");
    Serial.println(data, HEX);
#endif

    // Return received data
    return data;
}

/*
*****************************************************************************************
* Description : Function that writes a register from the RFM
*
* Arguments   : address Address of register to be written
*         data    Data to be written
*****************************************************************************************
*/
void RFM_Write(unsigned char address, unsigned char data)
{
// br: SPI Transfer Debug
#ifdef DEBUG
    Serial.print("SPI Write ADDR: ");
    Serial.print(address, HEX);
    Serial.print(" DATA: ");
    Serial.println(data, HEX);
#endif

    // Add transactions in Read and Write methods
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

    // Set NSS pin Low to start communication
    digitalWrite(rfm_nsel, LOW);

    // Send Address with MSB 1 to make it a write command
    SPI.transfer(address | 0x80);
    // Send Data
    SPI.transfer(data);

    // Set NSS pin High to end communication
    digitalWrite(rfm_nsel, HIGH);

    // End the transaction so that other hardware can use the bus
    SPI.endTransaction();
}

/*
*****************************************************************************************
* Description : Function to change the operation mode of the RFM. Switching mode and wait
*				for mode ready flag
*               DO NOT USE FOR SLEEP
*
* Arguments   : Mode the mode to set
*****************************************************************************************
*/
void RFM_Switch_Mode(unsigned char Mode)
{
    Mode = Mode | 0x80; // Set high bit for LoRa mode

    // Switch mode on RFM module
    RFM_Write(RFM_REG_OP_MODE, Mode);
}

/*
*****************************************************************************************
* Description : Function to retrieve value of the last received packet rssi register
*****************************************************************************************
*/
int16_t RFM_Get_Rssi()
{
    return RFM_Read(RFM_REG_LAST_RSSI);
}

/********************************************************************************************
 * Description : Change Spread Factor and Band Width
 *
 * Arguments:    _SF = {6,7,8,9,10,11,12}
 *               _BW = {0x00 -> 7.8khz   , 0x01 -> 10.4khz, 0x02 -> 15.6khz, 0x03 -> 20.8khz,
 *                      0x04 -> 31.25khz , 0x05 -> 41.7khz, 0x06 -> 62.5khz, 0x07 -> 125khz,
 *                      0x08 -> 250khz   , 0x09 -> 500khz}
 ********************************************************************************************/
static void RFM_Change_SF_BW(unsigned char _SF, unsigned char _BW)
{
    RFM_Write(RFM_REG_MODEM_CONFIG2, (_SF << 4) | 0b0100); // SFx CRC On
    RFM_Write(RFM_REG_MODEM_CONFIG1, (_BW << 4) | 0x02);   // x kHz 4/5 coding rate explicit header mode

    if (_SF > 10)
        RFM_Write(RFM_REG_MODEM_CONFIG3, 0b1100); // Low datarate optimization on AGC auto on
    else
        RFM_Write(RFM_REG_MODEM_CONFIG3, 0b0100); // Mobile node, low datarate optimization on AGC acorging to register LnaGain
}

/*
*****************************************************************************************
* Description : Function to change the channel of the RFM module. Setting the following
*				register: Channel
*
* Arguments   : Channel the channel to set
*****************************************************************************************
*/
static void RFM_Change_Channel(uint32_t freq_hz)
{
    // { 0xE5, 0x00, 0x26 }, //Channel [4], 916,0 MHz / 61.035 Hz = 15007782 = 0xE50026
    RFM_Write(RFM_REG_FR_MSB, 0xE5);
    RFM_Write(RFM_REG_FR_MSB + 1, 0x00);
    RFM_Write(RFM_REG_FR_MSB + 2, 0x26);

    // float freq_units = ((float)Hz) / (PWL_RFM9X_BASE_CLOCK_FREQENCY / 524288.0);
    // uint32_t fr = (uint32_t)freq_units;
    // _write_byte(RFM9X_REG_FrfMsb, (fr >> 16) & 0xFF);
    // _write_byte(RFM9X_REG_FrfMid, (fr >>  8) & 0xFF);
    // _write_byte(RFM9X_REG_FrfLsb, (fr >>  0) & 0xFF);
    // _freq = Hz;
    // return PWL_RFM9X_STATUS_GOOD;
}

/*
*****************************************************************************************
* Description: Function used to initialize the RFM module on startup
*****************************************************************************************
*/
bool RFM_Init(uint8_t miso, uint8_t mosi, uint8_t sclk, uint8_t nsel, uint8_t nrst)
{
    rfm_nsel = nsel;
    rfm_nrst = nrst;

    // Configure the Radio SPI Slave Select pin
    pinMode(nsel, OUTPUT);
    digitalWrite(nsel, HIGH);
    SPI.begin(sclk, miso, mosi, nsel);

    // Reset the radio
    pinMode(nrst, OUTPUT);
    digitalWrite(nrst, LOW);
    delay(10);
    digitalWrite(nrst, INPUT_PULLUP);
    delay(10);

    uint8_t ver = RFM_Read(0x42);
    if (ver != 18)
    {
        Serial.println("Wrong version");
        return 0;
    }
    // Switch RFM to sleep
    // DON'T USE Switch mode function
    RFM_Write(RFM_REG_OP_MODE, RFM_MODE_SLEEP);

    // Wait until RFM is in sleep mode
    delay(50);

    // Set RFM in LoRa mode
    // DON'T USE Switch mode function
    RFM_Write(RFM_REG_OP_MODE, RFM_MODE_LORA);
    // Switch RFM to standby
    RFM_Switch_Mode(RFM_MODE_STANDBY);
    // Set channel to channel 0
    RFM_Change_Channel(0);
    // Set default power to maximun on US915 TODO AS/AU/EU config
    // Set the default output pin as PA_BOOST
    // RFM_Set_Tx_Power(20, 0);

    // Switch LNA boost on
    RFM_Write(RFM_REG_LNA, 0x23);

    // Set RFM To datarate 0 SF12 BW 125 kHz
    RFM_Change_SF_BW(10, 0x07);
    // RFM_Change_Datarate(0x00);

    // Rx Timeout set to 37 symbols
    RFM_Write(RFM_REG_SYM_TIMEOUT_LSB, 0x25);
    // RFM_Write(RFM_REG_SYM_TIMEOUT_LSB, 0x05);

    // Preamble length set to 8 symbols
    // 0x0008 + 4 = 12
    RFM_Write(RFM_REG_PREAMBLE_MSB, 0x00);
    RFM_Write(RFM_REG_PREAMBLE_LSB, 0x08);

    // Set LoRa sync word
    RFM_Write(RFM_REG_SYNC_WORD, 0x34);

    // Set FIFO pointers
    // TX base address
    RFM_Write(0x0E, 0x80);
    // Rx base address
    RFM_Write(0x0F, 0x00);
    return 1;
}

void RFM_Set_Tx_Power(int level, int outputPin)
{
    RFM_Write(RFM_REG_PA_CONFIG, 0x80 | level);
    RFM_Write(RFM_REG_PA_DAC, 0x84 | level);
    // if (dBm < 5 || dBm > 20) return PWL_RFM9X_STATUS_BAD;
    // _write_byte(RFM9X_REG_PaConfig, 0x80 | (dBm - 5));
    // _write_byte(RFM9X_REG_PaDac, 0x87);
    // return PWL_RFM9X_STATUS_GOOD;

    // if (RFO_PIN == outputPin)
    // {
    //     // RFO
    //     if (level < 0)
    //     {
    //         level = 0;
    //     }
    //     else if (level > 14)
    //     {
    //         level = 14;
    //     }

    //     RFM_Write(RFM_REG_PA_CONFIG, 0x70 | level);
    // }
    // else
    // {
    //     // PA BOOST
    //     if (level > 17)
    //     {
    //         if (level > 20)
    //         {
    //             level = 20;
    //         }

    //         // subtract 3 from level, so 18 - 20 maps to 15 - 17
    //         level -= 3;

    //         // High Power +20 dBm Operation (Semtech SX1276/77/78/79 5.4.3.)
    //         RFM_Write(RFM_REG_PA_DAC, 0x87);
    //         RFM_Set_OCP(140);
    //     }
    //     else
    //     {
    //         if (level < 2)
    //         {
    //             level = 2;
    //         }
    //         // Default value PA_HF/LF or +17dBm
    //         RFM_Write(RFM_REG_PA_DAC, 0x84);
    //         RFM_Set_OCP(100);
    //     }

    //     RFM_Write(RFM_REG_PA_CONFIG, 0x80 | (level - 2)); // PA Boost mask
    // }
}

// bool RFM_isRxDone()
// {
//     return (RFM_Read(RFM_REG_IRQ_FLAGS & 0x7f) & IRQ_RX_DONE_MASK) != 0;
// }

// void RFM_Set_OCP(uint8_t mA)
// {
//     uint8_t ocpTrim = 27;

//     if (mA <= 120)
//     {
//         ocpTrim = (mA - 45) / 5;
//     }
//     else if (mA <= 240)
//     {
//         ocpTrim = (mA + 30) / 10;
//     }

//     RFM_Write(RFM_REG_OCP, 0x20 | (0x1F & ocpTrim));
// }

/*
*****************************************************************************************
* Description : Function for sending a package with the RFM
*
* Arguments   : *RFM_Tx_Package pointer to buffer with data and counter of data
*               *LoRa_Settings pointer to sSettings struct
*****************************************************************************************
*/

message_t RFM_Send_Package(uint8_t data[], uint8_t len)
{
    unsigned char i;
    unsigned char RFM_Tx_Location = 0x00;

    // Set RFM in Standby mode
    RFM_Switch_Mode(RFM_MODE_STANDBY);

    // Change Datarate
    RFM_Change_SF_BW(12, 0x07);

    // Change Channel
    RFM_Change_Channel(916000000);

    // Set payload length to the right length
    RFM_Write(RFM_REG_PAYLOAD_LENGTH, len);

    // Get location of Tx part of FiFo
    RFM_Tx_Location = RFM_Read(0x0E);

    // Set SPI pointer to start of Tx part in FiFo
    RFM_Write(RFM_REG_FIFO_ADDR_PTR, RFM_Tx_Location);

    // Write Payload to FiFo
    for (i = 0; i < len; i++)
    {
        RFM_Write(RFM_REG_FIFO, data[i]);
    }

    // Switch RFM to Tx
    RFM_Write(RFM_REG_OP_MODE, 0x83);

    // Wait for TxDone
    while ((RFM_Read(RFM_REG_IRQ_FLAGS & 0x7f) & IRQ_TX_DONE_MASK) == 0)
        ;

    // Clear interrupt
    RFM_Write(RFM_REG_IRQ_FLAGS, 0x08);

    return MESSAGE_DONE;
}

/*
*****************************************************************************************
* Description : Function to switch RFM to single receive mode, used for Class A motes
*
* Arguments   : *LoRa_Settings pointer to sSettings struct
*
* Return	  : message_t Status of the received message
*****************************************************************************************
*/
message_t RFM_Single_Receive(void)
{
    message_t Message_Status = NO_MESSAGE;

    RFM_Switch_Mode(RFM_MODE_STANDBY);

    // Change Datarate
    RFM_Change_SF_BW(12, 0x07);

    // Change Channel
    RFM_Change_Channel(916000000);

    // Switch RFM to Single reception
    RFM_Switch_Mode(RFM_MODE_RXSINGLE);

    // Wait until RxDone or Timeout
    // Wait until timeout or RxDone interrupt
    byte RegIrqFlags;

    RegIrqFlags = RFM_Read(RFM_REG_IRQ_FLAGS & 0x7f);
    while ((RegIrqFlags & (IRQ_RX_DONE_MASK | IRQ_RX_TIMEOUT_MASK)) == 0)
    {
        RegIrqFlags = RFM_Read(RFM_REG_IRQ_FLAGS & 0x7f);
    }

    // Check for Timeout
    if ((RegIrqFlags & IRQ_RX_TIMEOUT_MASK) != 0)
    {
        // Clear interrupt register
        RFM_Write(RFM_REG_IRQ_FLAGS, 0xE0);
        Message_Status = TIMEOUT;
        Serial.println("RX Timeout");
    }

    // Check for RxDone
    if ((RegIrqFlags & IRQ_RX_DONE_MASK) != 0)
    {
        Message_Status = NEW_MESSAGE;
        Serial.println("RX Good");
    }

    return Message_Status;
}

/*
*****************************************************************************************
* Description : Function to retrieve a message received by the RFM
*
* Arguments   : *RFM_Rx_Package pointer to sBuffer struct containing the data received
*				and number of bytes received
*
* Return	  : message_t Status of the received message
*****************************************************************************************
*/
message_t RFM_Get_Package(uint8_t data[], uint8_t *len)
{
    unsigned char i;
    unsigned char RFM_Interrupts = 0x00;
    unsigned char RFM_Package_Location = 0x00;
    // message_t Message_Status;

    message_t status = RFM_Single_Receive();

    if (status == NEW_MESSAGE)
    {
        // Get interrupt register
        RFM_Interrupts = RFM_Read(0x12);

        if ((RFM_Interrupts & 0x40))
        {                                        // IRQ_RX_DONE_MASK
            if ((RFM_Interrupts & 0x20) != 0x20) // Check CRC
            {
                status = CRC_OK;
            }
            else
            {
                status = WRONG_MESSAGE;
            }
        }
        RFM_Package_Location = RFM_Read(0x10); /*Read start position of received package*/
        *len = RFM_Read(0x13);                 /*Read length of received package*/

        RFM_Write(RFM_REG_FIFO_ADDR_PTR, RFM_Package_Location); /*Set SPI pointer to start of package*/

        for (i = 0x00; i < *len; i++)
        {
            data[i] = RFM_Read(RFM_REG_FIFO);
        }

        // Clear interrupt register
        RFM_Write(RFM_REG_IRQ_FLAGS, RFM_Interrupts);
    }

    return status;
}
