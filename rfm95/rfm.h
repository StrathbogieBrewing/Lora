#ifndef RFM_H
#define RFM_H

#include <stdint.h>

typedef enum
{
    NO_MESSAGE,
    NEW_MESSAGE,
    CRC_OK,
    MIC_OK,
    ADDRESS_OK,
    MESSAGE_DONE,
    TIMEOUT,
    WRONG_MESSAGE
} message_t;

#define PA_OUTPUT_RFO_PIN 0
#define PA_OUTPUT_PA_BOOST_PIN 1

bool RFM_Init(uint8_t RFM_MISO,
             uint8_t RFM_MOSI,
             uint8_t RFM_SCLK,
             uint8_t RFM_NSEL,
             uint8_t RFM_NRST);

// void RFM_Send_Package(sBuffer *RFM_Tx_Package, sSettings *LoRa_Settings);
message_t RFM_Send_Package(uint8_t data[], uint8_t len);

// message_t RFM_Single_Receive(sSettings *LoRa_Settings);
// message_t RFM_Single_Receive(void);

// void RFM_Continuous_Receive(sSettings *LoRa_Settings);
// message_t RFM_Get_Package(sBuffer *RFM_Rx_Package);
message_t RFM_Get_Package(uint8_t data[], uint8_t *len);


// void RFM_Write(unsigned char RFM_Address, unsigned char RFM_Data);
// void RFM_Switch_Mode(unsigned char Mode);
// void RFM_Set_Tx_Power(int level, int outputPin);
// bool RFM_isRxDone();
// void RFM_Set_OCP(unsigned char mA);

int16_t RFM_Get_Rssi();

#endif
