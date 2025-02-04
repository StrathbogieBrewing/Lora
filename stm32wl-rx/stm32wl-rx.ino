#include <RadioLib.h>

#define LED LED_BUILTIN

STM32WLx radio = new STM32WLx_Module();

static const uint32_t rfswitch_pins[] = {PA4, PA5, RADIOLIB_NC, RADIOLIB_NC,
                                         RADIOLIB_NC};
static const Module::RfSwitchMode_t rfswitch_table[] = {
    {STM32WLx::MODE_IDLE, {LOW, LOW}},
    {STM32WLx::MODE_RX, {HIGH, LOW}},
    // {STM32WLx::MODE_TX_HP, {LOW, HIGH}},  // for LoRa-E5 mini
    {STM32WLx::MODE_TX_LP, {HIGH, HIGH}}, // for LoRa-E5-LE mini
    END_OF_MODE_TABLE,
};

// flag to indicate that a packet was received
volatile bool receivedFlag = false;

// this function is called when a complete packet
// is received by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
void setFlag(void) {
    // we got a packet, set the flag
    receivedFlag = true;
}

void setup() {
    Serial.begin(115200);

    radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);
    radio.begin(916.0);
    radio.setBandwidth(250.0);
    radio.setSpreadingFactor(10);

    radio.setCodingRate(RADIOLIB_SX126X_LORA_CR_4_8);
    radio.setOutputPower(14);
    radio.setTCXO(1.7);

    // set the function that will be called
    // when new packet is received
    radio.setDio1Action(setFlag);

    // start listening for LoRa packets
    Serial.print(F("[STM32WL] Starting to listen ... "));
    int state = radio.startReceive();
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println(F("success!"));
    } else {
        Serial.print(F("failed, code "));
        Serial.println(state);
        while (true) {
            delay(10);
        }
    }

    pinMode(LED, OUTPUT);
    digitalWrite(LED, LOW);
}

void loop() {
    // static int count = 0;

    // // you can transmit C-string or Arduino string up to
    // // 256 characters long
    // String str = "STM: #" + String(count++);
    // digitalWrite(LED, LOW);
    // radio.transmit(str);

    // // you can also transmit byte array up to 256 bytes long
    // /*
    //   byte byteArr[] = {0x01, 0x23, 0x45, 0x56, 0x78, 0xAB, 0xCD, 0xEF};
    //   int state = radio.transmit(byteArr, 8);
    // */
    // digitalWrite(LED, HIGH);
    // delay(5000);
    // check if the flag is set
    if (receivedFlag) {
        // reset flag
        receivedFlag = false;

        byte byteArr[16];
        int numBytes = radio.getPacketLength();
        int state = radio.readData(byteArr, numBytes);

        if (state == RADIOLIB_ERR_NONE) {

            // // print the data of the packet
            // Serial.print(F("[STM32WL] Data:\t\t"));
            // Serial.print(str);

            // print the RSSI (Received Signal Strength Indicator)
            // of the last received packet
            Serial.print(F("{\"rssi\": "));
            Serial.print(radio.getRSSI());
            // Serial.println(F(" dBm"));

            // print the SNR (Signal-to-Noise Ratio)
            // of the last received packet
            Serial.print(F(", \"snr\": "));
            Serial.print(radio.getSNR());
            Serial.print(F(", \"data\": ["));
            for (int i = 0; i < numBytes; i++) {
                if (i) {
                    Serial.print(", ");
                }
                Serial.print(byteArr[i]);
            }

            Serial.println(F("]}"));

            digitalWrite(LED, LOW);
            delay(50);
            digitalWrite(LED, HIGH);

        } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
            // timeout occurred while waiting for a packet
            Serial.println(F("{\"error\": \"timeout\"}"));
        } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
            // packet was received, but is malformed
            Serial.println(F("{\"error\": \"bad crc\"}"));
        } else {
            // some other error occurred
            Serial.print(F("{\"error\": "));
            Serial.print(state);
            Serial.println(F("}"));
        }
    }
}
