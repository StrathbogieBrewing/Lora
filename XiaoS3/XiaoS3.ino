#include <RadioLib.h>
#include <WiFiManager.h>

#define LORA_DIO_1 39
#define LORA_NSS 41
#define LORA_RESET 42
#define LORA_BUSY 40
#define LORA_SCLK 7
#define LORA_MISO 8
#define LORA_MOSI 9

// static SPIClass spi;
SX1262 radio = new Module(LORA_NSS, LORA_DIO_1, LORA_RESET, LORA_BUSY);


WiFiManager wm;
// flag to indicate that a packet was received
volatile bool receivedFlag = false;

// this function is called when a complete packet
// is received by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
#if defined(ESP8266) || defined(ESP32)
ICACHE_RAM_ATTR
#endif
void setFlag(void) {
    // we got a packet, set the flag
    receivedFlag = true;
}

void hexdump(uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        Serial.printf("%02X ", data[i]);
    }
    Serial.println();
}

void setup() {
    WiFi.mode(WIFI_STA);
    Serial.begin(115200);
    SPI.begin(LORA_SCLK, LORA_MISO, LORA_MOSI);
    

    bool res = wm.autoConnect("AutoConnectAP"); 
    if (!res) {
        Serial.println("Failed to connect");
        ESP.restart();
    }

    // initialize SX1262 at 434 MHz
    Serial.print(F("[SX1262] Initializing ... "));
    // ConfigLoRa_t config;
    // config.frequency = 434;
    int state =
        radio.begin(916.0, 125.0, 10, 8, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 14,
                    8, 3.0, true);
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println(F("success!"));
    } else {
        Serial.print(F("failed, code "));
        Serial.println(state);
        while (true) {
            delay(10);
        }
    }

    // set the function that will be called
    // when new packet is received
    radio.setPacketReceivedAction(setFlag);

    // start listening for LoRa packets
    Serial.print(F("[SX1262] Starting to listen ... "));
    state = radio.startReceive();
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println(F("success!"));
    } else {
        Serial.print(F("failed, code "));
        Serial.println(state);
        while (true) {
            delay(10);
        }
    }

    // if needed, 'listen' mode can be disabled by calling
    // any of the following methods:
    //
    // radio.standby()
    // radio.sleep()
    // radio.transmit();
    // radio.receive();
    // radio.scanChannel();
}

void loop() {
    // check if the flag is set
    if (receivedFlag) {
        // reset flag
        receivedFlag = false;

        // you can read received data as an Arduino String
        // String str;
        // int state = radio.readData(str);

        // you can also read received data as byte array

        byte byteArr[256];
        int numBytes = radio.getPacketLength();
        int state = radio.readData(byteArr, numBytes);

        if (state == RADIOLIB_ERR_NONE) {
            // packet was successfully received
            Serial.println(F("[SX1262] Received packet!"));

            // print data of the packet
            Serial.print(F("[SX1262] Data:\t\t"));
            // Serial.println(str);
            hexdump(byteArr, numBytes);

            // print RSSI (Received Signal Strength Indicator)
            Serial.print(F("[SX1262] RSSI:\t\t"));
            Serial.print(radio.getRSSI());
            Serial.println(F(" dBm"));

            // print SNR (Signal-to-Noise Ratio)
            Serial.print(F("[SX1262] SNR:\t\t"));
            Serial.print(radio.getSNR());
            Serial.println(F(" dB"));

            // print frequency error
            Serial.print(F("[SX1262] Frequency error:\t"));
            Serial.print(radio.getFrequencyError());
            Serial.println(F(" Hz"));

        } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
            // packet was received, but is malformed
            Serial.println(F("CRC error!"));

        } else {
            // some other error occurred
            Serial.print(F("failed, code "));
            Serial.println(state);
        }
    }
}
