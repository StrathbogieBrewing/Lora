/*
  RadioLib SX126x Blocking Receive Example

  This example listens for LoRa transmissions using SX126x Lora modules.
  To successfully receive data, the following settings have to be the same
  on both transmitter and receiver:
  - carrier frequency
  - bandwidth
  - spreading factor
  - coding rate
  - sync word
  - preamble length

  Other modules from SX126x family can also be used.

Using blocking receive is not recommended, as it will lead
to significant amount of timeouts, inefficient use of processor
time and can some miss packets!
Instead, interrupt receive is recommended.

  For default module settings, see the wiki page
  https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx126x---lora-modem

  For full API reference, see the GitHub Pages
  https://jgromes.github.io/RadioLib/
*/

// include the library
#include <RadioLib.h>

#define LED_POWER 48

#define LORA_DIO_1 39
#define LORA_NSS 41
#define LORA_RESET 42
#define LORA_BUSY 40
#define LORA_SCLK 7
#define LORA_MISO 8
#define LORA_MOSI 9

// SX1262 has the following connections:
// NSS pin:   10
// DIO1 pin:  2
// NRST pin:  3
// BUSY pin:  9
// SX1262 radio = new Module(10, 2, 3, 9);

// static SPIClass spi;
SX1262 radio =
    new Module(LORA_NSS, LORA_DIO_1, LORA_RESET, LORA_BUSY); // spi);



// or detect the pinout automatically using RadioBoards
// https://github.com/radiolib-org/RadioBoards
/*
#define RADIO_BOARD_AUTO
#include <RadioBoards.h>
Radio radio = new RadioModule();
*/



void hd(uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        Serial.printf("%02X ", data[i]);
    }
    Serial.println();
}

void setup() {
    pinMode(LED_POWER, OUTPUT);
    Serial.begin(115200);

    SPI.begin(LORA_SCLK, LORA_MISO, LORA_MOSI);
    // radio.std_init(&spi);

    // initialize SX1262 at 916 MHz
    // while(1){
    //   digitalWrite(LED_POWER, HIGH);
    //   delay(1000);
    //     digitalWrite(LED_POWER, LOW);
    //   delay(1000);
    //   Serial.println(("Hello"));
    // }
    Serial.print(F("[SX1262] Initializing ... "));
    // ConfigLoRa_t config;
    // config.frequency = 916.0;
    // config.bandwidth = 125.0;
    // config.spreadingFactor = 10;
    // config.codingRate = 8;
    // config.syncWord = RADIOLIB_SX126X_SYNC_WORD_PRIVATE;
    // config.power = 14;
    // config.preambleLength = 8;
    // int state = radio.begin(config);
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
}

void loop() {
    Serial.print(F("[SX1262] Waiting for incoming transmission ... "));

    // you can receive data as an Arduino String
    // String str;
    // int state = radio.receive(str);

    // you can also receive data as byte array

    byte byteArr[16];
    int state = radio.receive(byteArr, 16);

    if (state == RADIOLIB_ERR_NONE) {
        // packet was successfully received
        Serial.println(F("success!"));

        // print the data of the packet
        Serial.print(F("[SX1262] Data:\t\t"));
        // Serial.println(byteArr);
        hd(byteArr, 16);

        // print the RSSI (Received Signal Strength Indicator)
        // of the last received packet
        Serial.print(F("[SX1262] RSSI:\t\t"));
        Serial.print(radio.getRSSI());
        Serial.println(F(" dBm"));

        // print the SNR (Signal-to-Noise Ratio)
        // of the last received packet
        Serial.print(F("[SX1262] SNR:\t\t"));
        Serial.print(radio.getSNR());
        Serial.println(F(" dB"));

        // print frequency error
        // Serial.print(F("[SX1262] Frequency error:\t"));
        // Serial.print(radio.getFrequencyError());
        // Serial.println(F(" Hz"));

    } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
        // timeout occurred while waiting for a packet
        Serial.println(F("timeout!"));

    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
        // packet was received, but is malformed
        Serial.println(F("CRC error!"));

    } else {
        // some other error occurred
        Serial.print(F("failed, code "));
        Serial.println(state);
    }
}
