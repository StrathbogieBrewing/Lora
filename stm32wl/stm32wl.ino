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

void setup() {
    Serial.begin(9600);

    radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);
    radio.begin(916.0);
    radio.setBandwidth(125.0);
    radio.setSpreadingFactor(7);
    radio.setCodingRate(RADIOLIB_SX126X_LORA_CR_4_5);
    radio.setOutputPower(10);
    radio.setTCXO(1.7);

    pinMode(LED, OUTPUT);
    digitalWrite(LED, LOW);
}

void loop() {
    static int count = 0;

    // you can transmit C-string or Arduino string up to
    // 256 characters long
    String str = "STM: #" + String(count++);
    digitalWrite(LED, LOW);
    radio.transmit(str);

    // you can also transmit byte array up to 256 bytes long
    /*
      byte byteArr[] = {0x01, 0x23, 0x45, 0x56, 0x78, 0xAB, 0xCD, 0xEF};
      int state = radio.transmit(byteArr, 8);
    */
    digitalWrite(LED, HIGH);
    delay(5000);
}
