#include "STM32LowPower.h"
#include <RadioLib.h>

#define LED LED_BUILTIN

#include <NewPing.h>

#define TRIGGER_PIN                                                            \
    9               // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN 10 // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE                                                           \
    400 // Maximum distance we want to ping for (in centimeters). Maximum sensor
        // distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN,
              MAX_DISTANCE); // NewPing setup of pins and maximum distance.

// #include <HCSR04.h>

// HCSR04 hc(9, 10); //initialisation class HCSR04 (trig pin , echo pin)

STM32WLx radio = new STM32WLx_Module();

HardwareSerial mySerial(USART2);

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

    LowPower.begin();

    radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);
    radio.begin(916.0);
    radio.setBandwidth(250.0);
    radio.setSpreadingFactor(10);

    radio.setCodingRate(RADIOLIB_SX126X_LORA_CR_4_8);
    radio.setOutputPower(14);
    radio.setTCXO(1.7);

    pinMode(LED, OUTPUT);
    digitalWrite(LED, LOW);

    // pinMode(0, OUTPUT);
    // digitalWrite(0, LOW);
}

#define DEVICE_ID 99

uint32_t getDistance_mm(void) {
    mySerial.begin(9600);
    mySerial.write(0x01);
    delay(50);
    if (mySerial.available()) {
        uint32_t distance_mm;
        uint8_t startByte, h_data, l_data, sum = 0;
        uint8_t buf[3];

        startByte = (byte)mySerial.read();
        if (startByte == 255) {
            mySerial.readBytes(buf, 3);
            h_data = buf[0];
            l_data = buf[1];
            sum = buf[2];
            distance_mm = (h_data << 8) + l_data;
            if (((h_data + l_data - 1) & 0xFF) == sum) {
                return distance_mm;
            } else {
                return 0;
            }
        }
    }
    return 0;
}

void loop() {
    static uint8_t message_count = 0;

    digitalWrite(LED, LOW);
    // digitalWrite(0, HIGH);
    // Serial.begin(115200);
    // Serial.println("Loop");

    // Serial1.begin(9600);
    // Serial1.println("Loop");

    // delay(1000);

    uint8_t buffer[8] = {0};
    buffer[0] = DEVICE_ID;
    buffer[1] = message_count++;

    uint32_t distance_cm = getDistance_mm() / 10; // sonar.ping_cm();
    buffer[2] = distance_cm >> 8;
    buffer[3] = distance_cm >> 0;

    radio.transmit(buffer, 4);

    digitalWrite(LED, HIGH);
    // digitalWrite(0, LOW);

    radio.sleep(true);
    LowPower.deepSleep(5000);
    radio.standby();
}
