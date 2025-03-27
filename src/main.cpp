#include <Arduino.h>
#include <RadioLib.h>
#include <fanet/fanet.hpp>
//#include <fanet/service.hpp>
//#include <fanet/txFrame.hpp>
#include <etl/bit_stream.h>

// Define LED pins
#define LED_RX PA8
#define LED_TX PA9

// Setup STM32WL internal LoRa radio
STM32WLx_Module stm32wl;
STM32WLx radio(&stm32wl);

// RF switch configuration (based on working example)
static const uint32_t rfswitch_pins[5] = {PB8, PC13, RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC};
static const Module::RfSwitchMode_t rfswitch_table[] = {
    {STM32WLx::MODE_IDLE,    {LOW, LOW}},
    {STM32WLx::MODE_RX,      {HIGH, LOW}},
    {STM32WLx::MODE_TX_HP,   {LOW, HIGH}},
    END_OF_MODE_TABLE
};

// Implement the FANET Connector interface
class MyConnector : public FANET::Connector {
public:
    explicit MyConnector(STM32WLx* radioRef) : radio(radioRef) {}

    bool fanet_sendFrame(uint8_t /* codingRate */, etl::span<const uint8_t> data) override {
        Serial.print("\U0001F4E1 Transmitting frame, length: ");
        Serial.print(data.size());
        Serial.println(" bytes");
        Serial.print("  Data: ");
        for (size_t i = 0; i < data.size(); i++) {
            Serial.print("0x");
            if (data[i] < 0x10) Serial.print("0");
            Serial.print(data[i], HEX);
            Serial.print(" ");
        }
        Serial.println();

        int16_t result = radio->transmit(data.data(), data.size());

        if (result == RADIOLIB_ERR_NONE) {
            Serial.println("✅ Transmission successful");
            return true;
        } else {
            Serial.print("❌ Transmission failed, error code: ");
            Serial.println(result);
            return false;
        }
    }

    uint32_t fanet_getTick() const override {
        return millis();
    }

    void fanet_ackReceived(uint16_t id) override {
        Serial.print("✅ ACK received for ID: ");
        Serial.println(id);
    }

private:
    STM32WLx* radio;
};

// FANET protocol and address
MyConnector connector(&radio);
FANET::Protocol protocol(&connector);
FANET::Address myAddress(0x01, 0x1234);

void setup() {
    Serial.begin(115200);
    delay(500);

    pinMode(LED_RX, OUTPUT);
    pinMode(LED_TX, OUTPUT);
    digitalWrite(LED_RX, HIGH);
    digitalWrite(LED_TX, HIGH);

    pinMode(PB8, OUTPUT);
    pinMode(PC13, OUTPUT);
    digitalWrite(PB8, LOW);
    digitalWrite(PC13, LOW);

    stm32wl.setRfSwitchTable(rfswitch_pins, rfswitch_table);

    int state = radio.begin(920.8, 500.0, 7, 5, 0xF1, 22, 10); // freq, bw, SF, CR, syncword, power, preamble

    if (state != RADIOLIB_ERR_NONE) {
        Serial.println("❌ Radio initialization failed!");
        while (true) {
            digitalWrite(LED_TX, LOW);
            delay(100);
            digitalWrite(LED_TX, HIGH);
            delay(100);
        }
    }

    Serial.println("✅ Radio initialized");
    Serial.println("FANET Address initialized.");
    Serial.print("  Manufacturer ID: 0x");
    Serial.println(myAddress.manufacturer(), HEX);
    Serial.print("  Unique Device ID: 0x");
    Serial.println(myAddress.unique(), HEX);

    protocol.ownAddress(myAddress);
}

void loop() {
    static uint32_t lastSent = 0;

    if (millis() - lastSent > 5000) {
        FANET::ServicePayload payload;
        payload.latitude(47.397742)
               .longitude(8.545594)
               .temperature(20.0)
               .windHeading(270.0)
               .windSpeed(10.0)
               .windGust(15.0)
               .humidity(65.0)
               .barometric(1013.0)
               .battery(27.0);


        auto packet = FANET::Packet<1>()
                        .payload(payload)
                        .forward(true);

        Serial.println("📦 Sending ServicePayload packet:");
        Serial.print("  Latitude: "); Serial.println(payload.latitude(), 6);
        Serial.print("  Longitude: "); Serial.println(payload.longitude(), 6);
        Serial.print("  Temperature: "); Serial.println(payload.temperature());
        Serial.print("  Wind Heading: "); Serial.println(payload.windHeading());
        Serial.print("  Wind Speed: "); Serial.println(payload.windSpeed());
        Serial.print("  Wind Gust: "); Serial.println(payload.windGust());
        Serial.print("  Humidity: "); Serial.println(payload.humidity());
        Serial.print("  Pressure: "); Serial.println(payload.barometric());

        digitalWrite(LED_TX, LOW);
        protocol.sendPacket(packet, 0); // Queue and schedule for transmission
        
        digitalWrite(LED_TX, HIGH);

        lastSent = millis();
    }
    protocol.handleTx();           // Let the protocol handle sending
}
