#include <Arduino.h>
#include <RadioLib.h>
#include <fanet/fanet.hpp>
#include <fanet/service.hpp>
#include <fanet/txFrame.hpp>
#include <etl/bit_stream.h>

// Define pins
#define LED_RX PA8
#define LED_TX PA9

// GPS Coordinates
const float latitude = -40.9747;  // New Zealand
const float longitude = 174.6339;

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

    // Auto frequency/bandwidth selection based on lat/lon
    float freq = 868.2;
    float bw = 250.0;
    uint8_t cr = 5;
    int8_t power = 14;

    if (-169.0f < longitude && longitude < -30.0f) {
        freq = 920.8; bw = 500.0; power = 15;
        Serial.println("🌍 Region: US920");
    } else if (110.0f < longitude && longitude < 179.0f && -48.0f < latitude && latitude < -10.0f) {
        freq = 920.8; bw = 500.0; power = 18;
        Serial.println("🌍 Region: AU920");
    } else if (69.0f < longitude && longitude < 89.0f && 5.0f < latitude && latitude < 40.0f) {
        freq = 866.2; bw = 250.0; power = 14;
        Serial.println("🌍 Region: IN866");
    } else if (124.0f < longitude && longitude < 130.0f && 34.0f < latitude && latitude < 39.0f) {
        freq = 923.2; bw = 125.0; power = 15;
        Serial.println("🌍 Region: KR923");
    } else if (89.0f < longitude && longitude < 146.0f && 21.0f < latitude && latitude < 47.0f) {
        freq = 923.2; bw = 125.0; power = 15;
        Serial.println("🌍 Region: AS920");
    } else if (34.0f < longitude && longitude < 36.0f && 29.0f < latitude && latitude < 34.0f) {
        freq = 918.5; bw = 125.0; power = 15;
        Serial.println("🌍 Region: IL918");
    } else {
        freq = 868.2; bw = 250.0; power = 14;
        Serial.println("🌍 Region: EU868 (default)");
    }

    int state = radio.begin(freq, bw, 7, cr, 0xF1, power, 10); // freq, bw, SF, CR, syncword, power, preamble

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
        payload.latitude(latitude)
               .longitude(longitude)
               .temperature(20.0f)
               .windHeading(270.0f)
               .windSpeed(10.0f)
               .windGust(15.0f)
               .humidity(65.0f)
               .barometric(1013.0f)
               .battery(50.0f);

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
        Serial.print("  Battery: "); Serial.println(payload.battery());

        digitalWrite(LED_TX, LOW);
        protocol.sendPacket(packet, 0); // Queue and schedule for transmission
        protocol.handleTx();           // Let the protocol handle sending
        digitalWrite(LED_TX, HIGH);

        lastSent = millis();
    }
}
