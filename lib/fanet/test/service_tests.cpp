
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "../include/fanet/fanet.hpp"
#include "etl/vector.h"
#include "helpers.hpp"

using namespace FANET;

TEST_CASE("ServicePayload Default Constructor", "[ServicePayload]")
{
    ServicePayload payload;

    REQUIRE(payload.type() == Header::MessageType::SERVICE);
    REQUIRE(payload.latitude() == 0);
    REQUIRE(payload.longitude() == 0);
    REQUIRE(payload.temperature() == 0);
    REQUIRE(payload.windHeading() == 0);
    REQUIRE(payload.windSpeed() == 0);
    REQUIRE(payload.windGust() == 0);
    REQUIRE(payload.humidity() == 0);
    REQUIRE(payload.barometric() == Catch::Approx(430.f).margin(0.01));
    REQUIRE(payload.hasWind() == false);
    REQUIRE(payload.hasHumidity() == false);
    REQUIRE(payload.hasBarometric() == false);
    REQUIRE(payload.hasTemperature() == false);
}

TEST_CASE("ServicePayload Latitude ", "[single-file]")
{
    ServicePayload payload;
    REQUIRE(payload.latitude() == Catch::Approx(0.0).margin(0.00001));
    payload.latitude(56.95812f);
    REQUIRE(payload.latitude() == Catch::Approx(56.95812f).margin(0.00001));
    payload.latitude(-56.18748);
    REQUIRE(payload.latitude() == Catch::Approx(-56.18748f).margin(0.00001));
    payload.latitude(-91);
    REQUIRE(payload.latitude() == Catch::Approx(-90).margin(0.00001));
    payload.latitude(91);
    REQUIRE(payload.latitude() == Catch::Approx(90).margin(0.00001));
}

TEST_CASE("ServicePayload Longitude ", "[single-file]")
{
    ServicePayload payload;
    REQUIRE(payload.longitude() == Catch::Approx(0.0).margin(0.00002));
    payload.longitude(160.54197);
    REQUIRE(payload.longitude() == Catch::Approx(160.54197).margin(0.00002));
    payload.longitude(-126.74510);
    REQUIRE(payload.longitude() == Catch::Approx(-126.74510).margin(0.00002));
    payload.longitude(-181);
    REQUIRE(payload.longitude() == Catch::Approx(-180).margin(0.00002));
    payload.longitude(181);
    REQUIRE(payload.longitude() == Catch::Approx(180).margin(0.00002));
}

TEST_CASE("ServicePayload windHeading ", "[single-file]")
{
    ServicePayload payload;
    payload.windHeading(123);
    REQUIRE(payload.windHeading() == Catch::Approx(123).margin(01.4));
    payload.windHeading(-123);
    REQUIRE(payload.windHeading() == Catch::Approx(237).margin(01.4));
    payload.windHeading(400);
    REQUIRE(payload.windHeading() == Catch::Approx(40).margin(01.4));
    REQUIRE(payload.hasWind() == true);
}

TEST_CASE("ServicePayload windSpeed ", "[single-file]")
{
    ServicePayload payload;
    payload.windSpeed(12.6);
    REQUIRE(payload.windSpeed() == Catch::Approx(12.6).margin(0.2));
    payload.windSpeed(50.5);
    REQUIRE(payload.windSpeed() == Catch::Approx(50.5).margin(1));
    payload.windSpeed(-10);
    REQUIRE(payload.windSpeed() == Catch::Approx(0).margin(0));
    payload.windSpeed(255);
    REQUIRE(payload.windSpeed() == Catch::Approx(127).margin(1));
    REQUIRE(payload.hasWind() == true);
}

TEST_CASE("ServicePayload windGust ", "[single-file]")
{
    ServicePayload payload;
    payload.windGust(12.6);
    REQUIRE(payload.windGust() == Catch::Approx(12.6).margin(0.2));
    payload.windGust(50.5);
    REQUIRE(payload.windGust() == Catch::Approx(50.5).margin(1));
    payload.windGust(-10);
    REQUIRE(payload.windGust() == Catch::Approx(0).margin(0));
    payload.windSpeed(255);
    REQUIRE(payload.windSpeed() == Catch::Approx(127).margin(1));
    REQUIRE(payload.hasWind() == true);
}

TEST_CASE("ServicePayload temperature ", "[single-file]")
{
    ServicePayload payload;
    payload.temperature(-128);
    REQUIRE(payload.temperature() == Catch::Approx(-64).margin(0.5));
    payload.temperature(128);
    REQUIRE(payload.temperature() == Catch::Approx(63.5).margin(0.5));
    payload.temperature(12.5);
    REQUIRE(payload.temperature() == Catch::Approx(12.5).margin(0.5));
    payload.temperature(-22.5);
    REQUIRE(payload.temperature() == Catch::Approx(-22.5).margin(0.5));
    REQUIRE(payload.hasTemperature() == true);
}

TEST_CASE("ServicePayload Barometric ", "[single-file]")
{
    ServicePayload payload;
    payload.barometric(0);
    REQUIRE(payload.barometric() == Catch::Approx(430).margin(0.01));
    payload.barometric(2000);
    REQUIRE(payload.barometric() == Catch::Approx(1085.36).margin(0.01));
    payload.barometric(1013.01);
    REQUIRE(payload.barometric() == Catch::Approx(1013.01).margin(0.01));
    REQUIRE(payload.hasBarometric() == true);
}

TEST_CASE("ServicePayload humidity ", "[single-file]")
{
    ServicePayload payload;
    payload.humidity(75);
    REQUIRE(payload.humidity() == Catch::Approx(75).margin(0.4));
    payload.humidity(102);
    REQUIRE(payload.humidity() == Catch::Approx(100).margin(0.4));
}

TEST_CASE("ServicePayload serialize/deserialize empty", "[single-file]")
{
    ServicePayload payload;
    auto result = createRadioPacket(payload);
    REQUIRE(result == makeVector({0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }));
}

TEST_CASE("ServicePayload serialize/deserialize altitude", "[single-file]")
{
    ServicePayload payload;
    payload.latitude(57.05812);
    payload.longitude(10.05419);
    payload.windGust(3.5);
    payload.windSpeed(12.6);
    payload.windHeading(123);
    payload.temperature(12.5);
    payload.humidity(75);
    payload.barometric(1013.02);
    auto result = createRadioPacket(payload);
    dumpHex(result);

    REQUIRE(result == makeVector({0x78, 0x0F, 0x26, 0x51, 0x4B, 0x26, 0x07, 0x19, 0x57, 0x3F, 0x12, 0xBC, 0xE3, 0xBE, }));

    auto reader = createReader(result);
    auto received=ServicePayload::deserialize(reader);
    REQUIRE(received.latitude() == Catch::Approx(57.05812).margin(0.00001));
    REQUIRE(received.longitude() == Catch::Approx(10.05419).margin(0.00001));
    REQUIRE(received.windGust() == Catch::Approx(3.5).margin(1));
    REQUIRE(received.windSpeed() == Catch::Approx(12.6).margin(1));
    REQUIRE(received.windHeading() == Catch::Approx(123).margin(1));    
    REQUIRE(received.temperature() == Catch::Approx(12.5).margin(0.5));
    REQUIRE(received.barometric() == Catch::Approx(1013.02).margin(0.01));
    REQUIRE(received.humidity()  == Catch::Approx(75).margin(0.4));
}
