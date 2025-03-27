
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "../include/fanet/zone.hpp"
#include "etl/vector.h"

using namespace FANET;

TEST_CASE("Test Zone", "[Zone]") {
    Zone zone;

    SECTION("EU868") {
        REQUIRE(zone.findZone(52.0f, 4.0).name == "EU868");
    }

    SECTION("AU920") {
        REQUIRE(zone.findZone(-42.0f, 173.0).name == "AU920");
    }

    SECTION("UNK") {
        REQUIRE(zone.findZone(91, 0).name == "UNK");
    }

}
