#pragma once

#include <stdint.h>
#include "etl/string.h"
#include "etl/vector.h"
#include "header.hpp"

namespace FANET
{
    struct ZoneRegion
    {
        const etl::string_view name;
        struct sx_region_t
        {
            const float channel;
            const int16_t dBm;
            const uint16_t bw;
        } mac;
        const int16_t lat1; // Latitude 1
        const int16_t lat2; // Latitude 2
        const int16_t lon1; // Longitude 1
        const int16_t lon2; // Longitude 2
    };

    static constexpr auto DEFAULT_ZONE = ZoneRegion{"UNK", {0, -127, 0}, 0, 0, 0, 0};

    static const auto DEFAULT_ZONES = etl::make_vector(
        ZoneRegion{"US920", {920800, 15, 500}, 90, -90, -30, -169},
        ZoneRegion{"AU920", {920800, 15, 500}, -10, -48, 179, 110},
        ZoneRegion{"IN866", {868200, 14, 250}, 40, 5, 89, 69},
        ZoneRegion{"KR923", {923200, 15, 125}, 39, 34, 130, 124},
        ZoneRegion{"AS920", {923200, 15, 125}, 47, 21, 146, 89},
        ZoneRegion{"IL918", {918500, 15, 125}, 34, 29, 36, 34},
        ZoneRegion{"EU868", {868200, 14, 250}, 90, -90, 180, -180}, // Functions as a catch all with valid lat/lon coordinates
        DEFAULT_ZONE
    );

    /*
     * The Zone class provides a mechanism to manage and identify regions based on geographic coordinates.
     * It allows the user to either use a default set of predefined zones or provide custom zones.
     * The `findZone` method determines the appropriate zone for given latitude and longitude coordinates,
     * returning a default zone if no match is found.
     */
    class Zone final
    {
    public:
        const etl::span<const ZoneRegion> zones;

        // Default constructor uses default zones
        Zone() : zones(DEFAULT_ZONES) {}

        // Constructor with custom zones
        // When you create your own custom zones you must end it with FANET::DEFAULT_ZONE
        Zone(const etl::span<ZoneRegion> &custom_zones) : zones(custom_zones) {}

        /**
         * @brief Based on latitude and longitude find the current zone. When teh zone is known,
         *
         * the returned data can be used to setup the transceiver with the correct bandwith, frequency and maximum power
         * The coding rate changes based on the neighbors and will be given per package to be send
         *
         * @param lon The longitude in degrees.
         * @param lon The longitude in degrees.
         * @return zone, when not known DEFAULT_ZONE will be returned which should be the last item in the list
         */
        const ZoneRegion &findZone(float latitude, float longitude)
        {
            int16_t lat = static_cast<int16_t>(latitude);
            int16_t lon = static_cast<int16_t>(longitude);
            for (const auto &zone : zones)
            {
                if (lat >= zone.lat2 && lat <= zone.lat1 && lon >= zone.lon2 && lon <= zone.lon1)
                {
                    return zone;
                }
            }
            return zones.back(); // Default to the last zone if no match is found
        }
    };
}
