#pragma once

#include <stdint.h>
#include <math.h>
#include "etl/algorithm.h"
#include "header.hpp"

namespace FANET
{
    /**
     * Service payload
     * Messagetype : 4
     */
    class ServicePayload final
    {
    public:
        enum class ServiceType : uint8_t
        {
            WEATHER = 0,  // Weather station
            WINDSOCK = 1, // Windsock
            DEM = 2,      // Digital Elevation Model
            TEMP = 3,     // Temperature
            HUMIDITY = 4, // Humidity
            PRESSURE = 5, // Pressure
            GATEWAY = 6   // Gateway
        };

    private:
        uint8_t header = 0;
        uint8_t eHeader = 0;
        int32_t latitudeRaw = 0;
        int32_t longitudeRaw = 0;
        uint16_t altitudeRaw = 0;
        int8_t temperatureRaw = 0;
        uint8_t windHeadingRaw = 0;
        bool sWindBit = false;
        uint8_t windSpeedRaw = 0;
        bool gWindBit = false;
        uint8_t windGustRaw = 0;
        uint8_t humidityRaw = 0;
        uint16_t barometricRaw = 0;

    public:
        /**
         * @brief Default constructor.
         */
        ServicePayload() = default;

        /**
         * @brief Get the message type.
         * @return The message type.
         */
        Header::MessageType type() const
        {
            return Header::MessageType::SERVICE;
        }

        bool hasGateway() const
        {
            return header & 0x80;
        }

        ServicePayload &setGateway(bool enabled)
        {
            if (enabled)
            {
                header |= 0x80;
            }
            else
            {
                header &= ~0x80;
            }
            return *this;
        }

        bool hasTemperature() const
        {
            return header & 0x40;
        }

        bool hasWind() const
        {
            return header & 0x20;
        }

        bool hasHumidity() const
        {
            return header & 0x10;
        }

        bool hasBarometric() const
        {
            return header & 0x08;
        }

        bool hasExtendedHeader() const
        {
            return header & 0x01;
        }

        /**
         * @brief Get the latitude in degrees.
         * @return The latitude in degrees.
         */
        float latitude() const
        {
            return ((latitudeRaw << 8) >> 8) / 93206.f;
        }

        /**
         * @brief Get the longitude in degrees.
         * @return The longitude in degrees.
         */
        float longitude() const
        {
            return ((longitudeRaw << 8) >> 8) / 46603.f;
        }

        /**
         * @brief Set the latitude in degrees.
         * @param lat The latitude in degrees.
         * @return Reference to the current object.
         */
        ServicePayload &latitude(float lat)
        {
            lat = etl::clamp(lat, -90.f, 90.f);
            latitudeRaw = roundf(lat * 93206.0f);
            return *this;
        }

        /**
         * @brief Set the longitude in degrees.
         * @param lon The longitude in degrees.
         * @return Reference to the current object.
         */
        ServicePayload &longitude(float lon)
        {
            lon = etl::clamp(lon, -180.f, 180.f);
            longitudeRaw = roundf(lon * 46603.0f);
            return *this;
        }

        /**
         * @brief get the temperature in degrees
         * @param The temperature
         */
        float temperature()
        {
            return temperatureRaw / 2.f;
        }

        ServicePayload &temperature(float temperature)
        {
            header |= 0x40;

            temperatureRaw = etl::clamp(static_cast<int>(roundf(temperature * 2.0f)), -128, 127);
            return *this;
        }

        float windHeading() const
        {
            return static_cast<float>(windHeadingRaw) * 360.f / 256.f;
        }

        ServicePayload &windHeading(float windHeading)
        {
            header |= 0x20;

            if (windHeading < 0.0f)
            {
                windHeading += 360.0f;
            }
            else if (windHeading >= 360.0f)
            {
                windHeading -= 360.0f;
            }

            windHeadingRaw = etl::clamp(static_cast<int>(roundf(windHeading * 256.0f / 360.0f)), 0, 255);
            return *this;
        }

        /**
         * @brief Get the speed in kilometers per hour.
         * @return The speed in kilometers per hour.
         */
        float windSpeed() const
        {
            return sWindBit ? windSpeedRaw : windSpeedRaw / 5.0f;
        }

        /**
         * @brief Set the speed in kilometers per hour.
         * @param speed The speed in kilometers per hour.
         * @return Reference to the current object.
         */
        ServicePayload &windSpeed(float speed)
        {
            header |= 0x20;
            int32_t speed2 = etl::clamp(static_cast<int>(roundf(speed * 5)), 0, 127 * 5);
            if (speed2 > 127)
            {
                windSpeedRaw = speed2 / 5;
                sWindBit = true;
            }
            else
            {
                windSpeedRaw = speed2;
            }
            return *this;
        }

        /**
         * @brief Get the gust speed in kilometers per hour.
         * @return The speed in kilometers per hour.
         */
        float windGust() const
        {
            return gWindBit ? windGustRaw : windGustRaw / 5.0f;
        }

        /**
         * @brief Set the wind gust speed in kilometers per hour.
         * @param speed The speed in kilometers per hour.
         * @return Reference to the current object.
         */
        ServicePayload &windGust(float speed)
        {
            header |= 0x20;
            int32_t speed2 = etl::clamp(static_cast<int>(roundf(speed * 5)), 0, 127 * 5);
            if (speed2 > 127)
            {
                windGustRaw = speed2 / 5;
                gWindBit = true;
            }
            else
            {
                windGustRaw = speed2;
            }
            return *this;
        }

        float humidity() const
        {
            return humidityRaw * 0.4f;
        }

        ServicePayload &humidity(float humidity)
        {
            header |= 0x10;
            humidityRaw = etl::clamp(static_cast<int>(roundf(humidity * 2.5f)), 0, 250);
            return *this;
        }

        float barometric() const
        {
            return barometricRaw / 100.0f + 430.0f;
        }

        ServicePayload &barometric(float barometric)
        {
            header |= 0x08;
            barometricRaw = etl::clamp(static_cast<int>(roundf(barometric * 100.0f - 43000.f)), 0, 0xFFFF);
            return *this;
        }

        /**
         * @brief Serialize the service payload to a bit stream.
         * @param writer The bit stream writer.
         */
        void serialize(etl::bit_stream_writer &writer) const
        {
            writer.write_unchecked(header);
            if (hasExtendedHeader())
            {
                writer.write_unchecked(eHeader);
            }

            writer.write_unchecked(etl::reverse_bytes(latitudeRaw << 8), 24U);
            writer.write_unchecked(etl::reverse_bytes(longitudeRaw << 8), 24U);

            if (hasTemperature())
            {
                writer.write_unchecked(temperatureRaw);
            }

            if (hasWind())
            {
                writer.write_unchecked(windHeadingRaw, 8U);
                writer.write_unchecked(sWindBit);
                writer.write_unchecked(windSpeedRaw, 7U);
                writer.write_unchecked(gWindBit);
                writer.write_unchecked(windGustRaw, 7U);
            }

            if (hasHumidity())
            {
                writer.write_unchecked(humidityRaw);
            }

            if (hasBarometric())
            {
                writer.write_unchecked<uint16_t>(barometricRaw);
            }
        }

        /**
         * @brief Deserialize the service payload from a bit stream.
         * @param reader The bit stream reader.
         * @return The deserialized service payload.
         */
        static ServicePayload deserialize(etl::bit_stream_reader &reader)
        {
            ServicePayload service;

            service.header = reader.read_unchecked<uint8_t>();
            if (service.hasExtendedHeader())
            {
                service.eHeader = reader.read_unchecked<uint8_t>();
            }

            service.latitudeRaw = etl::reverse_bytes(reader.read_unchecked<uint32_t>(24U)) >> 8;
            service.longitudeRaw = etl::reverse_bytes(reader.read_unchecked<uint32_t>(24U)) >> 8;

            if (service.hasTemperature())
            {
                service.temperatureRaw = reader.read_unchecked<int8_t>();
            }

            if (service.hasWind())
            {
                service.windHeadingRaw = reader.read_unchecked<uint8_t>();
                service.sWindBit = reader.read_unchecked<bool>();
                service.windSpeedRaw = reader.read_unchecked<uint8_t>(7U);
                service.gWindBit = reader.read_unchecked<bool>();
                service.windGustRaw = reader.read_unchecked<uint8_t>(7U);
            }

            if (service.hasHumidity())
            {
                service.humidityRaw = reader.read_unchecked<uint8_t>();
            }

            if (service.hasBarometric())
            {
                service.barometricRaw = reader.read_unchecked<uint16_t>();
            }

            return service;
        }
    };

}