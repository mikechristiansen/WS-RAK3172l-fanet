# FANET Library

FANET (Flying Ad-hoc Network) is a lightweight, embedded networking well tested library designed for aerial vehicles and ground stations. It provides a robust protocol for communication between flying objects and ground stations.

This library is a c++ implementation header only *no malloc* version designed to handle the full FANET protocol for embedded systems.

The protocol was taking from https://github.com/3s1d/fanet-stm32 and a library was created without the radio integration, this makes
it easer to pull this into your codebase.

R. van Twisk <github@rvt.dds.nl>

To run the tests you can use `./build.sh` However, since this is a header only library you can just include them in your project (use PlatformIO!) 

## Quick Start of the required components:

```cpp
#include "fanet/fanet.hpp"

// Create a connector implementation
class MyConnector : public FANET::Connector {
    private:
        FANET::Protocol protocol;
        Radio &myRadio; // Some form of implementation of your radio

        /**
         * @brief return a number of ms. Does not matter if it's from epoch, as long as it's monotonic.
         */
        uint32_t fanet_getTick() const override
        {
            return millis();
        }

        /**
         * @brief send a frame to the network
         * @param codingRate the coding rate to use
         * @param data the data to send
         * @return true if the frame was sent successfully
         */
        bool fanet_sendFrame(uint8_t codingRate, etl::span<const uint8_t> data) override
        {
            (void)data;
            auto sendOk = myRadio.sendData(codingRate, data)
            return sendOk;
        }

        /**
         * @brief called when an ack is received of any frames you send directory to an destination 
         * excluding tracking packets like Ground and Tracking. They are handled special.
         * @param id the id of the ack
         */
        void fanet_ackReceived(uint16_t id) override
        {
            printf("fanet_ackReceived %d\n", id);
        }

    public:
        MyConnector(Radio myRadio_) : protocol(this), myRadio(myRadio)
        {
            protocol.ownAddress(FANET::Address(0x01, 0x1234));
        }
};

// Initialize the protocol
// aRadio is some form of implementation of a radio system.
MyConnector connector(aRadio);

## Core Components

### Protocol (`protocol.hpp`)
The main protocol handler that manages packet transmission and reception. It handles:
- Packet forwarding
- Acknowledgments
- Neighbor discovery
- Transmission scheduling

```cpp

// Example: Sending a tracking packet
FANET::TrackingPayload payload;
payload.latitude(ownshipPosition.lat)
    .longitude(ownshipPosition.lon)
    .altitude(ownshipPosition.heightEgm96)
    .speed(ownshipPosition.groundSpeed * MS_TO_KPH)
    .groundTrack(ownshipPosition.course)
    .climbRate(ownshipPosition.verticalSpeed)
    .tracking(true)
    .turnRate(ownshipPosition.hTurnRate)
    .aircraftType(mapAircraftCategory(openAceConfiguration.category));

auto packet = FANET::Packet<1>() // Package size set to 1, not repoevant for TrackingPayload (this is on my TODO to improve)
                    .payload(payload)
                    .forward(true);

protocol.sendPacket(packet, 0); 


FANET::NamePayload<20> namePayload;
namePayload.name("OpenAce");
auto packet = FANET::Packet<20>() // 20 bytes is fine to store OpenAce.
                    .payload(namePayload)
                    .destination(FANET::Address{0x08158C})
                    .singleHop()
                    .forward(true);

protocol.sendPacket(packet, 12); // When received by 0x08158C and ack is send back, fanet_ackReceived will be called with the id of 12

```

> [!NOTE] You are responsible for the timings of when to send packages. The connector will handle the scheduling and will keep track of airtime, eg if you are allowed to send the package. 


### Address (`address.hpp`)
Represents a FANET device address consisting of:
- Manufacturer ID (8 bits)
- Unique Device ID (16 bits)

```cpp
// Example
auto address = FANET::Address addr(0x01, 0x1234);  // Mfg: 0x01, Device: 0x1234
```

### Packet Types

#### Tracking (`tracking.hpp`)
Used for real-time position reporting of aircraft:
- Position (lat/lon)
- Altitude 
- Speed
- Aircraft type
- Climb rate
- Ground track
- Turn rate

> [!NOTE]  
> altitude is in meters EGM96, check your GPS what it's sending you! 

```cpp
// Example
FANET::TrackingPayload tracking;
tracking.latitude(47.123)
        .longitude(8.456)
        .altitude(1000)
        .speed(30.5)
        .aircraftType(FANET::TrackingPayload::AircraftType::PARAGLIDER);
```

#### Ground Tracking (`groundTracking.hpp`)
Used for ground-based objects:
- Position
- Type (walking, vehicle, bike, etc.)
- Support status (need help, technical support, etc.)

```cpp
// Example
FANET::GroundTrackingPayload ground;
ground.latitude(47.123)
      .longitude(8.456)
      .groundType(FANET::GroundTrackingPayload::TrackingType::WALKING);
```

#### Message (`message.hpp`)
General-purpose message transmission:
- Text messages
- Custom data payloads
- Configurable size

```cpp
FANET::MessagePayload<100> msg;  // 100-byte message capacity
etl::vector<uint8_t, 100> message;
// Fill message
msg.message(message);
```

#### Name (`name.hpp`)
Device identification payload:
- Device name/identifier
- Configurable size

```cpp
FANET::NamePayload<20> name;  // 20-char name capacity
name.name("OpenACE");
```

### Supporting Classes

#### NeighbourTable (`neighbourTable.hpp`)
Maintains a list of nearby FANET devices:
- Address tracking
- Last seen timestamps
- Automatic cleanup

#### Zone (`zone.hpp`)
Manages regional settings:
- Frequency bands
- Power limits
- Geographic boundaries

```cpp
FANET::Zone zone;
auto region = zone.findZone(47.123, 8.456);  // Get region settings for coordinates
```

#### BlockAllocator (`blockAllocator.hpp`)
Memory management for packet transmission:
- Fixed-size memory pool
- Efficient allocation/deallocation
- Prevents fragmentation

Used internally

## Advanced Features

### Packet Forwarding
FANET supports multi-hop packet forwarding:
```cpp
auto packet = FANET::Packet<1>()
    .source(myAddress)
    .destination(targetAddress)
    .payload(payload)
    .forward(true)    // Enable forwarding
    .twoHop();       // Request 2-hop acknowledgment
```

### Acknowledgments
Three acknowledgment modes:
- None: No acknowledgment
- Single-hop: Direct acknowledgment
- Two-hop: Forwarded acknowledgment

> [!NOTE] Acknowledgments only happen for messages that are not tracking


## Implementation Requirements

To use FANET, implement the `Connector` interface:
```cpp
class MyConnector : public FANET::Connector {
public:
    uint32_t fanet_getTick() const override {
        return /* your timestamp implementation */;
    }

    bool fanet_sendFrame(uint8_t codingRate, etl::span<const uint8_t> data) override {
        return /* your radio transmission implementation */;
    }

    void fanet_ackReceived(uint16_t id) override {
        /* handle received acknowledgments */
    }
};
```

## Dependencies
- ETL (Embedded Template Library)
- C++17 or later

## Notes
- All memory allocations are static, no malloc
- Designed for embedded systems
- Uses template metaprogramming for compile-time optimizations
- Supports various payload sizes through templates

## Best Practices
1. Always check regional compliance using the Zone system
2. Implement proper error handling in your Connector
3. Monitor neighbor table size in dense networks
4. Use appropriate acknowledgment modes based on reliability needs
5. Consider airtime restrictions when sending frequent updates

