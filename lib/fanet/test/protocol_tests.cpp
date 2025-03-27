
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "../include/fanet/fanet.hpp"
#include "../include/fanet/protocol.hpp"
#include "etl/vector.h"
#include "helpers.hpp"

using namespace FANET;

auto RSSI_HIGH = -100;
auto RSSI_LOW = -70;

class TestProtocol : public Protocol
{
public:
    TestProtocol(Connector* connector_) : Protocol(connector_)
    {
    }

    TxFrame<uint8_t> *getNextTxFrameWrap(uint32_t timeMs)
    {
        return getNextTxFrame(timeMs);
    }

    void deleteTxFrame(TxFrame<uint8_t> *frm)
    {
        txPool.remove(frm);
    }

    void seen(Address address, uint32_t timeMs)
    {
        neighborTable_.addOrUpdate(address, timeMs);
    }

    void setAirTime(float time)
    {
        airTime(time);
    }
};

class TestApp : public Connector
{
public:
    uint16_t receivedAckId = 0;
    uint32_t receivedAckTotal = 0;
    bool sendFrameResult = true;
    bool sendFrameReceived = false;
    uint32_t TICK_TIME = 3;

    virtual uint32_t fanet_getTick() const override
    {
        return TICK_TIME;
    }

    virtual void fanet_ackReceived(uint16_t id) override
    {
        printf("============= > Ack Received %d\n", id);
        receivedAckId = id;
        receivedAckTotal++;
    }


    virtual bool fanet_sendFrame(uint8_t codingRate, const etl::span<const uint8_t> data) override
    {
        sendFrameReceived = true;
        return sendFrameResult;
    }
};

class TestFixture
{
public:
    TestApp app;
    TestProtocol protocol;
    TrackingPayload payload;

    TestFixture()
        : protocol(&app)
    {
        protocol.ownAddress(OWN_ADDRESS);
        payload.altitude(1000).climbRate(12);
    }
};

TEST_CASE_METHOD(TestFixture, "handleRx", "[Protocol]")
{

    SECTION("neighborTable")
    {
        SECTION("Adds in neighborTable", "[Protocol]")
        {
            auto v = Packet<1>().source(OTHER_ADDRESS_66).destination(OTHER_ADDRESS_55).payload(payload).build();
            protocol.handleRx(RSSI_HIGH, v);
            REQUIRE(protocol.neighborTable().lastSeen(OTHER_ADDRESS_66) == 3);

            SECTION("Update last seen")
            {
                app.TICK_TIME = 10;
                protocol.handleRx(0, v);

                REQUIRE(protocol.neighborTable().lastSeen(OTHER_ADDRESS_66) == 10);
                REQUIRE(protocol.neighborTable().size() == 1);
            }

            SECTION("Cleanup called")
            {
                app.TICK_TIME = 20 + (4 * 60 * 1000 + 10'000); // NEIGHBOR_MAX_TIMEOUT_MS
                auto other = Packet<1>().source(OTHER_ADDRESS_55).destination(OTHER_ADDRESS_66).payload(payload).build();
                protocol.handleRx(RSSI_HIGH, other);

                REQUIRE(protocol.neighborTable().lastSeen(OTHER_ADDRESS_55) == app.TICK_TIME);
                REQUIRE(protocol.neighborTable().lastSeen(OTHER_ADDRESS_66) == 0);
            }
        }
    }

    SECTION("Ignores Own Address")
    {
        auto v = Packet<1>().source(OWN_ADDRESS).payload(payload).build();
        protocol.handleRx(RSSI_HIGH, v);
        REQUIRE(protocol.pool().getAllocatedBlocks().size() == 0);
        REQUIRE(protocol.neighborTable().size() == 0);
    }

    SECTION("Init should clean ")
    {
        auto v = Packet<1>().source(OTHER_ADDRESS_55).payload(payload).build();
        protocol.handleRx(RSSI_HIGH, v);
        REQUIRE(protocol.neighborTable().size() == 1);
        auto packet = Packet<1>().payload(payload).destination(OTHER_ADDRESS_55).singleHop();
        protocol.sendPacket(packet, 0, true);
        REQUIRE(protocol.pool().getAllocatedBlocks().size() == 1);

        protocol.init();
        REQUIRE(protocol.pool().getAllocatedBlocks().size() == 0);
        REQUIRE(protocol.neighborTable().size() == 0);
    }

    SECTION("Ack response")
    {
        SECTION("Broadcast")
        {
            SECTION("No Ack requested, should not ack")
            {
                auto v = Packet<1>().source(OTHER_ADDRESS_55).payload(payload).build();
                protocol.handleRx(RSSI_HIGH, v);

                REQUIRE(protocol.pool().getAllocatedBlocks().size() == 0);
                REQUIRE(protocol.neighborTable().lastSeen(OTHER_ADDRESS_55) == 3);
            }

            SECTION("Ack over single Hop, should ack without forward")
            {
                auto v = Packet<1>().source(OTHER_ADDRESS_55).payload(payload).singleHop().build();
                auto resultType = protocol.handleRx(RSSI_HIGH, v);

                REQUIRE(resultType == payload.type());
                auto poolItem = findByAddress(protocol, OTHER_ADDRESS_55, OWN_ADDRESS);
                REQUIRE(protocol.pool().getAllocatedBlocks().size() == 1);
                REQUIRE(protocol.neighborTable().lastSeen(OTHER_ADDRESS_55) == 3);
                REQUIRE(makeVectorS<100>(poolItem->data()) == makeVector({
                                                                  0x80,
                                                                  0x11,
                                                                  0x11,
                                                                  0x11,
                                                                  0x20,
                                                                  0x55,
                                                                  0x55,
                                                                  0x55,
                                                              }));
            }

            SECTION("Ack over Two Hop, should ack with forward")
            {
                auto v = Packet<1>().source(OTHER_ADDRESS_55).payload(payload).twoHop().build();
                protocol.handleRx(RSSI_HIGH, v);

                auto poolItem = findByAddress(protocol, OTHER_ADDRESS_55, OWN_ADDRESS);
                REQUIRE(protocol.pool().getAllocatedBlocks().size() == 1);
                REQUIRE(protocol.neighborTable().lastSeen(OTHER_ADDRESS_55) == 3);
                dumpHex(poolItem->data());
                REQUIRE(makeVectorS<100>(poolItem->data()) == makeVector({
                                                                  0xC0,
                                                                  0x11,
                                                                  0x11,
                                                                  0x11,
                                                                  0x20,
                                                                  0x55,
                                                                  0x55,
                                                                  0x55,
                                                              }));
            }
        }

        SECTION("Unicast")
        {
            SECTION("No Ack requested, should not ack")
            {
                auto v = Packet<1>().source(OTHER_ADDRESS_55).destination(OWN_ADDRESS).payload(payload).build();
                protocol.handleRx(0, v);

                REQUIRE(protocol.pool().getAllocatedBlocks().size() == 0);
                REQUIRE(protocol.neighborTable().lastSeen(OTHER_ADDRESS_55) == 3);
            }

            SECTION("Ack over single Hop, should ack without forward")
            {
                auto v = Packet<1>().source(OTHER_ADDRESS_55).payload(payload).destination(OWN_ADDRESS).singleHop().build();
                protocol.handleRx(RSSI_HIGH, v);

                auto poolItem = findByAddress(protocol, OTHER_ADDRESS_55, OWN_ADDRESS);
                REQUIRE(protocol.pool().getAllocatedBlocks().size() == 1);
                REQUIRE(protocol.neighborTable().lastSeen(OTHER_ADDRESS_55) == 3);
                REQUIRE(makeVectorS<100>(poolItem->data()) == makeVector({
                                                                  0x80,
                                                                  0x11,
                                                                  0x11,
                                                                  0x11,
                                                                  0x20,
                                                                  0x55,
                                                                  0x55,
                                                                  0x55,
                                                              }));
            }

            SECTION("Ack over Two Hop, should ack and forward")
            {
                auto v = Packet<1>().source(OTHER_ADDRESS_55).payload(payload).destination(OWN_ADDRESS).twoHop().build();
                protocol.handleRx(RSSI_HIGH, v);

                auto poolItem = findByAddress(protocol, OTHER_ADDRESS_55, OWN_ADDRESS);
                REQUIRE(protocol.pool().getAllocatedBlocks().size() == 1);
                REQUIRE(protocol.neighborTable().lastSeen(OTHER_ADDRESS_55) == 3);
                REQUIRE(makeVectorS<100>(poolItem->data()) == makeVector({
                                                                  0xC0,
                                                                  0x11,
                                                                  0x11,
                                                                  0x11,
                                                                  0x20,
                                                                  0x55,
                                                                  0x55,
                                                                  0x55,
                                                              }));
            }
        }
    }

    SECTION("Ack Received")
    {
        SECTION("With Packed in pool")
        {
            // Add packet to pool with a request to ack (singleHop, twoHop is set)
            auto packet = Packet<1>().payload(payload).destination(OTHER_ADDRESS_55).singleHop();
            protocol.sendPacket(packet, 10);
            packet = Packet<1>().payload(payload).destination(OTHER_ADDRESS_66).singleHop();
            protocol.sendPacket(packet, 11);
            REQUIRE(protocol.pool().getAllocatedBlocks().size() == 2);

            SECTION("When ack received for us, should remove the frames")
            {
                auto ack = Packet<1>().source(OTHER_ADDRESS_55).destination(OWN_ADDRESS).buildAck();
                protocol.handleRx(RSSI_HIGH, ack);

                REQUIRE(protocol.pool().getAllocatedBlocks().size() == 1);
                REQUIRE(findByAddress(protocol, OTHER_ADDRESS_55) == nullptr);
                REQUIRE(findByAddress(protocol, OTHER_ADDRESS_66) != nullptr);
                REQUIRE(app.receivedAckId == 10);
                REQUIRE(app.receivedAckTotal == 1);
            }

            SECTION("When ack received broadcast should remove frames")
            {
                auto ack = Packet<1>().source(OTHER_ADDRESS_55).buildAck();
                protocol.handleRx(RSSI_HIGH, ack);

                REQUIRE(protocol.pool().getAllocatedBlocks().size() == 1);
                REQUIRE(findByAddress(protocol, OTHER_ADDRESS_55) == nullptr);
                REQUIRE(findByAddress(protocol, OTHER_ADDRESS_66) != nullptr);
                REQUIRE(app.receivedAckId == 10);
                REQUIRE(app.receivedAckTotal == 1);
            }

            SECTION("When not ack received for us")
            {
                auto not_ack = Packet<1>().source(OTHER_ADDRESS_55).destination(OWN_ADDRESS).payload(payload).build();
                protocol.handleRx(RSSI_HIGH, not_ack);

                REQUIRE(findByAddress(protocol, OTHER_ADDRESS_55) != nullptr);
                REQUIRE(findByAddress(protocol, OTHER_ADDRESS_66) != nullptr);
            }

            SECTION("When ack received broadcast with forward, should remove forward and ack")
            {
                auto ack = Packet<1>().source(OTHER_ADDRESS_55).forward(true).buildAck();
                protocol.handleRx(RSSI_HIGH, ack);

                auto poolItem = findByAddress(protocol, BROADCAST_ADDRESS, OTHER_ADDRESS_55);
                REQUIRE(findByAddress(protocol, OTHER_ADDRESS_66) != nullptr);

                // Should we really send a broadcast like this????
                REQUIRE(makeVectorS<100>(poolItem->data()) == makeVector({
                                                                  0x00,
                                                                  0x55,
                                                                  0x55,
                                                                  0x55,
                                                              }));
                REQUIRE(app.receivedAckId == 10);
                REQUIRE(app.receivedAckTotal == 1);
            }

            SECTION("When ack received for other")
            {
                auto ack = Packet<1>().source(OTHER_ADDRESS_55).destination(OTHER_ADDRESS_UNR).buildAck();
                protocol.handleRx(RSSI_HIGH, ack);

                REQUIRE(findByAddress(protocol, OTHER_ADDRESS_55) != nullptr);
                REQUIRE(findByAddress(protocol, OTHER_ADDRESS_66) != nullptr);
            }
        }

        SECTION("When ack received seen with forward, should forward ack frames")
        {
            // Add to pool so we have seen OTHER_ADDRESS_66
            protocol.seen(OTHER_ADDRESS_66, app.TICK_TIME);

            auto ack = Packet<1>().source(OTHER_ADDRESS_55).destination(OTHER_ADDRESS_66).forward(true).buildAck();
            protocol.handleRx(RSSI_HIGH, ack);

            auto poolItem = findByAddress(protocol, Header::MessageType::ACK, OTHER_ADDRESS_66, OTHER_ADDRESS_55);
            REQUIRE(makeVectorS<100>(poolItem->data()) == makeVector({0x80, 0x55, 0x55, 0x55, 0x20, 0x66, 0x66, 0x66}));
        }

        SECTION("When ack received not seen with forward, should not forward")
        {
            auto ack = Packet<1>().source(OTHER_ADDRESS_55).destination(OTHER_ADDRESS_UNR).forward(true).buildAck();
            protocol.handleRx(RSSI_HIGH, ack);
            REQUIRE(protocol.pool().getAllocatedBlocks().size() == 0);
        }
    }

    SECTION("Packet Forwarding Unicast")
    {
        auto FORWARDPACKETUNI = Packet<1>().source(OTHER_ADDRESS_UNR).destination(OTHER_ADDRESS_66).payload(payload).forward(true).build();
        auto FORWARDPACKETUNIONEHOP = Packet<1>().source(OTHER_ADDRESS_UNR).destination(OTHER_ADDRESS_66).payload(payload).forward(true).ack(ExtendedHeader::AckType::SINGLEHOP).build();
        auto FORWARDPACKETBROADCAST = Packet<1>().source(OTHER_ADDRESS_UNR).payload(payload).forward(true).build();

        SECTION("Seen, forward without ack, low RSSI ")
        {
            protocol.seen(OTHER_ADDRESS_66, app.TICK_TIME);

            SECTION("Should not forward, not seen")
            {
                protocol.handleRx(RSSI_LOW, FORWARDPACKETUNI);
                REQUIRE(protocol.pool().getAllocatedBlocks().size() == 0);
            }

            SECTION("Should forward unicast")
            {
                protocol.handleRx(RSSI_HIGH, FORWARDPACKETUNI);
                auto poolItem = findByAddress(protocol, OTHER_ADDRESS_66, OTHER_ADDRESS_UNR);
                REQUIRE(protocol.pool().getAllocatedBlocks().size() == 1);
                REQUIRE(poolItem->numTx() == 0);
                REQUIRE(poolItem->nextTx() >= 103);
                REQUIRE(poolItem->rssi() >= RSSI_HIGH);
                REQUIRE(makeVectorS<100>(poolItem->data()) == makeVector({
                                                                  0x81,
                                                                  0xEE,
                                                                  0xEE,
                                                                  0xEE,
                                                                  0x20,
                                                                  0x66,
                                                                  0x66,
                                                                  0x66,
                                                                  0x00,
                                                                  0x00,
                                                                  0x00,
                                                                  0x00,
                                                                  0x00,
                                                                  0x00,
                                                                  0xE8,
                                                                  0x03,
                                                                  0x00,
                                                                  0x98,
                                                                  0x00,
                                                              }));

                SECTION("Should drop frames with LOW RSSI")
                {
                    protocol.handleRx(RSSI_LOW, FORWARDPACKETUNI);
                    REQUIRE(protocol.pool().getAllocatedBlocks().size() == 0);
                }

                SECTION("Should adjust departure time")
                {
                    app.TICK_TIME = 5000;
                    protocol.handleRx(RSSI_HIGH, FORWARDPACKETUNI);
                    auto poolItem = findByAddress(protocol, OTHER_ADDRESS_66, OTHER_ADDRESS_UNR);
                    REQUIRE(poolItem->nextTx() >= 5000);
                }

                SECTION("Should not adjust departure time with different payload")
                {
                    auto payloadDifferent = payload;
                    payloadDifferent.climbRate(13);
                    auto DIFFERENT = Packet<1>().source(OTHER_ADDRESS_UNR).destination(OTHER_ADDRESS_66).payload(payloadDifferent).forward(true).build();

                    app.TICK_TIME = 5000;
                    protocol.handleRx(RSSI_HIGH, DIFFERENT);
                    auto poolItem = findByAddress(protocol, OTHER_ADDRESS_66, OTHER_ADDRESS_UNR);
                    REQUIRE(poolItem->nextTx() < 2000);
                }
            }

            SECTION("Should forward unicast with hop")
            {
                protocol.handleRx(RSSI_HIGH, FORWARDPACKETUNIONEHOP);
                auto poolItem = findByAddress(protocol, OTHER_ADDRESS_66, OTHER_ADDRESS_UNR);
                REQUIRE(poolItem->numTx() == 1);
            }

            SECTION("Should forward broadcast")
            {
                protocol.handleRx(RSSI_HIGH, FORWARDPACKETBROADCAST);
                auto poolItem = findByAddress(protocol, IGNORING_ADDRESS, OTHER_ADDRESS_UNR);
                REQUIRE(protocol.pool().getAllocatedBlocks().size() == 1);
                REQUIRE(poolItem->numTx() == 0);
                REQUIRE(poolItem->nextTx() >= 103);
                REQUIRE(makeVectorS<100>(poolItem->data()) == makeVector({
                                                                  0x01,
                                                                  0xEE,
                                                                  0xEE,
                                                                  0xEE,
                                                                  0x00,
                                                                  0x00,
                                                                  0x00,
                                                                  0x00,
                                                                  0x00,
                                                                  0x00,
                                                                  0xE8,
                                                                  0x03,
                                                                  0x00,
                                                                  0x98,
                                                                  0x00,
                                                              }));
            }

            SECTION("Should not forward due to high airtime")
            {
                protocol.setAirTime(1000);
                protocol.handleRx(RSSI_HIGH, FORWARDPACKETUNI);
                REQUIRE(protocol.pool().getAllocatedBlocks().size() == 0);
            }
        }

        SECTION("Not Seen, should not add to queue")
        {
            protocol.handleRx(RSSI_HIGH, FORWARDPACKETUNI);
            REQUIRE(protocol.pool().getAllocatedBlocks().size() == 0);
        }
    }
}

TEST_CASE_METHOD(TestFixture, "sendPacket in strict mode", "[Protocol]")
{
    app.TICK_TIME = 50;
    SECTION("Should set required values no ack")
    {
        auto packet = Packet<1>().payload(payload);
        protocol.sendPacket(packet, 11);
        REQUIRE(protocol.pool().begin()->source() == OWN_ADDRESS);
        REQUIRE(protocol.pool().begin()->id() == 11);
        REQUIRE(protocol.pool().begin()->forward() == false);
        REQUIRE(protocol.pool().begin()->self() == true);
        REQUIRE(protocol.pool().begin()->numTx() == 0);
        REQUIRE(protocol.pool().begin()->nextTx() == app.TICK_TIME);
    }

    SECTION("Should set required values")
    {
        auto packet = Packet<1>().payload(payload).singleHop();
        protocol.sendPacket(packet, 10);
        REQUIRE(protocol.pool().begin()->source() == OWN_ADDRESS);
        REQUIRE(protocol.pool().begin()->id() == 10);
        REQUIRE(protocol.pool().begin()->forward() == true);
        REQUIRE(protocol.pool().begin()->self() == true);
        REQUIRE(protocol.pool().begin()->numTx() == 3);
        REQUIRE(protocol.pool().begin()->nextTx() == app.TICK_TIME);
    }
}

TEST_CASE_METHOD(TestFixture, "getNextTxFrame", "[Protocol]")
{
    protocol.seen(OTHER_ADDRESS_55, app.TICK_TIME);
    protocol.seen(OTHER_ADDRESS_66, app.TICK_TIME);

    SECTION("Should retrieve in the correct order high to low -> Self Priority Ack Others")
    {
        // Setup
        app.TICK_TIME = 3;
        GroundTrackingPayload gtPayload;

        auto PAYLOADPACKGE = Packet<1>().source(OTHER_ADDRESS_UNR).destination(OTHER_ADDRESS_66).payload(gtPayload).forward(true).ack(ExtendedHeader::AckType::SINGLEHOP).build();

        MessagePayload<5> messagepayload;
        auto MESSAGEPACKET = Packet<5>().source(OTHER_ADDRESS_UNR).destination(OTHER_ADDRESS_55).payload(messagepayload).forward(true).ack(ExtendedHeader::AckType::SINGLEHOP).build();

        // Ack frames other
        auto ack = Packet<1>().source(OTHER_ADDRESS_55).forward(true).buildAck();
        protocol.handleRx(RSSI_HIGH, ack);

        // Non tracking
        protocol.handleRx(RSSI_HIGH, MESSAGEPACKET);

        // Priority frames
        protocol.handleRx(RSSI_HIGH, PAYLOADPACKGE);

        // Add self packet
        NamePayload<5> namePayload;
        auto selfPacket = Packet<5>().payload(namePayload);
        protocol.sendPacket(selfPacket, 0);

        // TEST
        app.TICK_TIME = 2;
        REQUIRE(protocol.getNextTxFrameWrap(app.TICK_TIME) == nullptr);
        REQUIRE(protocol.pool().getAllocatedBlocks().size() == 4);

        // Self
        app.TICK_TIME = 10000;
        auto txFrame = protocol.getNextTxFrameWrap(app.TICK_TIME);
        REQUIRE(txFrame->type() == Header::MessageType::NAME);
        REQUIRE(txFrame->source() == OWN_ADDRESS);
        protocol.deleteTxFrame(txFrame);
        REQUIRE(protocol.pool().getAllocatedBlocks().size() == 3);

        // Priority
        txFrame = protocol.getNextTxFrameWrap(app.TICK_TIME);
        REQUIRE(txFrame->type() == Header::MessageType::GROUND_TRACKING);
        protocol.deleteTxFrame(txFrame);
        REQUIRE(protocol.pool().getAllocatedBlocks().size() == 2);

        // Ack
        txFrame = protocol.getNextTxFrameWrap(app.TICK_TIME);
        REQUIRE(txFrame->type() == Header::MessageType::ACK);
        protocol.deleteTxFrame(txFrame);
        REQUIRE(protocol.pool().getAllocatedBlocks().size() == 1);

        // Others
        txFrame = protocol.getNextTxFrameWrap(app.TICK_TIME);
        REQUIRE(txFrame->type() == Header::MessageType::MESSAGE);
        protocol.deleteTxFrame(txFrame);
        REQUIRE(protocol.pool().getAllocatedBlocks().size() == 0);
    }

    SECTION("Should Retreive correctly order based on time -> Self tracking Ack Others")
    {
        GroundTrackingPayload gtPayload;

        // Add self packet
        app.TICK_TIME = 15000;
        auto selfPacket = Packet<5>().payload(gtPayload).destination(OTHER_ADDRESS_66);
        protocol.sendPacket(selfPacket, 0);

        app.TICK_TIME = 10000;
        selfPacket = Packet<5>().payload(gtPayload).destination(OTHER_ADDRESS_55);
        protocol.sendPacket(selfPacket, 0);

        app.TICK_TIME = 5000;
        auto txFrame = protocol.getNextTxFrameWrap(app.TICK_TIME);
        REQUIRE(txFrame == nullptr);

        app.TICK_TIME = 12000;
        txFrame = protocol.getNextTxFrameWrap(app.TICK_TIME);
        REQUIRE(txFrame->destination() == OTHER_ADDRESS_55);

        app.TICK_TIME = 22000;
        txFrame = protocol.getNextTxFrameWrap(app.TICK_TIME);
        REQUIRE(txFrame->destination() == OTHER_ADDRESS_55);

        protocol.deleteTxFrame(txFrame);
        txFrame = protocol.getNextTxFrameWrap(app.TICK_TIME);
        REQUIRE(txFrame->destination() == OTHER_ADDRESS_66);
    }
}

TEST_CASE_METHOD(TestFixture, "handleTx", "[Protocol]")
{
    protocol.seen(OTHER_ADDRESS_55, app.TICK_TIME);
    protocol.seen(OTHER_ADDRESS_66, app.TICK_TIME);

    MessagePayload<5> messagepayload;
    auto MESSAGEPACKET = Packet<5>().payload(messagepayload);

    SECTION("When sendPackage")
    {
        SECTION("Without ack")
        {
            auto selfPacket = Packet<5>().payload(NamePayload<5>{}).destination(OTHER_ADDRESS_55);
            protocol.sendPacket(selfPacket, 0);

            protocol.handleTx();
            REQUIRE(app.sendFrameReceived == true);
            REQUIRE(protocol.pool().getAllocatedBlocks().size() == 0);
        }

        SECTION("With ack non tracking")
        {
            auto selfPacket = Packet<5>().payload(NamePayload<5>{}).destination(OTHER_ADDRESS_55).singleHop();
            protocol.sendPacket(selfPacket, 0);

            SECTION("When Success, should keep the frames and handle backoff")
            {
                auto nextTx = protocol.handleTx();
                REQUIRE(app.sendFrameReceived == true);
                auto it = findByAddress(protocol, OTHER_ADDRESS_55, OWN_ADDRESS);
                REQUIRE(it->numTx() == 2);
                REQUIRE(it->nextTx() == 1003);
                REQUIRE(nextTx == 34);

                app.TICK_TIME = 1003;
                nextTx = protocol.handleTx();
                it = findByAddress(protocol, OTHER_ADDRESS_55, OWN_ADDRESS);
                REQUIRE(it->numTx() == 1);
                REQUIRE(it->nextTx() == 3003);
                REQUIRE(nextTx == 1034);

                app.TICK_TIME = 3003;
                nextTx = protocol.handleTx();
                it = findByAddress(protocol, OTHER_ADDRESS_55, OWN_ADDRESS);
                REQUIRE(it->numTx() == 0);
                REQUIRE(it->nextTx() == 4003);
                REQUIRE(nextTx == 3034);

                app.TICK_TIME = 4003;
                nextTx = protocol.handleTx();
                it = findByAddress(protocol, OTHER_ADDRESS_55, OWN_ADDRESS);
                REQUIRE(it == nullptr);
                REQUIRE(nextTx == 5003);
            }

            SECTION("When Failed, should keep for retry")
            {
                app.sendFrameResult = false;
                protocol.handleTx();
                REQUIRE(app.sendFrameReceived == true);
                REQUIRE(protocol.pool().getAllocatedBlocks().size() == 1);
            }
        }

        SECTION("With ack tracking")
        {
            auto selfPacket = Packet<5>().payload(TrackingPayload{}).destination(OTHER_ADDRESS_55).singleHop();
            protocol.sendPacket(selfPacket, 0);

            SECTION("When Success, should remove the frame")
            {
                protocol.handleTx();
                REQUIRE(app.sendFrameReceived == true);
                REQUIRE(protocol.pool().getAllocatedBlocks().size() == 0);
            }

            SECTION("When Failed, should remove the frame")
            {
                app.sendFrameResult = false;
                protocol.handleTx();
                REQUIRE(app.sendFrameReceived == true);
                REQUIRE(protocol.pool().getAllocatedBlocks().size() == 0);
            }
        }
    }

    SECTION("Package time not ready")
    {
        app.TICK_TIME = 10000;
        auto selfPacket = Packet<5>().payload(NamePayload<5>{}).destination(OTHER_ADDRESS_55);
        protocol.sendPacket(selfPacket, 0);
        app.TICK_TIME = 9000;
        protocol.handleTx();
        REQUIRE(app.sendFrameReceived == false);
        REQUIRE(findByAddress(protocol, OTHER_ADDRESS_55, OWN_ADDRESS) != nullptr);
    }
}