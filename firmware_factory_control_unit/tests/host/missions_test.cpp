// RoArm missions on the (simulated) SD card: src/roarm/missions.cpp against sim/sd_card_sim.cpp.
#include <gtest/gtest.h>

#include <array>
#include <string>
#include <vector>

#include "roarm/missions.hpp"
#include "sd_card_sim.hpp"
#include "ws_protocol.hh"

namespace {

using namespace fcu::roarm;
namespace ws = WsProtocol::roarm;

/// Encoded steps (what the web UI sends in SaveMissionRequest) and their count.
struct Steps {
    std::array<std::uint8_t, 512> bytes{};
    std::size_t size = 0;
    std::size_t count = 0;

    void move(std::int16_t angle) {
        ws::JointMoveStep::Payload s{};
        for (auto& a : s.jointAnglesCentiDeg) {
            a = angle;
        }
        s.maxSpeedDegPerSec = 60;
        size = ws::AppendMissionStepsJointMoveStepElement(s, bytes.data(), size, bytes.size());
        ++count;
    }
    void gpio(std::uint8_t id, bool state) {
        size = ws::AppendMissionStepsGpioStepElement(ws::GpioStep::Payload{id, state}, bytes.data(), size, bytes.size());
        ++count;
    }
    std::span<const std::uint8_t> span() const { return {bytes.data(), size}; }
};

std::vector<MissionSummary> list_all() {
    std::array<MissionSummary, 16> out{};
    const std::size_t n = missions::list(out);
    return {out.begin(), out.begin() + static_cast<std::ptrdiff_t>(n)};
}

class Missions : public ::testing::Test {
protected:
    void SetUp() override { fcu::sd::sim::insert_blank_card(); }
};

TEST_F(Missions, SaveThenLoadRoundTripsNameAndSteps) {
    Steps steps;
    steps.move(1000);
    steps.gpio(2, true);
    ASSERT_TRUE(missions::save(7, "Greifen", steps.span(), steps.count));

    std::array<std::uint8_t, max_mission_bytes> buffer{};
    const std::size_t size = missions::load(7, buffer);
    ASSERT_GT(size, 0u);
    ws::Mission::Payload m{};
    ASSERT_TRUE(ws::Mission::Decode(buffer.data(), size, m));
    EXPECT_STREQ(m.name, "Greifen");
    ASSERT_EQ(m.stepsCount, 2u);
    std::string seen;
    ASSERT_TRUE(ws::DecodeMissionStepsElements(m.stepsData, m.stepsDataSize, m.stepsCount, [&](const auto& step) {
        using T = std::decay_t<decltype(step)>;
        if constexpr (std::is_same_v<T, ws::JointMoveStep::Payload>) {
            seen += "M" + std::to_string(step.jointAnglesCentiDeg[0]);
        } else if constexpr (std::is_same_v<T, ws::GpioStep::Payload>) {
            seen += "G" + std::to_string(step.gpioId) + (step.state ? "+" : "-");
        } else {
            seen += "?";
        }
    }));
    EXPECT_EQ(seen, "M1000G2+");
}

// Regression (combination of two causes): list() decoded only the beginning of each mission file
// (max_name + 8 bytes), and BestBinaryBuffers' Decode() checks the byte-length prefix of the steps
// list (since the format with that prefix) against the data actually present -- together every
// mission with more step bytes than fit into that beginning vanished from the list. Both a short
// and a maximum-length name.
TEST_F(Missions, ListShowsMissionsWhoseStepsDoNotFitIntoTheFileBeginning) {
    Steps steps;
    for (int i = 0; i < 10; ++i) {
        steps.move(static_cast<std::int16_t>(i * 100));
    }
    ASSERT_GT(steps.size, max_name + 8);
    ASSERT_TRUE(missions::save(1, "Pick", steps.span(), steps.count));
    const std::string long_name(max_name, 'L');
    ASSERT_TRUE(missions::save(2, long_name, steps.span(), steps.count));

    const auto all = list_all();
    ASSERT_EQ(all.size(), 2u);
    EXPECT_EQ(all[0].index, 1);
    EXPECT_STREQ(all[0].name.data(), "Pick");
    EXPECT_EQ(all[1].index, 2);
    EXPECT_EQ(std::string(all[1].name.data()), long_name);
}

TEST_F(Missions, ListIsSortedByIndex) {
    Steps steps;
    steps.gpio(0, false);
    ASSERT_TRUE(missions::save(12, "c", steps.span(), steps.count));
    ASSERT_TRUE(missions::save(3, "a", steps.span(), steps.count));
    ASSERT_TRUE(missions::save(7, "b", steps.span(), steps.count));
    const auto all = list_all();
    ASSERT_EQ(all.size(), 3u);
    EXPECT_EQ(all[0].index, 3);
    EXPECT_EQ(all[1].index, 7);
    EXPECT_EQ(all[2].index, 12);
}

TEST_F(Missions, SavingAgainReplacesTheMission) {
    Steps one;
    one.gpio(1, true);
    Steps two;
    two.gpio(1, true);
    two.gpio(2, true);
    ASSERT_TRUE(missions::save(4, "alt", one.span(), one.count));
    ASSERT_TRUE(missions::save(4, "neu", two.span(), two.count));
    const auto all = list_all();
    ASSERT_EQ(all.size(), 1u);
    EXPECT_STREQ(all[0].name.data(), "neu");
}

TEST_F(Missions, RemoveDeletesTheMission) {
    Steps steps;
    steps.gpio(1, true);
    ASSERT_TRUE(missions::save(4, "weg", steps.span(), steps.count));
    ASSERT_TRUE(missions::remove(4));
    EXPECT_TRUE(list_all().empty());
    std::array<std::uint8_t, max_mission_bytes> buffer{};
    EXPECT_EQ(missions::load(4, buffer), 0u);
}

TEST_F(Missions, WithoutCardThereAreNoMissions) {
    fcu::sd::sim::remove_card();
    Steps steps;
    steps.gpio(1, true);
    EXPECT_FALSE(missions::save(1, "x", steps.span(), steps.count));
    EXPECT_TRUE(list_all().empty());
    std::array<std::uint8_t, max_mission_bytes> buffer{};
    EXPECT_EQ(missions::load(1, buffer), 0u);
}

TEST_F(Missions, MissionsSurviveRemovingAndReinsertingTheCard) {
    Steps steps;
    steps.gpio(1, true);
    ASSERT_TRUE(missions::save(9, "bleibt", steps.span(), steps.count));
    fcu::sd::sim::remove_card();
    EXPECT_TRUE(list_all().empty());
    fcu::sd::sim::reinsert_card();
    const auto all = list_all();
    ASSERT_EQ(all.size(), 1u);
    EXPECT_STREQ(all[0].name.data(), "bleibt");
}

TEST_F(Missions, ANewCardGetsTheMissionDirectory) {
    Steps steps;
    steps.gpio(1, true);
    ASSERT_TRUE(missions::save(1, "erste Karte", steps.span(), steps.count));
    fcu::sd::sim::insert_blank_card();  // another, empty card: /MISSIONS must be created again
    EXPECT_TRUE(list_all().empty());
    ASSERT_TRUE(missions::save(2, "zweite Karte", steps.span(), steps.count));
    const auto all = list_all();
    ASSERT_EQ(all.size(), 1u);
    EXPECT_EQ(all[0].index, 2);
}

// Files in the format before the byte-length prefix (2026-11-17) are no longer readable -- accepted
// (missions are recreated). They must be skipped cleanly, not break the list.
TEST_F(Missions, FileInTheOldFormatIsSkipped) {
    ASSERT_TRUE(missions::mount());
    Steps steps;
    steps.gpio(1, true);
    ASSERT_TRUE(missions::save(1, "neu", steps.span(), steps.count));

    // old layout: header, name, count, elements -- no byte length before the elements
    std::vector<std::uint8_t> old{};
    auto u16 = [&](std::uint16_t v) {
        old.push_back(static_cast<std::uint8_t>(v));
        old.push_back(static_cast<std::uint8_t>(v >> 8));
    };
    u16(ws::NAMESPACE_ID);
    u16(ws::Mission::TYPE_ID);
    u16(0);
    u16(0);
    for (char c : std::string("alt")) {
        old.push_back(static_cast<std::uint8_t>(c));
    }
    old.push_back(0);
    u16(1);                         // one step
    u16(ws::DelayStep::CLASS_ID);   // DelayStep{durationMs}
    u16(500);
    ASSERT_TRUE(fcu::sd::sim::write_file("/MISSIONS/5.MSN", old));

    const auto all = list_all();
    ASSERT_EQ(all.size(), 1u);
    EXPECT_STREQ(all[0].name.data(), "neu");
}

}  // namespace
