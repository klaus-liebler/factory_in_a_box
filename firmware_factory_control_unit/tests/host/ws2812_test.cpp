// src/ws2812.hpp: the WS2812 frames - patterns, single LEDs by the pixel register, and the order of
// both when they come between two I/O cycles.
#include "ws2812.hpp"

#include <gtest/gtest.h>

namespace {

using namespace fcu::ws2812;
struct NoGuard {};
using TestStrip = Strip<NoGuard>;

constexpr std::uint32_t pixel(std::uint16_t index, std::uint16_t rgb565) { return (std::uint32_t{index} << 16) | rgb565; }

TEST(Ws2812, Rgb565ExpandsToFullScale) {
    EXPECT_EQ(rgb888(0x0000), 0x000000u);
    EXPECT_EQ(rgb888(0xFFFF), 0xFFFFFFu);
    EXPECT_EQ(rgb888(0xF800), 0xFF0000u);
    EXPECT_EQ(rgb888(0x07E0), 0x00FF00u);
    EXPECT_EQ(rgb888(0x001F), 0x0000FFu);
    EXPECT_EQ(rgb888(0x8410), 0x848284u);  // mid grey: 10000 100000 10000
}

TEST(Ws2812, FirstSyncShowsThePatternAlsoZero) {
    TestStrip s;
    s.sync(0);
    const auto f = s.take();
    ASSERT_TRUE(f.has_value());
    EXPECT_EQ(*f, strip_pattern(0));
    EXPECT_FALSE(s.take().has_value());  // unchanged: nothing new to show
    s.sync(0);
    EXPECT_FALSE(s.take().has_value());
}

TEST(Ws2812, PixelChangesOneLedOfThePattern) {
    TestStrip s;
    s.sync(3);  // all green
    (void)s.take();
    ASSERT_TRUE(s.pixel(3, pixel(2, 0xF800)));
    const auto f = s.take();
    ASSERT_TRUE(f.has_value());
    for (std::size_t i = 0; i < strip_leds; ++i) EXPECT_EQ((*f)[i], i == 2 ? 0xFF0000u : 0x00FF00u) << i;
}

// Two pixel writes between two I/O cycles: both LEDs change (a polled command register would show
// only the last one).
TEST(Ws2812, TwoPixelsBetweenTwoCyclesBothCount) {
    TestStrip s;
    s.sync(0);
    (void)s.take();
    ASSERT_TRUE(s.pixel(0, pixel(0, 0x001F)));
    ASSERT_TRUE(s.pixel(individual, pixel(1, 0x07E0)));  // the hook set the pattern register to 0xFFFF
    s.sync(individual);
    const auto f = s.take();
    ASSERT_TRUE(f.has_value());
    EXPECT_EQ((*f)[0], 0x0000FFu);
    EXPECT_EQ((*f)[1], 0x00FF00u);
}

// A pattern written just before a pixel, both before the next cycle: the pixel changes the new pattern.
TEST(Ws2812, PatternThenPixelBeforeTheCycle) {
    TestStrip s;
    s.sync(0);
    (void)s.take();
    ASSERT_TRUE(s.pixel(1, pixel(7, 0x001F)));  // pattern register already 1 (red), not seen by a cycle yet
    const auto f = s.take();
    ASSERT_TRUE(f.has_value());
    for (std::size_t i = 0; i < strip_leds; ++i) EXPECT_EQ((*f)[i], i == 7 ? 0x0000FFu : 0xFF0000u) << i;
}

TEST(Ws2812, PatternAgainAfterPixelsReplacesTheFrame) {
    TestStrip s;
    s.sync(1);
    ASSERT_TRUE(s.pixel(1, pixel(0, 0x001F)));
    (void)s.take();
    s.sync(1);  // the same pattern as before the pixels: written again, it counts
    const auto f = s.take();
    ASSERT_TRUE(f.has_value());
    EXPECT_EQ(*f, strip_pattern(1));
}

TEST(Ws2812, IndexBeyondTheChannelChangesNothing) {
    TestStrip s;
    s.sync(2);
    (void)s.take();
    EXPECT_FALSE(s.pixel(2, pixel(strip_leds, 0xFFFF)));
    EXPECT_FALSE(s.take().has_value());
    Single<NoGuard> one;
    EXPECT_TRUE(one.pixel(0, pixel(0, 0xFFFF)));
    EXPECT_FALSE(one.pixel(individual, pixel(1, 0xFFFF)));
}

TEST(Ws2812, WritingIndividualByHandKeepsTheLeds) {
    TestStrip s;
    s.sync(4);
    (void)s.take();
    s.sync(individual);
    EXPECT_FALSE(s.take().has_value());
}

}  // namespace
