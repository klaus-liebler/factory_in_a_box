// The WS2812 channels' frames: what the LEDs of a channel shall show, set two ways (schema/register_map/
// Ws2812.cs):
//   pattern register  a pattern (CH1) or palette colour (CH2) replaces the whole frame
//   pixel register    32 bits, high word an LED index, low word an RGB565 colour: changes one LED of
//                     the frame as it is; the pattern register then reads `individual` (0xFFFF) until a
//                     pattern is written again, the pixel register `pixel_ready` (a command's answer)
//
// The pixel register is a command: every write counts, also two within one I/O cycle (LED 0, then
// LED 1). So a pixel write is applied at once, in the writer's thread (the register bank's write hook,
// io::holding_written), not polled by the I/O cycle - polling would see only the last of them. Pattern
// writes are taken both ways: in the hook (before a pixel that follows it at once) and by the I/O
// cycle's sync() (writes that bypass the hook: the PLC's safe state).
//
// Pure (no hardware): tests/host/ws2812_test.cpp. Guard: an RAII lock type (Kernel::Lock on the target).
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace fcu::ws2812 {

/// The pattern register's value while single LEDs were set.
inline constexpr std::uint16_t individual = 0xFFFF;

/// The pixel register's answer once a write is done ("ready"; also at the start): every write that
/// follows is a change - the PLC writes a command only when the program changed it (plc.cpp), so
/// "LED 0 red" again after a pattern still arrives.
inline constexpr std::uint32_t pixel_ready = 0xFFFF'FFFF;

/// RGB565 -> 0xRRGGBB; each component's top bits repeated into the low ones (full scale stays full).
[[nodiscard]] constexpr std::uint32_t rgb888(std::uint16_t c) noexcept {
    const std::uint32_t r5 = (c >> 11) & 0x1F, g6 = (c >> 5) & 0x3F, b5 = c & 0x1F;
    const std::uint32_t r = (r5 << 3) | (r5 >> 2), g = (g6 << 2) | (g6 >> 4), b = (b5 << 3) | (b5 >> 2);
    return (r << 16) | (g << 8) | b;
}

/// The pixel register's parts.
[[nodiscard]] constexpr std::uint16_t pixel_index(std::uint32_t value) noexcept { return static_cast<std::uint16_t>(value >> 16); }
[[nodiscard]] constexpr std::uint16_t pixel_colour(std::uint32_t value) noexcept { return static_cast<std::uint16_t>(value); }

inline constexpr std::size_t strip_leds = 8;  ///< CH1
using StripFrame = std::array<std::uint32_t, strip_leds>;
using SingleFrame = std::array<std::uint32_t, 1>;  ///< CH2

/// CH1: 0 off, 1 red, 2 yellow, 3 green, 4 blue/white alternating (modulo 5).
[[nodiscard]] constexpr StripFrame strip_pattern(std::uint16_t p) noexcept {
    StripFrame c{};
    switch (p % 5) {
        case 1: c.fill(0xFF0000); break;
        case 2: c.fill(0xFFFF00); break;
        case 3: c.fill(0x00FF00); break;
        case 4:
            for (std::size_t i = 0; i < c.size(); ++i) c[i] = i % 2 == 0 ? 0x0000FF : 0xFFFFFF;
            break;
        default: break;
    }
    return c;
}

/// CH2: a palette colour (modulo 8).
[[nodiscard]] constexpr SingleFrame single_pattern(std::uint16_t p) noexcept {
    constexpr std::array<std::uint32_t, 8> palette{0x000000, 0xFF0000, 0x00FF00, 0x0000FF, 0xFFFFFF, 0xFFFF00, 0x00FFFF, 0xFF00FF};
    return {palette[p % palette.size()]};
}

/// One channel's frame. Every member takes the guard: the hook runs in the writer's thread (network,
/// EtherCAT, PLC), take() in the I/O thread.
template <std::size_t N, std::array<std::uint32_t, N> (*Pattern)(std::uint16_t) noexcept, class Guard>
class Channel {
public:
    using Frame = std::array<std::uint32_t, N>;

    /// The pattern register's value: a pattern other than the one shown replaces the frame.
    void sync(std::uint16_t pattern) noexcept {
        [[maybe_unused]] const Guard guard{};
        sync_locked(pattern);
    }

    /// A pixel register write (`pattern`: the pattern register's value now). false: the index is
    /// beyond the channel (nothing changed); true: the caller sets the pattern register to `individual`.
    [[nodiscard]] bool pixel(std::uint16_t pattern, std::uint32_t value) noexcept {
        [[maybe_unused]] const Guard guard{};
        sync_locked(pattern);  // a pattern written just before: the pixel changes it, not the old frame
        const std::uint16_t index = pixel_index(value);
        if (index >= N) return false;
        frame_[index] = rgb888(pixel_colour(value));
        shown_ = individual;
        dirty_ = true;
        return true;
    }

    /// The frame, if it changed since the last call (the I/O thread, when the LEDs can take it).
    [[nodiscard]] std::optional<Frame> take() noexcept {
        [[maybe_unused]] const Guard guard{};
        if (!dirty_) return std::nullopt;
        dirty_ = false;
        return frame_;
    }

private:
    void sync_locked(std::uint16_t pattern) noexcept {
        if (pattern == individual || pattern == shown_) return;
        frame_ = Pattern(pattern);
        shown_ = pattern;
        dirty_ = true;
    }

    Frame frame_{};
    std::uint32_t shown_ = 0x10000;  ///< the pattern in the frame; none yet: the first sync() applies one
    bool dirty_ = false;
};

template <class Guard>
using Strip = Channel<strip_leds, strip_pattern, Guard>;
template <class Guard>
using Single = Channel<1, single_pattern, Guard>;

}  // namespace fcu::ws2812
