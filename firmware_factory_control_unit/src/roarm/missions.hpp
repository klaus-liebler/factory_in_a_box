// RoArm missions on the SD card (sd_card.hpp): one file "/MISSIONS/<index>.MSN" per mission
// holding the encoded WsProtocol::roarm::Mission (name and steps), written as "<index>.TMP" and
// renamed, so a power loss keeps the old or the new one. The card is mounted at the start and, if it
// is missing or fails, again at the next request (at most every 5 s): a card inserted later is
// found. Without a card there are no missions.
// STM32H563 boards (revisions 1/2) had the missions in the internal flash (littlefs, the last
// 256 KB): the first mount moves them onto the card (a mission already on the card wins) and
// removes them from the flash. Owned by the I/O thread (FileX and littlefs have no locks).
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace fcu::roarm {

inline constexpr std::size_t max_name = 31;
inline constexpr std::size_t max_mission_bytes = 4096;

struct MissionSummary {
    std::uint16_t index = 0;
    std::array<char, max_name + 1> name{};
};

namespace missions {
bool mount() noexcept;
/// Up to out.size() missions; returns how many.
std::size_t list(std::span<MissionSummary> out) noexcept;
/// The encoded mission into `out`; returns its size, 0 if it does not exist.
std::size_t load(std::uint16_t index, std::span<std::uint8_t> out) noexcept;
bool save(std::uint16_t index, std::string_view name, std::span<const std::uint8_t> steps, std::size_t step_count) noexcept;
bool remove(std::uint16_t index) noexcept;
}  // namespace missions

}  // namespace fcu::roarm
