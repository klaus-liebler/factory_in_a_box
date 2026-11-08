// RoArm missions in the internal flash: littlefs outside the image (FLASH_DATA_START in
// CMakeLists.txt; STM32H563: the last 256 KB, 32 sectors of 8 KB; STM32H743: bank 2, 1 MB in 8
// sectors of 128 KB - littlefs gives every file above 512 bytes blocks of its own: there only 5
// to 6 such missions fit), one file "/<index>.msn" per mission holding the
// encoded WsProtocol::roarm::Mission (name and steps). Same layout as the former firmware: the
// missions on a board survive the update. Owned by the I/O thread (littlefs has no locks).
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
