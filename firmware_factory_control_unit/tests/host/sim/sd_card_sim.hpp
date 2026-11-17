// Simulated SD card for the host tests: implements src/sd_card.hpp (fcu::sd) over a FileX RAM disk
// (rtos::storage::FileXMedia, like rtoos' filex_media_test) instead of SDMMC + rtos::storage::SdCard.
// The test controls the card through fcu::sd::sim.
//
// Differences to src/sd_card.cpp: no "at most every 5 s" retry throttling (mount() tries at every
// call), and megabytes() reports the RAM disk's size.
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace fcu::sd::sim {

/// A new, empty, FAT-formatted card is inserted (the previous one is gone).
void insert_blank_card(std::uint64_t blocks = 16384) noexcept;

/// The card is pulled out: mount() fails, an already mounted volume is closed (like after
/// sd::lost()).
void remove_card() noexcept;

/// The same card again (keeps its contents).
void reinsert_card() noexcept;

/// Writes `content` to `path` on the card (mounting it if necessary) -- e.g. a file in an old format.
[[nodiscard]] bool write_file(const char* path, std::span<const std::uint8_t> content) noexcept;

}  // namespace fcu::sd::sim
