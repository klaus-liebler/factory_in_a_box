// The SD card (J3, FAT12/16/32 - not exFAT: FileX), shared by the RoArm missions (/MISSIONS) and the
// PLC project (/PLC). Mounted at the first use and, if it is missing or fails, again at a later one (at
// most every 5 s): a card inserted later is found. Owned by the I/O thread (FileX has no locks).
#pragma once

#include <cstdint>

#include "fx_api.h"

namespace fcu::sd {

/// Mounted, or mounted now (a new attempt at most every 5 s). false: no usable card.
[[nodiscard]] bool mount() noexcept;

/// The FileX media; only after mount() returned true.
[[nodiscard]] FX_MEDIA* media() noexcept;

/// Counts the mounts: a user that prepares something per card (a directory, an import) compares it.
[[nodiscard]] std::uint32_t generation() noexcept;

/// The card's size in MB (after mount()).
[[nodiscard]] std::uint32_t megabytes() noexcept;

/// Writes FileX's caches to the card.
[[nodiscard]] bool flush() noexcept;

/// A file error that may mean the card is gone: mount again at the next use.
void lost(const char* who) noexcept;

/// FileX takes non-const names.
[[nodiscard]] inline CHAR* fx(const char* s) noexcept { return const_cast<CHAR*>(s); }

/// Creates `path` (an absolute directory) unless it exists.
[[nodiscard]] bool ensure_directory(const char* path) noexcept;

}  // namespace fcu::sd
