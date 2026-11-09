// The SD card (see sd_card.hpp).
#include "sd_card.hpp"

#include "board/fcu.hpp"
#include "hardware.hpp"
#include "rtos/log/log.hpp"
#include "rtos/storage/filex_media.hpp"
#include "rtos/storage/sd_card.hpp"
#include "system.hpp"

namespace fcu::sd {
namespace {

using rtos::storage::SdStatus;
using Card = rtos::storage::SdCard<hw::SdHost>;

hw::SdHost sd_host;
Card card{sd_host};
/// The FileX sector cache and bounce sector: where SDMMC1's IDMA reaches (STM32H743: the
/// non-cacheable AXI SRAM window).
RTOS_SDMMC_BUFFER rtos::storage::FileXMedia<Card> volume{card};
bool mounted = false;
bool board_ready = false;
bool attempted = false;
rtos::Tick last_attempt = 0;
std::uint32_t mounts = 0;
constexpr rtos::Tick retry_ms = 5'000;  ///< a missing card is looked for again at most this often

}  // namespace

bool mount() noexcept {
    if (mounted) {
        return true;
    }
    const rtos::Tick now = kernel.now();
    if (attempted && now - last_attempt < retry_ms) {
        return false;
    }
    attempted = true;
    last_attempt = now;
    if (!board_ready) {
        board::init_sdcard();
        board_ready = true;
    }
    if (const SdStatus s = card.init(); s != SdStatus::Ok) {
        RTOS_LOG_WARN("sd: no SD card (%s)", s == SdStatus::NoCard ? "none inserted" : "does not answer");
        return false;
    }
    if (const UINT s = volume.open(); s != FX_SUCCESS) {
        RTOS_LOG_ERROR("sd: the SD card has no FAT file system (error 0x%02X; exFAT cards need FAT32)", s);
        return false;
    }
    mounted = true;
    ++mounts;
    RTOS_LOG_INFO("sd: SD card mounted, %lu MB", static_cast<unsigned long>(megabytes()));
    return true;
}

FX_MEDIA* media() noexcept { return volume.media(); }

std::uint32_t generation() noexcept { return mounts; }

std::uint32_t megabytes() noexcept { return static_cast<std::uint32_t>(card.blocks() / 2048); }

bool flush() noexcept { return volume.flush() == FX_SUCCESS; }

void lost(const char* who) noexcept {
    if (mounted) {
        RTOS_LOG_WARN("%s: SD card error, card removed?", who);
        (void)volume.close();
        mounted = false;
    }
}

bool ensure_directory(const char* path) noexcept {
    const UINT s = fx_directory_create(media(), fx(path));
    return s == FX_SUCCESS || s == FX_ALREADY_CREATED;
}

}  // namespace fcu::sd
