// Simulated SD card (see sd_card_sim.hpp).
#include "sd_card_sim.hpp"

#include <cstring>

#include "rtos/storage/filex_media.hpp"
#include "rtos/storage/sd_card.hpp"
#include "sd_card.hpp"

namespace fcu::sd {
namespace {

using rtos::storage::SdStatus;
using rtos::storage::sd_block_size;

/// A card in the slot: RAM blocks, "present" while inserted.
struct RamCard {
    std::vector<std::byte> data;
    bool present = false;

    SdStatus read(std::uint64_t block, std::span<std::byte> out) noexcept {
        if (!present) {
            return SdStatus::NoCard;
        }
        if ((block + out.size() / sd_block_size) * sd_block_size > data.size()) {
            return SdStatus::CardError;
        }
        std::memcpy(out.data(), data.data() + block * sd_block_size, out.size());
        return SdStatus::Ok;
    }
    SdStatus write(std::uint64_t block, std::span<const std::byte> in) noexcept {
        if (!present) {
            return SdStatus::NoCard;
        }
        if ((block + in.size() / sd_block_size) * sd_block_size > data.size()) {
            return SdStatus::CardError;
        }
        std::memcpy(data.data() + block * sd_block_size, in.data(), in.size());
        return SdStatus::Ok;
    }
    std::uint64_t blocks() const noexcept { return data.size() / sd_block_size; }
};
static_assert(rtos::storage::BlockDevice<RamCard>);

RamCard card;
rtos::storage::FileXMedia<RamCard> volume{card};
bool mounted = false;
std::uint32_t mounts = 0;

void unmount() noexcept {
    if (mounted) {
        (void)volume.close();
        mounted = false;
    }
}

}  // namespace

bool mount() noexcept {
    if (mounted) {
        return true;
    }
    if (!card.present || volume.open() != FX_SUCCESS) {
        return false;
    }
    mounted = true;
    ++mounts;
    return true;
}

FX_MEDIA* media() noexcept { return volume.media(); }

std::uint32_t generation() noexcept { return mounts; }

std::uint32_t megabytes() noexcept { return static_cast<std::uint32_t>(card.blocks() / 2048); }

bool flush() noexcept { return volume.flush() == FX_SUCCESS; }

void lost(const char*) noexcept { unmount(); }

bool ensure_directory(const char* path) noexcept {
    const UINT s = fx_directory_create(media(), fx(path));
    return s == FX_SUCCESS || s == FX_ALREADY_CREATED;
}

namespace sim {

void insert_blank_card(std::uint64_t blocks) noexcept {
    unmount();
    card.data.assign(blocks * sd_block_size, std::byte{0});
    card.present = true;
    (void)volume.format_fat("SIMCARD");
    (void)volume.close();  // format leaves the media open; mount() opens it like a fresh card
}

void remove_card() noexcept {
    unmount();
    card.present = false;
}

void reinsert_card() noexcept { card.present = true; }

bool write_file(const char* path, std::span<const std::uint8_t> content) noexcept {
    if (!mount()) {
        return false;
    }
    (void)fx_file_delete(media(), fx(path));
    if (fx_file_create(media(), fx(path)) != FX_SUCCESS) {
        return false;
    }
    FX_FILE f{};
    if (fx_file_open(media(), &f, fx(path), FX_OPEN_FOR_WRITE) != FX_SUCCESS) {
        return false;
    }
    const bool ok = fx_file_write(&f, const_cast<std::uint8_t*>(content.data()), static_cast<ULONG>(content.size())) ==
                    FX_SUCCESS;
    return fx_file_close(&f) == FX_SUCCESS && ok && flush();
}

}  // namespace sim
}  // namespace fcu::sd
