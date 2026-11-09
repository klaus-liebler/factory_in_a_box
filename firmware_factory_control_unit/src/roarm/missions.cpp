// RoArm mission store (see missions.hpp).
#include "roarm/missions.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>

#include "board/fcu.hpp"
#include "rtos/log/log.hpp"
#include "sd_card.hpp"
#include "system.hpp"
#include "ws_protocol.hh"
#if RTOS_FCU_REVISION != 3
#include "rtos/port/stm32h5/flash_program.hpp"
#include "rtos/storage/littlefs.hpp"
#endif

namespace fcu::roarm::missions {
namespace {

using sd::fx;

std::uint32_t prepared = 0;  ///< the card (sd::generation()) the directory was set up on
constexpr const char* directory = "/MISSIONS";

std::array<std::uint8_t, max_mission_bytes> scratch;  // encoded mission (save, import)
std::array<CHAR, FX_MAX_LONG_NAME_LEN + 1> entry_name;

std::array<char, 16> filename(std::uint16_t index, const char* extension = "msn") noexcept {
    std::array<char, 16> f{};
    std::snprintf(f.data(), f.size(), "%u.%s", static_cast<unsigned>(index), extension);
    return f;
}

/// "<n>.msn" (any case: FAT keeps 8.3 names upper case) -> n.
bool parse_index(const char* name, unsigned& index) noexcept {
    index = 0;
    const char* p = name;
    for (; *p >= '0' && *p <= '9'; ++p) {
        index = index * 10 + static_cast<unsigned>(*p - '0');
        if (index > 0xFFFF) {
            return false;
        }
    }
    const auto lower = [](char c) { return c >= 'A' && c <= 'Z' ? static_cast<char>(c - 'A' + 'a') : c; };
    return p != name && p[0] == '.' && lower(p[1]) == 'm' && lower(p[2]) == 's' && lower(p[3]) == 'n' && p[4] == '\0';
}

/// A file error: the card may be gone - mount again on the next request.
void lost() noexcept { sd::lost("missions"); }

bool write_file(const char* path, std::span<const std::uint8_t> data) noexcept {
    FX_MEDIA* m = sd::media();
    (void)fx_file_delete(m, fx(path));
    FX_FILE f{};
    if (fx_file_create(m, fx(path)) != FX_SUCCESS || fx_file_open(m, &f, fx(path), FX_OPEN_FOR_WRITE) != FX_SUCCESS) {
        return false;
    }
    const bool ok = fx_file_write(&f, const_cast<std::uint8_t*>(data.data()), static_cast<ULONG>(data.size())) == FX_SUCCESS;
    return fx_file_close(&f) == FX_SUCCESS && ok;
}

/// Replaces "<index>.msn" by `data`: written as "<index>.tmp" first, renamed once complete, so a
/// power loss leaves the old mission or the new one.
bool store(std::uint16_t index, std::span<const std::uint8_t> data) noexcept {
    const auto tmp = filename(index, "tmp");
    const auto path = filename(index);
    FX_MEDIA* m = sd::media();
    if (!write_file(tmp.data(), data)) {
        return false;
    }
    (void)fx_file_delete(m, fx(path.data()));
    return fx_file_rename(m, fx(tmp.data()), fx(path.data())) == FX_SUCCESS && sd::flush();
}

#if RTOS_FCU_REVISION != 3
bool exists(std::uint16_t index) noexcept {
    FX_FILE f{};
    if (fx_file_open(sd::media(), &f, fx(filename(index).data()), FX_OPEN_FOR_READ) != FX_SUCCESS) {
        return false;
    }
    (void)fx_file_close(&f);
    return true;
}

// ---- import from the internal flash (STM32H563 boards: the missions were in littlefs) ------------
using MissionFlash = rtos::port::stm32h5::flash::Region<0x081C'0000, 256 * 1024, board::sysclk_hz>;
MissionFlash flash;
rtos::storage::LittleFs<MissionFlash> lfs{flash};
rtos::storage::LittleFs<MissionFlash>::File lfs_file;
bool imported = false;

/// Moves every mission of the flash store onto the card (once per start; a mission already on
/// the card wins), then removes it from the flash: the card is the only store from then on.
void import_from_flash() noexcept {
    if (imported) {
        return;
    }
    imported = true;
    if (lfs.mount() != LFS_ERR_OK) {
        return;  // never formatted: nothing to import
    }
    unsigned moved = 0, kept = 0, left = 0;
    lfs_dir_t dir{};
    if (lfs_dir_open(lfs.get(), &dir, "/") == LFS_ERR_OK) {
        lfs_info info{};
        while (lfs_dir_read(lfs.get(), &dir, &info) > 0) {
            unsigned index = 0;
            if (info.type != LFS_TYPE_REG || !parse_index(info.name, index)) {
                continue;
            }
            std::array<char, LFS_NAME_MAX + 2> path{};
            std::snprintf(path.data(), path.size(), "/%s", info.name);
            if (exists(static_cast<std::uint16_t>(index))) {
                ++kept;
            } else {
                if (lfs.open(lfs_file, path.data(), LFS_O_RDONLY) != LFS_ERR_OK) {
                    ++left;
                    continue;
                }
                const lfs_ssize_t got = lfs_file_read(lfs.get(), lfs_file.get(), scratch.data(), scratch.size());
                lfs_file_close(lfs.get(), lfs_file.get());
                if (got <= 0 || !store(static_cast<std::uint16_t>(index), {scratch.data(), static_cast<std::size_t>(got)})) {
                    RTOS_LOG_WARN("missions: mission %u not copied to the SD card, stays in the flash", index);
                    ++left;
                    continue;
                }
                ++moved;
            }
            (void)lfs_remove(lfs.get(), path.data());
        }
        lfs_dir_close(lfs.get(), &dir);
    }
    (void)lfs.unmount();
    if (moved + kept > 0) {
        RTOS_LOG_INFO("missions: %u moved from the flash to the SD card, %u already there", moved, kept);
    }
    if (left == 0) {
        // nothing of the old store is needed any more: erasing its superblocks (blocks 0 and 1)
        // frees the area for the PLC's program slots (src/plc.cpp refuses to write while they exist)
        if (flash.erase(0) && flash.erase(1)) {
            RTOS_LOG_INFO("missions: the old mission store in the flash is removed; the area belongs to the PLC");
        }
    }
}
#endif

/// The card mounted (sd::mount()) and, once per card, the mission directory set up: it becomes the
/// default directory, the mission files are relative names in it (other users of the card take
/// absolute paths).
bool ensure_mounted() noexcept {
    if (!sd::mount()) {
        return false;
    }
    if (prepared == sd::generation()) {
        return true;
    }
    if (!sd::ensure_directory(directory)) {
        RTOS_LOG_ERROR("missions: %s not created on the SD card", directory);
        lost();
        return false;
    }
    if (fx_directory_default_set(sd::media(), fx(directory)) != FX_SUCCESS || !sd::flush()) {
        lost();
        return false;
    }
    prepared = sd::generation();
    RTOS_LOG_INFO("missions: in %s on the SD card", directory);
#if RTOS_FCU_REVISION != 3
    import_from_flash();
#endif
    return true;
}

}  // namespace

bool mount() noexcept { return ensure_mounted(); }

std::size_t list(std::span<MissionSummary> out) noexcept {
    if (!ensure_mounted()) {
        return 0;
    }
    // first the indices (opening files during the directory search would disturb it), then the names
    FX_MEDIA* m = sd::media();
    std::size_t found = 0;
    UINT attributes = 0, year = 0, month = 0, day = 0, hour = 0, minute = 0, second = 0;
    ULONG size = 0;
    for (UINT s = fx_directory_first_full_entry_find(m, entry_name.data(), &attributes, &size, &year, &month, &day, &hour,
                                                     &minute, &second);
         s == FX_SUCCESS && found < out.size();
         s = fx_directory_next_full_entry_find(m, entry_name.data(), &attributes, &size, &year, &month, &day, &hour,
                                               &minute, &second)) {
        unsigned index = 0;
        if ((attributes & FX_DIRECTORY) == 0 && parse_index(reinterpret_cast<const char*>(entry_name.data()), index)) {
            out[found++].index = static_cast<std::uint16_t>(index);
        }
    }
    std::size_t n = 0;
    for (std::size_t i = 0; i < found; ++i) {
        const std::uint16_t index = out[i].index;
        std::array<std::uint8_t, max_name + 8> header{};
        const std::size_t got = load(index, header);  // the name comes first
        WsProtocol::roarm::Mission::Payload p{};
        if (got == 0 || !WsProtocol::roarm::Mission::Decode(header.data(), got, p)) {
            continue;
        }
        out[n] = MissionSummary{};
        out[n].index = index;
        std::strncpy(out[n].name.data(), p.name, max_name);
        ++n;
    }
    std::sort(out.begin(), out.begin() + static_cast<std::ptrdiff_t>(n),
              [](const MissionSummary& a, const MissionSummary& b) { return a.index < b.index; });
    return n;
}

std::size_t load(std::uint16_t index, std::span<std::uint8_t> out) noexcept {
    if (!ensure_mounted()) {
        return 0;
    }
    FX_FILE f{};
    const UINT s = fx_file_open(sd::media(), &f, fx(filename(index).data()), FX_OPEN_FOR_READ);
    if (s != FX_SUCCESS) {
        if (s != FX_NOT_FOUND) {
            lost();
        }
        return 0;
    }
    ULONG got = 0;
    const UINT r = fx_file_read(&f, out.data(), static_cast<ULONG>(out.size()), &got);
    (void)fx_file_close(&f);
    if (r != FX_SUCCESS && r != FX_END_OF_FILE) {
        lost();
        return 0;
    }
    return got;
}

bool save(std::uint16_t index, std::string_view name, std::span<const std::uint8_t> steps, std::size_t step_count) noexcept {
    if (!ensure_mounted()) {
        return false;
    }
    std::array<char, max_name + 1> n{};
    std::memcpy(n.data(), name.data(), std::min(name.size(), max_name));
    WsProtocol::roarm::Mission::Payload p{};
    p.name = n.data();
    p.stepsData = steps.data();
    p.stepsDataSize = steps.size();
    p.stepsCount = step_count;
    if (WsProtocol::roarm::Mission::EncodedSize(p) > scratch.size()) {
        RTOS_LOG_ERROR("missions: mission %u too large", static_cast<unsigned>(index));
        return false;
    }
    const std::size_t size = WsProtocol::roarm::Mission::Encode(p, scratch.data(), scratch.size());
    if (size == 0) {
        return false;
    }
    if (!store(index, {scratch.data(), size})) {
        lost();
        return false;
    }
    return true;
}

bool remove(std::uint16_t index) noexcept {
    if (!ensure_mounted()) {
        return false;
    }
    const UINT s = fx_file_delete(sd::media(), fx(filename(index).data()));
    if (s == FX_SUCCESS) {
        return sd::flush();
    }
    if (s != FX_NOT_FOUND) {
        lost();
    }
    return false;
}

}  // namespace fcu::roarm::missions
