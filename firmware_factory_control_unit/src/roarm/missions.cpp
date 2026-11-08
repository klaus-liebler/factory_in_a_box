// RoArm mission store (see missions.hpp).
#include "roarm/missions.hpp"

#include <cstdio>
#include <cstring>

#include "board/fcu.hpp"
#include "rtos/log/log.hpp"
#if RTOS_FCU_REVISION == 3
#include "rtos/port/stm32h7/flash_program.hpp"
#else
#include "rtos/port/stm32h5/flash_program.hpp"
#endif
#include "rtos/storage/littlefs.hpp"
#include "ws_protocol.hh"

namespace fcu::roarm::missions {
namespace {

#if RTOS_FCU_REVISION == 3
// STM32H743: the whole bank 2, 1 MB in 8 sectors of 128 KB (FLASH_DATA_START 0x08100000)
using MissionFlash = rtos::port::stm32h7::flash::Region<0x0810'0000, 1024 * 1024, board::sysclk_hz>;
#else
// STM32H563: the last 256 KB of bank 2, 32 sectors of 8 KB (FLASH_DATA_START 0x081C0000)
using MissionFlash = rtos::port::stm32h5::flash::Region<0x081C'0000, 256 * 1024, board::sysclk_hz>;
#endif
MissionFlash flash;
rtos::storage::LittleFs<MissionFlash> fs{flash};
using File = rtos::storage::LittleFs<MissionFlash>::File;
File file;  // one at a time (the I/O thread)
std::array<std::uint8_t, max_mission_bytes> encode_scratch;

std::array<char, 16> filename(std::uint16_t index) noexcept {
    std::array<char, 16> f{};
    std::snprintf(f.data(), f.size(), "/%u.msn", static_cast<unsigned>(index));
    return f;
}

}  // namespace

bool mount() noexcept {
    if (fs.mount() == LFS_ERR_OK) {
        return true;
    }
    RTOS_LOG_WARN("missions: mount failed, formatting the flash region (first start or data loss)");
    return fs.format() == LFS_ERR_OK && fs.mount() == LFS_ERR_OK;
}

std::size_t list(std::span<MissionSummary> out) noexcept {
    if (!fs.mounted()) {
        return 0;
    }
    lfs_dir_t dir{};
    if (lfs_dir_open(fs.get(), &dir, "/") != LFS_ERR_OK) {
        return 0;
    }
    std::size_t n = 0;
    lfs_info info{};
    while (n < out.size() && lfs_dir_read(fs.get(), &dir, &info) > 0) {
        unsigned index = 0;
        if (info.type != LFS_TYPE_REG || std::sscanf(info.name, "%u.msn", &index) != 1) {
            continue;
        }
        std::array<std::uint8_t, max_name + 8> header{};
        if (fs.open(file, info.name, LFS_O_RDONLY) != LFS_ERR_OK) {
            continue;
        }
        const lfs_ssize_t got = lfs_file_read(fs.get(), file.get(), header.data(), header.size());
        lfs_file_close(fs.get(), file.get());
        WsProtocol::roarm::Mission::Payload p{};
        if (got < 0 || !WsProtocol::roarm::Mission::Decode(header.data(), static_cast<std::size_t>(got), p)) {
            continue;
        }
        out[n].index = static_cast<std::uint16_t>(index);
        std::strncpy(out[n].name.data(), p.name, max_name);
        ++n;
    }
    lfs_dir_close(fs.get(), &dir);
    return n;
}

std::size_t load(std::uint16_t index, std::span<std::uint8_t> out) noexcept {
    if (!fs.mounted() || fs.open(file, filename(index).data(), LFS_O_RDONLY) != LFS_ERR_OK) {
        return 0;
    }
    const lfs_ssize_t got = lfs_file_read(fs.get(), file.get(), out.data(), static_cast<lfs_size_t>(out.size()));
    lfs_file_close(fs.get(), file.get());
    return got > 0 ? static_cast<std::size_t>(got) : 0;
}

bool save(std::uint16_t index, std::string_view name, std::span<const std::uint8_t> steps, std::size_t step_count) noexcept {
    if (!fs.mounted()) {
        return false;
    }
    std::array<char, max_name + 1> n{};
    std::memcpy(n.data(), name.data(), std::min(name.size(), max_name));
    WsProtocol::roarm::Mission::Payload p{};
    p.name = n.data();
    p.stepsData = steps.data();
    p.stepsDataSize = steps.size();
    p.stepsCount = step_count;
    if (WsProtocol::roarm::Mission::EncodedSize(p) > encode_scratch.size()) {
        RTOS_LOG_ERROR("missions: mission %u too large", static_cast<unsigned>(index));
        return false;
    }
    const std::size_t size = WsProtocol::roarm::Mission::Encode(p, encode_scratch.data(), encode_scratch.size());
    if (size == 0 || fs.open(file, filename(index).data(), LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC) != LFS_ERR_OK) {
        return false;
    }
    const lfs_ssize_t written = lfs_file_write(fs.get(), file.get(), encode_scratch.data(), static_cast<lfs_size_t>(size));
    return lfs_file_close(fs.get(), file.get()) == LFS_ERR_OK && written == static_cast<lfs_ssize_t>(size);
}

bool remove(std::uint16_t index) noexcept {
    return fs.mounted() && lfs_remove(fs.get(), filename(index).data()) == LFS_ERR_OK;
}

}  // namespace fcu::roarm::missions
