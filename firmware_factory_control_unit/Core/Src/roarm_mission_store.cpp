#include "roarm_mission_store.hh"

#include <cstdio>
#include <cstring>
#include <cstdlib>

#include "roarm_mission_flash_bd.hh"
#include "generated/ws_protocol.hh"
#include "log.h"

namespace RoArmMissionStore {

namespace {
constexpr size_t kMaxMissionBytes = 4096; // s. Kommentar in Store::Save()
uint8_t g_encodeScratch[kMaxMissionBytes];
} // namespace

void Store::IndexToFilename(uint16_t index, char* out, size_t outSize) { snprintf(out, outSize, "/%u.msn", (unsigned)index); }

bool Store::Mount() {
    RoArmMissionFlashBd::InitConfig(config_);
    if (lfs_mount(&lfs_, &config_) == LFS_ERR_OK) {
        mounted_ = true;
        return true;
    }

    log_warn("roarm_mission_store: Mount fehlgeschlagen -- formatiere Flash-Bereich neu (erster Start oder Datenverlust)");
    if (lfs_format(&lfs_, &config_) != LFS_ERR_OK) {
        log_error("roarm_mission_store: lfs_format fehlgeschlagen");
        return false;
    }
    if (lfs_mount(&lfs_, &config_) != LFS_ERR_OK) {
        log_error("roarm_mission_store: Mount nach Format weiterhin fehlgeschlagen");
        return false;
    }
    mounted_ = true;
    return true;
}

bool Store::List(MissionSummary* outList, size_t outCapacity, size_t& outCount) {
    outCount = 0;
    if (!mounted_) return false;

    lfs_dir_t dir;
    if (lfs_dir_open(&lfs_, &dir, "/") != LFS_ERR_OK) return false;

    lfs_info info;
    int res;
    while ((res = lfs_dir_read(&lfs_, &dir, &info)) > 0) {
        if (info.type != LFS_TYPE_REG) continue;
        if (outCount >= outCapacity) break;

        unsigned index = 0;
        if (std::sscanf(info.name, "%u.msn", &index) != 1) continue;

        lfs_file_t file;
        if (lfs_file_open(&lfs_, &file, info.name, LFS_O_RDONLY) != LFS_ERR_OK) continue;
        uint8_t headerBuf[kMaxNameLength + 8];
        lfs_ssize_t read = lfs_file_read(&lfs_, &file, headerBuf, sizeof(headerBuf));
        lfs_file_close(&lfs_, &file);
        if (read < 0) continue;

        WsProtocol::roarm::Mission::Payload payload{};
        if (!WsProtocol::roarm::Mission::Decode(headerBuf, static_cast<size_t>(read), payload)) continue;

        MissionSummary& out = outList[outCount++];
        out.index = static_cast<uint16_t>(index);
        std::strncpy(out.name, payload.name, kMaxNameLength);
        out.name[kMaxNameLength] = '\0';
    }
    lfs_dir_close(&lfs_, &dir);
    return true;
}

bool Store::Load(uint16_t missionIndex, uint8_t* outBuffer, size_t outCapacity, size_t& outSize) {
    outSize = 0;
    if (!mounted_) return false;

    char filename[24];
    IndexToFilename(missionIndex, filename, sizeof(filename));

    lfs_file_t file;
    if (lfs_file_open(&lfs_, &file, filename, LFS_O_RDONLY) != LFS_ERR_OK) return false;
    lfs_ssize_t read = lfs_file_read(&lfs_, &file, outBuffer, static_cast<lfs_size_t>(outCapacity));
    lfs_file_close(&lfs_, &file);
    if (read < 0) return false;
    outSize = static_cast<size_t>(read);
    return true;
}

bool Store::Save(uint16_t missionIndex, const char* name, const uint8_t* stepsData, size_t stepsDataSize, size_t stepsCount) {
    if (!mounted_) return false;

    char truncatedName[kMaxNameLength + 1];
    std::strncpy(truncatedName, name, kMaxNameLength);
    truncatedName[kMaxNameLength] = '\0';

    WsProtocol::roarm::Mission::Payload payload{};
    payload.name = truncatedName;
    payload.stepsData = stepsData;
    payload.stepsCount = stepsCount;
    payload.stepsDataSize = stepsDataSize;

    const size_t needed = WsProtocol::roarm::Mission::EncodedSize(payload);
    if (needed > kMaxMissionBytes) {
        log_error("roarm_mission_store: Mission %u zu gross (%u > %u Bytes)", (unsigned)missionIndex, (unsigned)needed,
                  (unsigned)kMaxMissionBytes);
        return false;
    }
    const size_t encoded = WsProtocol::roarm::Mission::Encode(payload, g_encodeScratch, sizeof(g_encodeScratch));
    if (encoded == 0) return false;

    char filename[24];
    IndexToFilename(missionIndex, filename, sizeof(filename));

    lfs_file_t file;
    if (lfs_file_open(&lfs_, &file, filename, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC) != LFS_ERR_OK) return false;
    lfs_ssize_t written = lfs_file_write(&lfs_, &file, g_encodeScratch, static_cast<lfs_size_t>(encoded));
    int closeResult = lfs_file_close(&lfs_, &file);
    return written == static_cast<lfs_ssize_t>(encoded) && closeResult == LFS_ERR_OK;
}

bool Store::Delete(uint16_t missionIndex) {
    if (!mounted_) return false;
    char filename[24];
    IndexToFilename(missionIndex, filename, sizeof(filename));
    return lfs_remove(&lfs_, filename) == LFS_ERR_OK;
}

} // namespace RoArmMissionStore
