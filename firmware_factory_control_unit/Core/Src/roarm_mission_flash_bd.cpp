#include "roarm_mission_flash_bd.hh"

#include <cstring>

#include "main.h"
#include "log.h"

namespace RoArmMissionFlashBd {

namespace {

constexpr uint32_t kBlockSize = 0x2000;    // FLASH_SECTOR_SIZE (8K)
constexpr uint32_t kBlockCount = 32;       // 256K / 8K
constexpr uint32_t kProgSize = 16;         // Quad-Word (FLASH_TYPEPROGRAM_QUADWORD)
constexpr uint32_t kReadSize = 16;
constexpr uint32_t kCacheSize = 512;       // Vielfaches von kProgSize, Teiler von kBlockSize
constexpr uint32_t kLookaheadSize = 16;    // 128 Bloecke Bitmap-Kapazitaet, reicht fuer 32

extern "C" uint32_t _roarm_mission_flash_start; // Linker-Symbol, s. STM32H563xx_FLASH.ld

// Bank2 ist bank-relativ 128 Sektoren (je 8K) gross (FLASH_BANK_SIZE = 1MB) -- die reservierten
// 256K sind die letzten 32 Sektoren von Bank2, unabhaengig von der absoluten Adresse.
constexpr uint32_t kBank2FirstSector = 96;

uint8_t g_readBuffer[kCacheSize];
uint8_t g_progBuffer[kCacheSize];
uint8_t g_lookaheadBuffer[kLookaheadSize];

uint32_t RegionBase() { return reinterpret_cast<uint32_t>(&_roarm_mission_flash_start); }

int BdRead(const struct lfs_config*, lfs_block_t block, lfs_off_t off, void* buffer, lfs_size_t size) {
    if (block >= kBlockCount) return LFS_ERR_INVAL;
    const uint32_t addr = RegionBase() + block * kBlockSize + off;
    memcpy(buffer, reinterpret_cast<const void*>(addr), size);
    return LFS_ERR_OK;
}

int BdProg(const struct lfs_config*, lfs_block_t block, lfs_off_t off, const void* buffer, lfs_size_t size) {
    if (block >= kBlockCount || (size % kProgSize) != 0 || (off % kProgSize) != 0) return LFS_ERR_INVAL;
    uint32_t addr = RegionBase() + block * kBlockSize + off;
    const uint8_t* src = static_cast<const uint8_t*>(buffer);

    HAL_FLASH_Unlock();
    int result = LFS_ERR_OK;
    for (lfs_size_t written = 0; written < size; written += kProgSize) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, addr + written, reinterpret_cast<uint32_t>(src + written)) != HAL_OK) {
            log_error("roarm_mission_flash_bd: HAL_FLASH_Program fehlgeschlagen (addr=0x%08lx)", (unsigned long)(addr + written));
            result = LFS_ERR_IO;
            break;
        }
    }
    HAL_FLASH_Lock();
    return result;
}

int BdErase(const struct lfs_config*, lfs_block_t block) {
    if (block >= kBlockCount) return LFS_ERR_INVAL;

    FLASH_EraseInitTypeDef erase{};
    erase.TypeErase = FLASH_TYPEERASE_SECTORS; // TZEN=0 auf diesem Board, s. Header-Kommentar
    erase.Banks = FLASH_BANK_2;
    erase.Sector = kBank2FirstSector + block;
    erase.NbSectors = 1;
    uint32_t sectorError = 0;

    HAL_FLASH_Unlock();
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase, &sectorError);
    HAL_FLASH_Lock();

    if (status != HAL_OK) {
        log_error("roarm_mission_flash_bd: HAL_FLASHEx_Erase fehlgeschlagen (block=%lu, sectorError=0x%08lx)", (unsigned long)block,
                  (unsigned long)sectorError);
        return LFS_ERR_IO;
    }
    return LFS_ERR_OK;
}

int BdSync(const struct lfs_config*) { return LFS_ERR_OK; }

} // namespace

void InitConfig(lfs_config& config) {
    config = lfs_config{};
    config.read = BdRead;
    config.prog = BdProg;
    config.erase = BdErase;
    config.sync = BdSync;
    config.read_size = kReadSize;
    config.prog_size = kProgSize;
    config.block_size = kBlockSize;
    config.block_count = kBlockCount;
    config.block_cycles = 500;
    config.cache_size = kCacheSize;
    config.lookahead_size = kLookaheadSize;
    config.read_buffer = g_readBuffer;
    config.prog_buffer = g_progBuffer;
    config.lookahead_buffer = g_lookaheadBuffer;
}

} // namespace RoArmMissionFlashBd
