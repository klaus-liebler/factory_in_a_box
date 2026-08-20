#pragma once
// Mission-Ablage im reservierten 256K-Flash-Bereich (s. STM32H563xx_FLASH.ld,
// roarm_mission_flash_bd.hh) ueber littlefs (libs/littlefs, vendort v2.11.3). Jede Mission ist
// eine Datei, benannt nach ihrem numerischen Index (z.B. "/7.msn"), deren Inhalt exakt die
// generierten WsProtocol::roarm::Mission-Bytes sind (Encode()/Decode() aus
// Core/generated/ws_protocol.hh) -- derselbe 4-Byte-Nachrichtenkopf dient zugleich als
// Datei-Magic, kein separates Speicherformat noetig (s. Plan Abschnitt 6).
#include <cstdint>
#include <cstddef>
#include "lfs.h"

namespace RoArmMissionStore {

constexpr size_t kMaxNameLength = 31;

struct MissionSummary {
    uint16_t index = 0;
    char name[kMaxNameLength + 1] = {0};
};

class Store {
public:
    // Mountet das Dateisystem; formatiert bei Bedarf (leerer/neuer Flash-Bereich) einmalig neu
    // und mountet danach erneut -- Standard-littlefs-Bootstrap-Pattern. false nur bei einem
    // Flash-I/O-Fehler (s. roarm_mission_flash_bd.cpp-Logs).
    bool Mount();

    // Schreibt bis zu 'outCapacity' Eintraege nach 'outList', liefert die tatsaechliche Anzahl
    // in 'outCount'. Nicht-numerisch benannte Dateien (sollte es keine geben) werden ignoriert.
    bool List(MissionSummary* outList, size_t outCapacity, size_t& outCount);

    // Liest die rohen, encodierten Mission-Bytes einer Datei nach 'outBuffer' (Kapazitaet
    // 'outCapacity'); 'outSize' = tatsaechlich gelesene Bytes. Decodierung (WsProtocol::roarm::
    // Mission::Decode) obliegt dem Aufrufer, da dessen zurueckgegebene Zeiger (name/stepsData)
    // erst dann gueltig sind, solange 'outBuffer' bestehen bleibt.
    bool Load(uint16_t missionIndex, uint8_t* outBuffer, size_t outCapacity, size_t& outSize);

    // 'name': wird intern auf kMaxNameLength gekuerzt. 'stepsData': bereits vorserialisierte
    // Schritt-Elemente (je [classId:u16][Klassenfelder], s. AppendMissionSteps*Element() in
    // ws_protocol.hh) -- diese Funktion baut daraus nur noch den Mission-Rahmen (Name+Anzahl)
    // und schreibt das Ergebnis als Datei.
    bool Save(uint16_t missionIndex, const char* name, const uint8_t* stepsData, size_t stepsDataSize, size_t stepsCount);

    bool Delete(uint16_t missionIndex);

private:
    lfs_t lfs_{};
    lfs_config config_{};
    bool mounted_ = false;

    static void IndexToFilename(uint16_t index, char* out, size_t outSize);
};

} // namespace RoArmMissionStore
