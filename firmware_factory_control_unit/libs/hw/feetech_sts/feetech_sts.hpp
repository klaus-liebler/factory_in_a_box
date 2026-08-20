#pragma once
// Minimaler, nicht-blockierender... eigentlich: kurz-blockierender (HAL_UART_Transmit/Receive
// mit knappem Timeout) Treiber fuer das Feetech-SMS/STS-Busservo-Protokoll (Dynamixel-1.0-artig:
// 0xFF 0xFF ID Length Instruction [Params...] ~Checksum), wie es die Feetech-STS3215-Servos des
// RoArm-M3 verwenden. Protokoll- und Speichertabellen-Konstanten 1:1 uebernommen aus zwei
// bereits verifizierten Quellen: der Speichertabelle in
// C:\repos\labathome\labathome_firmware_stm32arduino\src\busservo.hh (identisch mit der
// offiziellen SCServo-Bibliothek, s. https://github.com/parallax/scservo/blob/main/src/SMS_STS.h)
// und deren Frame-/Checksum-Logik (SCS::writeBuf/syncWrite/Read, s. dortiges SCS.cpp) --
// portiert von Arduino/HardwareSerial auf STM32-HAL (UART_HandleTypeDef*), sonst unveraendert.
//
// Halbduplex-Frage (per Konstruktor-Flag "consumeLoopback"): UART7 ist in dieser Firmware mit
// GETRENNTEN RX/TX-Pins konfiguriert (PE7/PE8, je eigene, unabhaengige AF-Push-Pull-Konfiguration
// in stm32h5xx_hal_msp.c -- kein Hinweis auf eine On-Chip-Halbduplex-Bruecke), daher Default
// consumeLoopback=false (echter Vollduplex-Betrieb: senden und direkt lesen, kein
// Pin-Umschalten noetig). FALLS die reale Platine TX/PE8 und RX/PE7 dennoch extern (z.B. per
// Widerstand nahe dem Steckverbinder) auf den EINEN Bus-Draht zu den Servos zusammenfuehrt --
// wie im obigen labathome-Beispiel, das genau dafuer gebaut wurde --, empfaengt dieser Treiber
// sonst sein eigenes Sendesignal als Echo VOR der eigentlichen Servo-Antwort und muss dieses
// Echo verwerfen; dafuer consumeLoopback=true setzen (dann werden txPin/txPinAlternateFunction
// gebraucht, um den TX-Pin waehrend des Empfangsfensters kurz auf Eingang umzuschalten) -- beim
// Hardware-Bring-up anhand des tatsaechlichen Schaltplans/Verhaltens verifizieren.
#include <cstdint>
#include <cstring>
#include "main.h"
#include "gpio.hh"

namespace feetech_sts {

enum class Instruction : uint8_t {
    Ping = 0x01,
    Read = 0x02,
    Write = 0x03,
    RegWrite = 0x04,
    Action = 0x05,
    FactoryReset = 0x06,
    SyncWrite = 0x83,
};

// Speichertabelle (SRAM-Bereich, read/write bzw. read-only) -- s. Quellenverweis oben.
namespace Memory {
constexpr uint8_t TorqueEnable = 40;
constexpr uint8_t Acc = 41;
constexpr uint8_t GoalPositionL = 42;
constexpr uint8_t GoalTimeL = 44;
constexpr uint8_t GoalSpeedL = 46;
constexpr uint8_t Lock = 55;
constexpr uint8_t PresentPositionL = 56;
constexpr uint8_t PresentSpeedL = 58;
constexpr uint8_t PresentLoadL = 60;
constexpr uint8_t PresentVoltage = 62;
constexpr uint8_t PresentTemperature = 63;
constexpr uint8_t Moving = 66;
constexpr uint8_t PresentCurrentL = 69;
// Present*L..PresentCurrentH liegen konsekutiv (56..70, mit zwei nicht dokumentierten
// Fuellbytes bei 64/65 und 67/68) -- ReadFeedback() liest sie in einem einzigen Read-Befehl.
constexpr uint8_t FeedbackBlockStart = PresentPositionL;
constexpr uint8_t FeedbackBlockLength = 15; // 56..70 inklusive
}

struct Feedback {
    bool commOk = false;
    int16_t position = 0;     // Vorzeichen+Betrag ueber Bit15 (s. DecodeSigned16), 0..4095 Normalbetrieb
    int16_t speed = 0;        // Vorzeichen+Betrag, Richtung der aktuellen Drehung
    int16_t load = 0;         // Vorzeichen+Betrag, aktuelle Drehmomentbelastung
    uint8_t voltage = 0;      // 0.1V-Einheiten
    uint8_t temperatureC = 0;
    bool moving = false;
    int16_t current = 0;
};

class FeetechStsBus {
public:
    FeetechStsBus(UART_HandleTypeDef* huart, bool consumeLoopback = false, gpio::Pin txPin = gpio::Pin::NO_PIN,
                  uint32_t txPinAlternateFunction = 0)
        : huart_(huart), consumeLoopback_(consumeLoopback), txPin_(txPin), txPinAlternateFunction_(txPinAlternateFunction) {}

    // Fuer einmalige Nachkonfiguration (z.B. Baudratenwechsel bei Setup(), s. roarm.hh) --
    // dieselbe Instanz bleibt danach unveraendert nutzbar.
    UART_HandleTypeDef* Uart() const { return huart_; }

    // Setzt Zielposition+Geschwindigkeit+Beschleunigung fuer ALLE Servos in einem einzigen
    // Sync-Write-Frame (Broadcast-ID 0xFE, keine Antwort laut Protokoll -- daher kein
    // Ruecklesen/Fehlercode hier). ids/positions/speeds muessen je 'count' Eintraege haben.
    bool SyncWritePositions(const uint8_t* ids, const int16_t* positions, const uint16_t* speeds, uint8_t accel, size_t count) {
        if (count == 0 || count > kMaxServos) return false;
        constexpr uint8_t kParamLen = 7; // Acc(1) + GoalPosition(2) + GoalTime(2, ungenutzt=0) + GoalSpeed(2)
        uint8_t payload[kMaxServos * (1 + kParamLen)];
        for (size_t i = 0; i < count; i++) {
            uint8_t* p = payload + i * (1 + kParamLen);
            p[0] = ids[i];
            uint16_t rawPos = EncodeSigned16(positions[i]);
            p[1] = accel;
            p[2] = static_cast<uint8_t>(rawPos & 0xFF);
            p[3] = static_cast<uint8_t>(rawPos >> 8);
            p[4] = 0; // GoalTime L (ungenutzt)
            p[5] = 0; // GoalTime H (ungenutzt)
            p[6] = static_cast<uint8_t>(speeds[i] & 0xFF);
            p[7] = static_cast<uint8_t>(speeds[i] >> 8);
        }
        return SyncWriteRaw(Memory::Acc, kParamLen, payload, count);
    }

    // Liest Position/Geschwindigkeit/Last/Spannung/Temperatur/Moving/Strom in einem Read-Befehl.
    bool ReadFeedback(uint8_t id, Feedback& out) {
        uint8_t data[Memory::FeedbackBlockLength];
        if (!ReadData(id, Memory::FeedbackBlockStart, data, sizeof(data))) {
            out.commOk = false;
            return false;
        }
        out.commOk = true;
        out.position = DecodeSigned16(data[0], data[1]);
        out.speed = DecodeSigned16(data[2], data[3]);
        out.load = DecodeSigned16(data[4], data[5]);
        out.voltage = data[6];
        out.temperatureC = data[7];
        // data[8],data[9] = Fuellbytes (64/65, nicht dokumentiert)
        out.moving = data[10] != 0;
        // data[11],data[12] = Fuellbytes (67/68, nicht dokumentiert)
        out.current = DecodeSigned16(data[13], data[14]);
        return true;
    }

    bool SetTorqueEnable(uint8_t id, bool enable) {
        uint8_t v = enable ? 1 : 0;
        return WriteData(id, Memory::TorqueEnable, &v, 1);
    }

    bool Ping(uint8_t id) {
        uint8_t frame[6] = {0xFF, 0xFF, id, 0x02, static_cast<uint8_t>(Instruction::Ping), 0};
        frame[5] = static_cast<uint8_t>(~(id + 0x02 + static_cast<uint8_t>(Instruction::Ping)));
        uint8_t rxParams[0];
        return SendAndReceive(id, frame, sizeof(frame), rxParams, 0);
    }

private:
    static constexpr size_t kMaxServos = 8;

    UART_HandleTypeDef* huart_;
    bool consumeLoopback_;
    gpio::Pin txPin_;
    uint32_t txPinAlternateFunction_;

    // Vorzeichen+Betrag ueber Bit15 (kein Zweierkomplement) -- Konvention aus WritePosEx() der
    // Referenzimplementierungen (labathome/busservo.hh, SCServo-Bibliothek).
    static uint16_t EncodeSigned16(int16_t value) {
        if (value >= 0) return static_cast<uint16_t>(value);
        return static_cast<uint16_t>((-static_cast<int32_t>(value)) | 0x8000);
    }
    static int16_t DecodeSigned16(uint8_t lo, uint8_t hi) {
        uint16_t raw = static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
        int16_t magnitude = static_cast<int16_t>(raw & 0x7FFF);
        return (raw & 0x8000) ? static_cast<int16_t>(-magnitude) : magnitude;
    }

    bool WriteData(uint8_t id, uint8_t memAddr, const uint8_t* data, uint8_t len) {
        uint8_t frame[4 + 2 + 32];
        size_t frameLen = BuildWriteFrame(frame, sizeof(frame), id, Instruction::Write, memAddr, data, len);
        if (frameLen == 0) return false;
        uint8_t rxParams[0];
        return SendAndReceive(id, frame, frameLen, rxParams, 0);
    }

    bool ReadData(uint8_t id, uint8_t memAddr, uint8_t* data, uint8_t len) {
        uint8_t params[2] = {memAddr, len};
        uint8_t frame[8];
        size_t frameLen = BuildWriteFrame(frame, sizeof(frame), id, Instruction::Read, 0xFF /*unused*/, nullptr, 0, params, sizeof(params));
        if (frameLen == 0) return false;
        return SendAndReceive(id, frame, frameLen, data, len);
    }

    // Baut ein WRITE/READ-Frame. Fuer READ wird 'explicitParams' (MemAddr+Laenge) direkt als
    // Payload genutzt (memAddr/data bleiben ungenutzt); fuer WRITE wird memAddr als erstes
    // Payload-Byte vorangestellt, gefolgt von 'data'. Deckt beide Faelle einheitlich ab, s.
    // Referenzimplementierung (dort zwei separate, fast identische Codepfade).
    static size_t BuildWriteFrame(uint8_t* frame, size_t frameCapacity, uint8_t id, Instruction inst, uint8_t memAddr, const uint8_t* data,
                                  uint8_t len, const uint8_t* explicitParams = nullptr, uint8_t explicitParamsLen = 0) {
        uint8_t params[34];
        uint8_t paramsLen;
        if (explicitParams) {
            if (explicitParamsLen > sizeof(params)) return 0;
            memcpy(params, explicitParams, explicitParamsLen);
            paramsLen = explicitParamsLen;
        } else {
            if (static_cast<size_t>(len) + 1 > sizeof(params)) return 0;
            params[0] = memAddr;
            if (data && len > 0) memcpy(params + 1, data, len);
            paramsLen = static_cast<uint8_t>(len + 1);
        }
        size_t total = 6 + paramsLen; // 0xFF 0xFF ID Len Inst + params + Checksum(1) -- Len selbst zaehlt hier nicht doppelt
        if (total > frameCapacity) return 0;

        uint8_t msgLen = static_cast<uint8_t>(paramsLen + 2);
        frame[0] = 0xFF;
        frame[1] = 0xFF;
        frame[2] = id;
        frame[3] = msgLen;
        frame[4] = static_cast<uint8_t>(inst);
        memcpy(frame + 5, params, paramsLen);
        uint8_t checksum = static_cast<uint8_t>(id + msgLen + static_cast<uint8_t>(inst));
        for (uint8_t i = 0; i < paramsLen; i++) checksum += params[i];
        frame[5 + paramsLen] = static_cast<uint8_t>(~checksum);
        return 5 + paramsLen + 1;
    }

    bool SyncWriteRaw(uint8_t memAddr, uint8_t paramLenPerServo, const uint8_t* payload, size_t count) {
        // payload: 'count' Eintraege je (1+paramLenPerServo) Bytes (ID gefolgt von den Daten).
        uint8_t frame[8 + kMaxServos * 9];
        uint8_t msgLen = static_cast<uint8_t>((paramLenPerServo + 1) * count + 4);
        size_t total = 7 + count * (1 + paramLenPerServo) + 1;
        if (total > sizeof(frame)) return false;

        frame[0] = 0xFF;
        frame[1] = 0xFF;
        frame[2] = 0xFE; // Broadcast
        frame[3] = msgLen;
        frame[4] = static_cast<uint8_t>(Instruction::SyncWrite);
        frame[5] = memAddr;
        frame[6] = paramLenPerServo;

        uint8_t checksum = static_cast<uint8_t>(0xFE + msgLen + static_cast<uint8_t>(Instruction::SyncWrite) + memAddr + paramLenPerServo);
        size_t pos = 7;
        for (size_t i = 0; i < count; i++) {
            const uint8_t* entry = payload + i * (1 + paramLenPerServo);
            memcpy(frame + pos, entry, 1 + paramLenPerServo);
            pos += 1 + paramLenPerServo;
            for (uint8_t j = 0; j < 1 + paramLenPerServo; j++) checksum += entry[j];
        }
        frame[pos++] = static_cast<uint8_t>(~checksum);

        TransmitFrame(frame, pos);
        // Broadcast-Schreiben bekommt laut Protokoll keine Antwort -- bei consumeLoopback muss
        // trotzdem das eigene Echo verworfen werden, damit es nicht die naechste Transaktion stoert.
        if (consumeLoopback_) ConsumeLoopback(pos);
        return true;
    }

    void TransmitFrame(const uint8_t* frame, size_t len) {
        if (consumeLoopback_ && txPin_ != gpio::Pin::NO_PIN) {
            gpio::Gpio::ConfigureAlternateFunction(txPin_, txPinAlternateFunction_);
        }
        HAL_UART_Transmit(huart_, const_cast<uint8_t*>(frame), static_cast<uint16_t>(len), 20);
        if (consumeLoopback_ && txPin_ != gpio::Pin::NO_PIN) {
            // TX-Pin waehrend des Empfangsfensters auf Eingang, damit die Servo-Antwort auf
            // demselben (extern zusammengefuehrten) Draht ankommen kann -- s. Klassenkommentar.
            gpio::Gpio::ConfigureGPIOInput(txPin_, gpio::PullDirection::UP);
        }
    }

    void ConsumeLoopback(size_t sentLen) {
        uint8_t scratch[40];
        size_t remaining = sentLen;
        while (remaining > 0) {
            size_t chunk = remaining > sizeof(scratch) ? sizeof(scratch) : remaining;
            if (HAL_UART_Receive(huart_, scratch, static_cast<uint16_t>(chunk), 5) != HAL_OK) break;
            remaining -= chunk;
        }
    }

    // Sendet 'txFrame' und liest -- falls rxParamsLen>0 oder eine Bestaetigung erwartet wird --
    // das Antwort-Frame (0xFF 0xFF ID Len Error [Params...] Checksum). Gibt false bei
    // Timeout/Prüfsummenfehler zurueck.
    bool SendAndReceive(uint8_t id, const uint8_t* txFrame, size_t txFrameLen, uint8_t* rxParams, uint8_t rxParamsLen) {
        TransmitFrame(txFrame, txFrameLen);
        if (consumeLoopback_) ConsumeLoopback(txFrameLen);

        uint8_t header[5];
        if (HAL_UART_Receive(huart_, header, sizeof(header), 15) != HAL_OK) return false;
        if (header[0] != 0xFF || header[1] != 0xFF || header[2] != id || header[3] != static_cast<uint8_t>(rxParamsLen + 2)) return false;

        uint8_t checksum = static_cast<uint8_t>(header[2] + header[3] + header[4]);
        if (rxParamsLen > 0) {
            if (HAL_UART_Receive(huart_, rxParams, rxParamsLen, 15) != HAL_OK) return false;
            for (uint8_t i = 0; i < rxParamsLen; i++) checksum += rxParams[i];
        }
        uint8_t rxChecksum;
        if (HAL_UART_Receive(huart_, &rxChecksum, 1, 15) != HAL_OK) return false;
        return static_cast<uint8_t>(~checksum) == rxChecksum;
    }
};

} // namespace feetech_sts
