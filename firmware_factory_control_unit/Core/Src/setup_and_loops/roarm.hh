#pragma once
// ============================================================================
// RoArm-M3 -- Servo-Bus (Feetech SMS/STS ueber UART7, s. feetech_sts.hpp) + Kinematik
// (roarm_kinematics.hh) + beschleunigungsbegrenzte Bewegungsglaettung (roarm_motion.hh) +
// Mission-Wiedergabe (roarm_mission_store.hh/littlefs) + Mission-GPIO (roarm_mission_gpio.hh),
// zusammengefuehrt nach demselben ISetupAndLoop-Muster wie stepper.hh.
//
// Gelenk-/Servo-Zuordnung (7 Servos fuer 6 logische DOF -- Schulter hat einen Differential-
// Doppelservo fuer die Parallelogramm-Mechanik, s. Core/RoArm-M3_example/RoArm-M3_module.h
// RoArmM3_shoulderJointCtrlRad(): "goalPos[1]=MIDDLE+computePos; goalPos[2]=MIDDLE-computePos" --
// beide Servos bekommen bei jeder Bewegung ein Sync-Write-Kommando, DRIVEN spiegelt DRIVING exakt
// gegenlaeufig um denselben Mittelpunkt). Positions-/Feedback-Umrechnungsformeln (Vorzeichen,
// Nullpunkt-Offsets je Gelenk) 1:1 aus derselben Referenzdatei uebernommen
// (RoArmM3_baseJointCtrlRad/_shoulderJointCtrlRad/_elbowJointCtrlRad/_wristJointCtrlRad/
// _rollJointCtrlRad/_handJointCtrlRad sowie calculateRadByFeedback()) -- NICHT gegen die reale
// Hardware verifiziert, wie der uebrige RoArm-Code auch (s. Plan-Kontext).
//
// UART-Flexibilitaet: der Konstruktor nimmt einen UART_HandleTypeDef* entgegen (wie
// tmc2209::TMC2209 in stepper.hh) -- welches UART tatsaechlich verwendet wird, entscheidet sich
// erst in io.hpp. Aktuell vorgesehen: UART7 (PE7/PE8, "ROBOT_RX"/"ROBOT_TX" laut .ioc).
// ============================================================================
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <type_traits>

#include "tx_api.h"
#include "main.h"
#include "log.h"
#include "interfaces.hh"
#include "modbus_register_model.hh"
#include "feetech_sts.hpp"
#include "roarm_kinematics.hh"
#include "roarm_motion.hh"
#include "roarm_mission_gpio.hh"
#include "roarm_mission_store.hh"
#include "generated/ws_protocol.hh"

// RoArmSetupAndLoop wird von ZWEI Threads angefasst: Loop()/Setup() laufen auf dem gemeinsamen
// io_thread (s. Io::Loop()), waehrend die Jog-/Teach-Modus-/Mission-API von einem (spaeteren)
// WebSocket-Handler auf dem nx_tcpserver-Thread aufgerufen wird (s. webserver.cpp). Ein eigener
// TX_MUTEX schuetzt daher den kompletten Zustand (Tracker, Missions-Zustandsmaschine, littlefs) --
// analog zu ModbusRegisterModel::Lock()/Unlock() (modbus_register_model.hh), nur ohne dessen
// "noch nicht armed"-Sonderfall: RoArmSetupAndLoop existiert ausschliesslich als Member von Io,
// das selbst erst innerhalb von tx_application_define() konstruiert wird (s. app.cc) -- also
// immer schon in einem fuer ThreadX-Objekterzeugung gueltigen Kontext.
class RoArmMutexGuard {
public:
    explicit RoArmMutexGuard(TX_MUTEX& mutex) : mutex_(mutex) { tx_mutex_get(&mutex_, TX_WAIT_FOREVER); }
    ~RoArmMutexGuard() { tx_mutex_put(&mutex_); }
    RoArmMutexGuard(const RoArmMutexGuard&) = delete;
    RoArmMutexGuard& operator=(const RoArmMutexGuard&) = delete;

private:
    TX_MUTEX& mutex_;
};

class RoArmSetupAndLoop : public ISetupAndLoop {
public:
    RoArmSetupAndLoop(Modbus::IModbusRegisterModel& model, UART_HandleTypeDef* huart)
        : register_model(model),
          servoBus_(huart, /*consumeLoopback=*/false),
          tracker_(RestPoseRad(), kMaxVelocityRadPerSec, kMaxAccelerationRadPerSec2) {
        tx_mutex_create(&mutex_, const_cast<CHAR*>("RoArm Mutex"), TX_INHERIT);
    }

    void Setup() override {
        // UART7 kommt per CubeMX mit 115200 (main.c MX_UART7_Init()) -- fuer den Feetech-Bus auf
        // 1.000.000 Baud umkonfigurieren, ohne die .ioc anzufassen/neu zu generieren (s.
        // Plan-Abschnitt 1). MspInit wird durch HAL_UART_Init() intern erneut aufgerufen (GPIO-
        // Konfiguration ist idempotent).
        huart_ = servoBus_.Uart();
        huart_->Init.BaudRate = 1000000;
        if (HAL_UART_DeInit(huart_) != HAL_OK || HAL_UART_Init(huart_) != HAL_OK) {
            log_error("RoArm: UART7-Reinit auf 1.000.000 Baud fehlgeschlagen -- Servobus bleibt auf 115200");
        }

        if (!missionStore_.Mount()) {
            log_error("RoArm: Mission-Store (littlefs) konnte nicht gemountet werden");
        }
    }

    void Loop(uint32_t now) override {
        RoArmMutexGuard guard(mutex_);
        double dt = (lastTickMs_ == 0) ? 0.05 : (static_cast<double>(now - lastTickMs_) / 1000.0);
        lastTickMs_ = now;
        if (!(dt > 0.0) || dt > 1.0) dt = 0.05; // Erster Aufruf/Tick-Ueberlauf: Fallback

        ProcessMissionControlRegister();
        if (missionState_ == MissionState::Running) TickMission(now);

        tracker_.Tick(dt);
        SendServoPositions();
        UpdateFeedbackAndStatusRegisters();
    }

    // --- API fuer den (spaeteren) WebSocket-Handler in webserver.cpp -- jede Methode nimmt
    // mutex_ (s. Klassenkommentar), da sie vom nx_tcpserver-Thread aus aufgerufen wird, waehrend
    // Loop() parallel auf io_thread laeuft. ---

    bool StartTeachMode() {
        RoArmMutexGuard guard(mutex_);
        if (missionState_ != MissionState::Idle) return false;
        teachModeActive_ = true;
        return true;
    }
    void StopTeachMode() {
        RoArmMutexGuard guard(mutex_);
        teachModeActive_ = false;
    }
    bool IsTeachModeActive() {
        RoArmMutexGuard guard(mutex_);
        return teachModeActive_;
    }

    void SetJointJogTargetCentiDeg(const std::array<int16_t, RoArmKinematics::kJointCount>& centiDeg) {
        RoArmMutexGuard guard(mutex_);
        if (!teachModeActive_ || missionState_ != MissionState::Idle) return;
        std::array<double, RoArmKinematics::kJointCount> rad{};
        for (int i = 0; i < RoArmKinematics::kJointCount; i++) rad[i] = RoArmKinematics::ClampJointRad(i, RoArmKinematics::CentiDegToRad(centiDeg[i]));
        tracker_.SetTargets(rad);
    }

    void SetCartesianJogTarget(const RoArmKinematics::CartesianPose& pose) {
        RoArmMutexGuard guard(mutex_);
        if (!teachModeActive_ || missionState_ != MissionState::Idle) return;
        auto ik = RoArmKinematics::InverseKinematics(pose);
        if (!ik.isReachable) return;
        for (int i = 0; i < RoArmKinematics::kJointCount; i++) ik.jointsRad[i] = RoArmKinematics::ClampJointRad(i, ik.jointsRad[i]);
        tracker_.SetTargets(ik.jointsRad);
    }

    WsProtocol::roarm::PoseFeedback::Payload GetPoseFeedback() {
        RoArmMutexGuard guard(mutex_);
        WsProtocol::roarm::PoseFeedback::Payload payload{};
        const auto jointsRad = tracker_.GetCurrentRad();
        for (int i = 0; i < RoArmKinematics::kJointCount; i++) payload.jointAnglesCentiDeg[i] = RoArmKinematics::RadToCentiDeg(jointsRad[i]);
        const auto pose = RoArmKinematics::ForwardKinematics(jointsRad);
        payload.xMm = static_cast<int16_t>(std::lround(pose.xMm));
        payload.yMm = static_cast<int16_t>(std::lround(pose.yMm));
        payload.zMm = static_cast<int16_t>(std::lround(pose.zMm));
        payload.pitchCentiDeg = RoArmKinematics::RadToCentiDeg(pose.pitchRad);
        payload.rollCentiDeg = RoArmKinematics::RadToCentiDeg(pose.rollRad);
        for (int i = 0; i < 7; i++) payload.servoStatus[i] = static_cast<WsProtocol::roarm::ServoStatusBits>(lastServoStatus_[i]);
        return payload;
    }

    // Mission-Verwaltung: gesperrte Weiterleitung an roarm_mission_store (littlefs ist nicht von
    // sich aus thread-sicher, s. Klassenkommentar zu mutex_) -- das eigentliche (De-)Serialisieren
    // der Mission::Payload/Schritt-Elemente obliegt dem Aufrufer (WS-Handler), da nur der das
    // generierte Wire-Format (ws_protocol.hh) direkt gegen Request/Response-Typen abbilden muss.
    bool ListMissions(RoArmMissionStore::MissionSummary* outList, size_t outCapacity, size_t& outCount) {
        RoArmMutexGuard guard(mutex_);
        return missionStore_.List(outList, outCapacity, outCount);
    }
    bool LoadMissionRaw(uint16_t missionIndex, uint8_t* outBuffer, size_t outCapacity, size_t& outSize) {
        RoArmMutexGuard guard(mutex_);
        return missionStore_.Load(missionIndex, outBuffer, outCapacity, outSize);
    }
    bool SaveMission(uint16_t missionIndex, const char* name, const uint8_t* stepsData, size_t stepsDataSize, size_t stepsCount) {
        RoArmMutexGuard guard(mutex_);
        return missionStore_.Save(missionIndex, name, stepsData, stepsDataSize, stepsCount);
    }
    bool DeleteMission(uint16_t missionIndex) {
        RoArmMutexGuard guard(mutex_);
        return missionStore_.Delete(missionIndex);
    }

private:
    enum class MissionState { Idle, Running };
    enum class StepKind { JointMove, Gpio, Delay };
    struct RuntimeStep {
        StepKind kind;
        std::array<double, RoArmKinematics::kJointCount> jointsRad{};
        double maxVelocityRadPerSec = kMaxVelocityRadPerSec;
        uint8_t gpioId = 0;
        bool gpioState = false;
        uint32_t delayMs = 0;
    };
    static constexpr size_t kMaxStepsPerMission = 64;

    // Servo-IDs (RoArm-M3_config.h) -- Reihenfolge fuer SyncWrite, s. SendServoPositions().
    static constexpr uint8_t kServoBase = 11;
    static constexpr uint8_t kServoShoulderDriving = 12;
    static constexpr uint8_t kServoShoulderDriven = 13;
    static constexpr uint8_t kServoElbow = 14;
    static constexpr uint8_t kServoWrist = 15;
    static constexpr uint8_t kServoRoll = 16;
    static constexpr uint8_t kServoGripper = 17;
    static constexpr std::array<uint8_t, 7> kAllServoIds = {kServoBase,  kServoShoulderDriving, kServoShoulderDriven, kServoElbow,
                                                             kServoWrist, kServoRoll,            kServoGripper};

    static constexpr int16_t kServoMiddle = 2047;
    static constexpr double kServoCountsPerRad = 4096.0 / (2.0 * M_PI);

    // Nicht gegen die reale Hardware verifiziert, s. Nur wenn hier nichts kommentiert wird: 1:1
    // aus RoArm-M3_module.h uebernommen (RoArmM3_*JointCtrlRad()).
    static int16_t RadToCounts(double rad) { return static_cast<int16_t>(std::lround(rad * kServoCountsPerRad)); }
    static int16_t BasePosFromRad(double rad) { return static_cast<int16_t>(kServoMiddle + RadToCounts(-std::clamp(rad, -M_PI, M_PI))); }
    static int16_t ShoulderDrivingPosFromRad(double rad) {
        return static_cast<int16_t>(kServoMiddle + RadToCounts(std::clamp(rad, -M_PI / 2, M_PI / 2)));
    }
    static int16_t ShoulderDrivenPosFromRad(double rad) {
        return static_cast<int16_t>(kServoMiddle - RadToCounts(std::clamp(rad, -M_PI / 2, M_PI / 2)));
    }
    static int16_t ElbowPosFromRad(double rad) { return std::clamp<int16_t>(static_cast<int16_t>(RadToCounts(rad) + 1024), 1024, 3071); }
    static int16_t WristPosFromRad(double rad) {
        return static_cast<int16_t>(kServoMiddle + RadToCounts(std::clamp(rad, -M_PI / 2, M_PI / 2)));
    }
    static int16_t RollPosFromRad(double rad) { return static_cast<int16_t>(kServoMiddle - RadToCounts(std::clamp(rad, -M_PI, M_PI))); }
    static int16_t GripperPosFromRad(double rad) { return std::clamp<int16_t>(RadToCounts(rad), 700, 3396); }

    static double BaseRadFromCounts(int16_t steps) { return -(static_cast<double>(steps) * 2.0 * M_PI / 4096.0) + M_PI; }
    static double ShoulderRadFromCounts(int16_t steps) { return (static_cast<double>(steps) * 2.0 * M_PI / 4096.0) - M_PI; }
    static double ElbowRadFromCounts(int16_t steps) { return (static_cast<double>(steps) * 2.0 * M_PI / 4096.0) - M_PI / 2; }
    static double WristRadFromCounts(int16_t steps) { return (static_cast<double>(steps) * 2.0 * M_PI / 4096.0) - M_PI; }
    static double RollRadFromCounts(int16_t steps) { return -(static_cast<double>(steps) * 2.0 * M_PI / 4096.0) + M_PI; }
    static double GripperRadFromCounts(int16_t steps) { return static_cast<double>(steps) * 2.0 * M_PI / 4096.0; }

    // Ruhe-/Anfangspose: Gelenkwinkel 0 fuer Base/Shoulder/Elbow/Wrist/Roll entsprechen jeweils
    // der Servo-Mittelstellung (s. obige *PosFromRad()-Formeln) -- Gripper 0 rad = Servo-Position
    // 0, NICHT die "geschlossen"-Stellung der Referenz (dort initG=pi) -- als sicherer,
    // konservativer Default gewaehlt, bis eine reale Greifer-Kalibrierung feststeht.
    static std::array<double, RoArmKinematics::kJointCount> RestPoseRad() { return {0, 0, 0, 0, 0, 0}; }

    static constexpr double kMaxVelocityRadPerSec = (90.0 * M_PI) / 180.0;
    static constexpr double kMaxAccelerationRadPerSec2 = (180.0 * M_PI) / 180.0;

    TX_MUTEX mutex_{}; // s. Klassenkommentar bei RoArmMutexGuard

    Modbus::IModbusRegisterModel& register_model;
    feetech_sts::FeetechStsBus servoBus_;
    UART_HandleTypeDef* huart_ = nullptr;
    RoArmMotion::MultiJointTracker tracker_;
    RoArmMissionStore::Store missionStore_;

    bool teachModeActive_ = false;
    uint32_t lastTickMs_ = 0;
    std::array<uint8_t, 7> lastServoStatus_{}; // WsProtocol::roarm::ServoStatusBits je Servo

    MissionState missionState_ = MissionState::Idle;
    int16_t runningMissionIndex_ = 0;
    std::array<RuntimeStep, kMaxStepsPerMission> steps_{};
    size_t stepCount_ = 0;
    size_t stepIndex_ = 0;
    uint32_t stepStartMs_ = 0;
    uint8_t missionLoadBuffer_[4096];

    void SendServoPositions() {
        const auto jointsRad = tracker_.GetCurrentRad();
        std::array<uint8_t, 7> ids = kAllServoIds;
        std::array<int16_t, 7> positions = {
            BasePosFromRad(jointsRad[RoArmKinematics::Base]),
            ShoulderDrivingPosFromRad(jointsRad[RoArmKinematics::Shoulder]),
            ShoulderDrivenPosFromRad(jointsRad[RoArmKinematics::Shoulder]),
            ElbowPosFromRad(jointsRad[RoArmKinematics::Elbow]),
            WristPosFromRad(jointsRad[RoArmKinematics::Wrist]),
            RollPosFromRad(jointsRad[RoArmKinematics::Roll]),
            GripperPosFromRad(jointsRad[RoArmKinematics::Gripper]),
        };
        std::array<uint16_t, 7> speeds{}; // 0 = servo-eigene Standardgeschwindigkeit; Glaettung
                                           // uebernimmt bereits roarm_motion.hh (s. Klassenkommentar).
        servoBus_.SyncWritePositions(ids.data(), positions.data(), speeds.data(), /*accel=*/20, ids.size());
    }

    void UpdateFeedbackAndStatusRegisters() {
        feetech_sts::Feedback fb{};
        bool anyError = false;
        for (size_t i = 0; i < kAllServoIds.size(); i++) {
            bool ok = servoBus_.ReadFeedback(kAllServoIds[i], fb);
            uint8_t status = static_cast<uint8_t>(WsProtocol::roarm::ServoStatusBits::Ok);
            if (!ok) {
                status = static_cast<uint8_t>(WsProtocol::roarm::ServoStatusBits::CommError);
                anyError = true;
            } else if (fb.temperatureC > 70) {
                status = static_cast<uint8_t>(WsProtocol::roarm::ServoStatusBits::Overheat);
            } else if (std::abs(fb.load) > 900) {
                status = static_cast<uint8_t>(WsProtocol::roarm::ServoStatusBits::Overload);
            } else if (fb.voltage < 60) {
                status = static_cast<uint8_t>(WsProtocol::roarm::ServoStatusBits::Undervoltage);
            }
            lastServoStatus_[i] = status;
        }

        const auto jointsRad = tracker_.GetCurrentRad();
        ModbusRegisters::Accessors::RoArmStatus::RoArmJoint1AngleCentiDeg_Write(register_model,
                                                                                 RoArmKinematics::RadToCentiDeg(jointsRad[0]));
        ModbusRegisters::Accessors::RoArmStatus::RoArmJoint2AngleCentiDeg_Write(register_model,
                                                                                 RoArmKinematics::RadToCentiDeg(jointsRad[1]));
        ModbusRegisters::Accessors::RoArmStatus::RoArmJoint3AngleCentiDeg_Write(register_model,
                                                                                 RoArmKinematics::RadToCentiDeg(jointsRad[2]));
        ModbusRegisters::Accessors::RoArmStatus::RoArmJoint4AngleCentiDeg_Write(register_model,
                                                                                 RoArmKinematics::RadToCentiDeg(jointsRad[3]));
        ModbusRegisters::Accessors::RoArmStatus::RoArmJoint5AngleCentiDeg_Write(register_model,
                                                                                 RoArmKinematics::RadToCentiDeg(jointsRad[4]));
        ModbusRegisters::Accessors::RoArmStatus::RoArmJoint6AngleCentiDeg_Write(register_model,
                                                                                 RoArmKinematics::RadToCentiDeg(jointsRad[5]));

        uint16_t statusBits = 0;
        statusBits |= (1u << 0); // TorqueEnabled -- Servos werden aktuell dauerhaft bestromt, kein separates Enable-Register
        if (tracker_.IsMoving()) statusBits |= (1u << 1);
        if (anyError) statusBits |= (1u << 2);
        if (teachModeActive_) statusBits |= (1u << 3);
        ModbusRegisters::Accessors::RoArmStatus::Status_Write(register_model, statusBits);
    }

    // Registerkonvention (s. register_map_schema/RoArmCtrl.cs): >0 = Mission laeuft (Wert =
    // Index), 0 = Leerlauf/erfolgreich, <0 = Leerlauf/Fehlercode -Wert. Client darf nur
    // schreiben, wenn aktueller Wert <=0 -- durchgesetzt per Kanten-Erkennung (wie
    // STEPPER1_START_HOMING in stepper.hh): faengt eine laufende Mission einen neuen Schreibwert
    // ab, der NICHT der eigene laufende Index ist, wird dieser im selben Tick zurueckgeschrieben.
    void ProcessMissionControlRegister() {
        const int16_t regValue = ModbusRegisters::Accessors::RoArmCtrl::RoArmMissionControl_Read(register_model);

        if (missionState_ == MissionState::Running) {
            if (regValue != runningMissionIndex_) {
                ModbusRegisters::Accessors::RoArmCtrl::RoArmMissionControl_Write(register_model, runningMissionIndex_);
            }
            return;
        }

        if (regValue <= 0) return; // Leerlauf, nichts zu tun

        if (teachModeActive_) {
            log_warn("RoArm: Mission %d abgelehnt -- Teach-Modus aktiv", (int)regValue);
            ModbusRegisters::Accessors::RoArmCtrl::RoArmMissionControl_Write(register_model, -2);
            return;
        }
        StartMission(regValue);
    }

    void StartMission(int16_t missionIndex) {
        size_t rawSize = 0;
        if (!missionStore_.Load(static_cast<uint16_t>(missionIndex), missionLoadBuffer_, sizeof(missionLoadBuffer_), rawSize)) {
            log_warn("RoArm: Mission %d nicht gefunden", (int)missionIndex);
            ModbusRegisters::Accessors::RoArmCtrl::RoArmMissionControl_Write(register_model, -1);
            return;
        }

        WsProtocol::roarm::Mission::Payload payload{};
        if (!WsProtocol::roarm::Mission::Decode(missionLoadBuffer_, rawSize, payload)) {
            log_error("RoArm: Mission %d Decode fehlgeschlagen (Datei beschaedigt?)", (int)missionIndex);
            ModbusRegisters::Accessors::RoArmCtrl::RoArmMissionControl_Write(register_model, -3);
            return;
        }

        stepCount_ = 0;
        bool decodeOk = WsProtocol::roarm::DecodeMissionStepsElements(
            payload.stepsData, payload.stepsDataSize, payload.stepsCount, [this](auto&& item) {
                if (stepCount_ >= kMaxStepsPerMission) return; // stillschweigend abschneiden, s. Header-Kommentar
                using T = std::decay_t<decltype(item)>;
                RuntimeStep& step = steps_[stepCount_++];
                if constexpr (std::is_same_v<T, WsProtocol::roarm::JointMoveStep::Payload>) {
                    step.kind = StepKind::JointMove;
                    for (int i = 0; i < RoArmKinematics::kJointCount; i++) {
                        step.jointsRad[i] = RoArmKinematics::CentiDegToRad(item.jointAnglesCentiDeg[i]);
                    }
                    step.maxVelocityRadPerSec = item.maxSpeedDegPerSec > 0 ? (item.maxSpeedDegPerSec * M_PI / 180.0) : kMaxVelocityRadPerSec;
                } else if constexpr (std::is_same_v<T, WsProtocol::roarm::GpioStep::Payload>) {
                    step.kind = StepKind::Gpio;
                    step.gpioId = item.gpioId;
                    step.gpioState = item.state;
                } else if constexpr (std::is_same_v<T, WsProtocol::roarm::DelayStep::Payload>) {
                    step.kind = StepKind::Delay;
                    step.delayMs = item.durationMs;
                }
            });

        if (!decodeOk || stepCount_ == 0) {
            log_error("RoArm: Mission %d hat keine (decodierbaren) Schritte", (int)missionIndex);
            ModbusRegisters::Accessors::RoArmCtrl::RoArmMissionControl_Write(register_model, -3);
            return;
        }

        missionState_ = MissionState::Running;
        runningMissionIndex_ = missionIndex;
        stepIndex_ = 0;
        stepStartMs_ = HAL_GetTick();
        BeginCurrentStep();
        ModbusRegisters::Accessors::RoArmCtrl::RoArmMissionControl_Write(register_model, missionIndex);
        ModbusRegisters::Accessors::RoArmStatus::RoArmActiveMission_Write(register_model, missionIndex);
    }

    void BeginCurrentStep() {
        const RuntimeStep& step = steps_[stepIndex_];
        stepStartMs_ = HAL_GetTick();
        if (step.kind == StepKind::JointMove) {
            tracker_.SetTargets(step.jointsRad);
        } else if (step.kind == StepKind::Gpio) {
            if (!RoArmMissionGpio::Set(step.gpioId, step.gpioState)) {
                log_warn("RoArm: Mission-GPIO-Schritt uebersprungen (gpioId=%u ohne zugewiesenen Pin, s. roarm_mission_gpio.hh)",
                         (unsigned)step.gpioId);
            }
        }
        // Delay: nichts zu tun, Fortschritt wird in TickMission() rein zeitbasiert erkannt.
    }

    void TickMission(uint32_t now) {
        const RuntimeStep& step = steps_[stepIndex_];
        bool stepDone = false;
        switch (step.kind) {
        case StepKind::JointMove:
            stepDone = !tracker_.IsMoving();
            break;
        case StepKind::Gpio:
            stepDone = true; // einmaliger Schaltvorgang, s. BeginCurrentStep()
            break;
        case StepKind::Delay:
            stepDone = (now - stepStartMs_) >= step.delayMs;
            break;
        }
        if (!stepDone) return;

        stepIndex_++;
        if (stepIndex_ >= stepCount_) {
            missionState_ = MissionState::Idle;
            ModbusRegisters::Accessors::RoArmCtrl::RoArmMissionControl_Write(register_model, 0);
            ModbusRegisters::Accessors::RoArmStatus::RoArmActiveMission_Write(register_model, 0);
            return;
        }
        BeginCurrentStep();
    }
};
