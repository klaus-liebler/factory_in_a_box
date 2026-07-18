#pragma once
// Fixer, von Hand gepflegter Teil des Modbus-Registermodells -- nur die Adress-Konstanten
// haengen von der Register-Map ab und werden per tools/generate-register-map.mjs aus
// register-map.json erzeugt (gleiches Muster wie stm32_libs/common_stm32/gpio/generated/*.inc).
// Die generierten Fragmente enthalten bewusst nur constexpr-Zeilen -- die umschliessenden
// namespace-Bloecke stehen hier, fest von Hand gepflegt. Diese Datei selbst -- Speicherlayout,
// Mutex-Handling, Zugriffsmethoden -- aendert sich nicht mit der Register-Map und wird deshalb
// nicht mitgeneriert.

#include <array>
#include <cstdint>
#include "tx_api.h"
#include "modbus_commons.hh"

namespace ModbusRegisters {

// ---------------------------------------------------------------------------
// Input-Register (FC04, read-only)
// ---------------------------------------------------------------------------
namespace Input {
#include "generated/register_input.inc"
} // namespace Input

// ---------------------------------------------------------------------------
// Holding-Register (FC03/06/16, read/write)
// ---------------------------------------------------------------------------
namespace Holding {
#include "generated/register_holding.inc"
} // namespace Holding

#include "generated/register_maxindex.inc"

} // namespace ModbusRegisters

// Registerspeicher fuer Holding- (FC03/06/16) und Input-Register (FC04), mit optional
// nachtraeglich aktivierbarem Thread-sicherem Zugriff (ThreadX-Mutex).
//
// Der Konstruktor erzeugt bewusst KEIN ThreadX-Objekt -- ModbusRegisterModel muss auch VOR
// tx_application_define() (also z.B. als globales/statisches Objekt, das schon vor main()
// konstruiert wird) sicher anlegbar sein, noch ohne jeden ThreadX-Kontext. Bis
// ArmForMultithreadingWithMutex() aufgerufen wird, sind GetXxxRegister()/SetXxxRegister() nicht
// gegen parallelen Zugriff geschuetzt -- unkritisch, solange bis dahin ohnehin nur ein einziger
// Ausfuehrungskontext existiert (z.B. waehrend io_setup() vor tx_kernel_enter()). Danach
// (fruehestens ab tx_application_define(), NICHT aus einer ISR heraus) einmalig
// ArmForMultithreadingWithMutex() aufrufen, sobald mehrere Threads parallel zugreifen koennen.
//
// Weder kopier- noch verschiebbar, sobald "armed": TX_MUTEX ist nach tx_mutex_create() nicht
// mehr sicher relozierbar (ThreadX verwaltet interne Verweise auf die Speicheradresse der
// Mutex-Kontrollstruktur, z.B. Suspend-Queues anderer Threads).
class ModbusRegisterModel : public Modbus::IModbusRegisterModel {
public:
    ModbusRegisterModel() = default;

    ModbusRegisterModel(const ModbusRegisterModel&) = delete;
    ModbusRegisterModel& operator=(const ModbusRegisterModel&) = delete;
    ModbusRegisterModel(ModbusRegisterModel&&) = delete;
    ModbusRegisterModel& operator=(ModbusRegisterModel&&) = delete;

    ~ModbusRegisterModel() override {
        if (mutex_armed_) {
            tx_mutex_delete(&mutex_);
        }
    }

    // Erzeugt den Schutz-Mutex nachtraeglich. Muss aus einem Kontext heraus aufgerufen werden,
    // in dem ThreadX-Objekterzeugung erlaubt ist (ab tx_application_define(), NICHT davor und
    // NICHT aus einer ISR). Gibt false zurueck, wenn bereits "armed" oder die Mutex-Erzeugung
    // fehlschlaegt.
    bool ArmForMultithreadingWithMutex() {
        if (mutex_armed_) {
            return false;
        }
        if (tx_mutex_create(&mutex_, const_cast<CHAR*>("Modbus Register Model Mutex"), TX_INHERIT) != TX_SUCCESS) {
            return false;
        }
        mutex_armed_ = true;
        return true;
    }

    uint16_t GetInputRegister(uint16_t addr) const override {
        return GetFromArray(input_registers_, addr);
    }

    uint16_t GetHoldingRegister(uint16_t addr) const override {
        return GetFromArray(holding_registers_, addr);
    }

    void SetInputRegister(uint16_t addr, uint16_t value) override {
        SetInArray(input_registers_, addr, value);
    }

    void SetHoldingRegister(uint16_t addr, uint16_t value) override {
        SetInArray(holding_registers_, addr, value);
    }

private:
    template <size_t N>
    uint16_t GetFromArray(const std::array<uint16_t, N>& regs, uint16_t addr) const {
        uint16_t value = 0;
        Lock();
        if (addr < regs.size()) {
            value = regs[addr];
        }
        Unlock();
        return value;
    }

    template <size_t N>
    void SetInArray(std::array<uint16_t, N>& regs, uint16_t addr, uint16_t value) {
        Lock();
        if (addr < regs.size()) {
            regs[addr] = value;
        }
        Unlock();
    }

    // No-op, solange noch nicht "armed" (s. Klassenkommentar) -- bewusst kein Fehlschlag/Assert
    // hier, da genau dieser Zustand (Zugriff vor ArmForMultithreadingWithMutex()) fuer den
    // fruehen Single-Context-Betrieb vorgesehen ist.
    void Lock() const {
        if (mutex_armed_) {
            tx_mutex_get(const_cast<TX_MUTEX*>(&mutex_), TX_WAIT_FOREVER);
        }
    }
    void Unlock() const {
        if (mutex_armed_) {
            tx_mutex_put(const_cast<TX_MUTEX*>(&mutex_));
        }
    }

    TX_MUTEX mutex_{};
    bool mutex_armed_ = false;
    std::array<uint16_t, ModbusRegisters::HOLDING_REGISTER_MAX_INDEX + 1> holding_registers_{};
    std::array<uint16_t, ModbusRegisters::INPUT_REGISTER_MAX_INDEX + 1> input_registers_{};
};

// Factory statt direkter Konstruktion an der Aufrufstelle (App::SetupBeforeThreadX()): das
// Modell muss auf dem Heap leben (App haelt nur einen Zeiger, referenziert von ModbusTcpServer/
// Io/den SetupAndLoop-Klassen ueber das gemeinsame Modbus::IModbusRegisterModel-Interface).
inline ModbusRegisterModel* BuildModbusRegisterModel() {
    return new ModbusRegisterModel();
}
