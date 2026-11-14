// USB Power Delivery of the board revision (src/hal/<revision>/power_delivery.cpp): revision 1
// negotiates with the MCU's UCPD sink, revisions 2/3 have a CH224K/CH224Q that does it alone
// (hw::mcu_power_delivery, hw::pd_controller in the revision's peripherals.hpp).
#pragma once

#include <optional>

namespace fcu::pd {

struct Status {
    bool connected = false;
    int active_mv = 0;
    int active_ma = 0;
};

/// Ask the source for `mv` (the MCU's sink starts); nothing where a CH224 decides by its CFG pins.
void request(int mv) noexcept;
/// The sink's events since the last call: logged. Nothing before request().
void poll() noexcept;
/// The negotiated contract; std::nullopt where the controller reports nothing to the MCU.
[[nodiscard]] std::optional<Status> status() noexcept;

}  // namespace fcu::pd
