// USB-PD of FCU revision 2: the CH224K asks for 20 V by its CFG1 pin and reports nothing to the
// MCU - the firmware sees the result on the INA226 only (boot.cpp).
#include "power_delivery.hpp"

namespace fcu::pd {

void request(int) noexcept {}
void poll() noexcept {}
std::optional<Status> status() noexcept { return std::nullopt; }

}  // namespace fcu::pd
