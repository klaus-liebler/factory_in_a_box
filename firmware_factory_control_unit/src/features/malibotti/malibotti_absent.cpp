// The Malibotti robot on a board without its connector (revisions 1 and 2): nothing to do, its
// registers stay 0 (src/hal/<revision>/hal.cmake lists this file instead of malibotti.cpp).
#include "malibotti/malibotti.hpp"

namespace fcu::malibotti {
void setup() noexcept {}
void cycle(std::uint32_t) noexcept {}
}  // namespace fcu::malibotti
