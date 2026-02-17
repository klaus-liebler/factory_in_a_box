#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include "gpio.hh"
#include "sigmoid_acceleration_profile_100_1000_5.hh"
#include "sigmoid_stepper.hh"
#include "log.h"
#include "stm32g431xx.h"

static void run_motion(SigmoidStepper &stepper, int32_t target, int32_t log_stride) {
  uint64_t time = 0;
  log_debug("gotoPosition target=%ld", target);
  stepper.gotoPosition(target);

  const int32_t max_ticks = target + 1000;
  for (int32_t tick = 0; tick < max_ticks && stepper.isMotionActive(); ++tick) {
    stepper.handle_update_interrupt_();
    
    if (log_stride > 0 && (tick % log_stride) == 0) {
      log_debug("time=%ld pos=%ld", time, stepper.getCurrentPosition());
    }
    time += static_cast<uint64_t>(SIGMOID_STEPPER_TIMER->ARR + 1);
  }

  log_debug("done pos=%ld target=%ld active=%d time=%lu",
            stepper.getCurrentPosition(),
            stepper.getTargetPosition(),
            stepper.isMotionActive() ? 1 : 0,
            time);
}

int main() {
  const SigmoidAccelerationProfile *profile = &profile_100_1000_5;

  log_debug("StartArr16=%u EndArr16=%u TotalSteps=%lu",
            profile->getLowSpeedARR16(),
            profile->speed_to_arr(profile->getHighSpeedStepsPerSecond()),
            static_cast<unsigned long>(profile->getTotalSteps()));

  gpio::Pin step{gpio::Pin::PA08};
  gpio::Pin dir(gpio::Pin::PA09);
  SigmoidStepper stepper(step, dir, profile);
  stepper.init();

  const int32_t full_target =
      static_cast<int32_t>(profile->getTotalSteps() + 200);
  run_motion(stepper, full_target, 200);

  const int32_t short_target =
      static_cast<int32_t>(stepper.getCurrentPosition() + (profile->getTotalSteps() / 4));
  run_motion(stepper, short_target, 100);

  return 0;
}
