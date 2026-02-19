#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>

#include "gpio.hh"
#include "mocks/stm32g431xx.h"
#include "sigmoid_acceleration_profile_100_1000_5.hh"
#include "sigmoid_stepper.hh"
#include "log.h"
#include "stm32g431xx.h"

const SigmoidAccelerationProfile *profile = &profile_100_1000_5;

static void run_motion(SigmoidStepper &stepper,
                       int32_t target,
                       int32_t log_stride,
                       const char *csv_path,
                       bool echo_stdout = true) {
  uint64_t time_us = 0;
  stepper.gotoPosition(target);

  const int32_t max_ticks = target + 1000;
  std::ofstream csv_file(csv_path, std::ios::out | std::ios::trunc);
  if (csv_file) {
    csv_file << "Tick;Time(ms);Position;Target;Active;Speed\n";
  }
  if (echo_stdout) {
    printf("Tick;Time(ms);Position;Target;Active;Speed\n");
  }
  for (int32_t tick = 0; tick < max_ticks && stepper.isMotionActive(); ++tick) {
    int32_t current_pos = stepper.getCurrentPosition();
    double time_ms = time_us / 1000.0;

    // Calculate speed from ARR value: speed = TimerFrequency / (ARR + 1)
    double speed = static_cast<double>(profile->getTimerFrequency()) /
                   (static_cast<double>(stepper.getCurrentARR()) + 1.0);

    if (csv_file) {
      csv_file << tick << ';'
               << time_ms << ';'
               << current_pos << ';'
               << target << ';'
               << stepper.isMotionActive() << ';'
               << speed << '\n';
    }
    if (echo_stdout) {
      printf("%d;%.3f;%ld;%ld;%d;%.2f\n",
             tick,
             time_ms,
             current_pos,
             target,
             stepper.isMotionActive(),
             speed);
    }

    stepper.handle_update_interrupt_();
    time_us += (1000000ULL * (static_cast<uint64_t>(TIM17_Instance.ARR) + 1)) / profile->getTimerFrequency();

  }

  double time_ms = time_us / 1000.0;
  log_debug("done pos=%ld target=%ld active=%d time=%.3f",
            stepper.getCurrentPosition(),
            stepper.getTargetPosition(),
            stepper.isMotionActive() ? 1 : 0,
            time_ms);
}

int main() {
  

  log_debug("StartArr16=%u EndArr16=%u TotalSteps=%lu",
            profile->getLowSpeedARR16(),
            profile->speed_to_arr(profile->getHighSpeedStepsPerSecond()),
            static_cast<unsigned long>(profile->getTotalSteps()));

  gpio::Pin step{gpio::Pin::PA08};
  gpio::Pin dir(gpio::Pin::PA09);
  SigmoidStepper stepper(step, dir, profile);
  stepper.init();

  const int32_t full_target =
      static_cast<int32_t>(2*profile->getTotalSteps() + 200);
  const char *full_csv_path = "test/motion_full.csv";
  run_motion(stepper, full_target, 200, full_csv_path);

  const int32_t short_target =
      static_cast<int32_t>(stepper.getCurrentPosition() + (profile->getTotalSteps() / 4));
  const char *short_csv_path = "test/motion_short.csv";
  run_motion(stepper, short_target, 100, short_csv_path);

  // Plot CSVs using Python/Matplotlib if available on PATH.
  const std::string plot_cmd = std::string("py test/plot_csv.py ") +
                               full_csv_path + " " + short_csv_path;
  const int plot_rc = std::system(plot_cmd.c_str());
  if (plot_rc != 0) {
    log_debug("Plot command failed with rc=%d. Ensure Python and Matplotlib are installed.", plot_rc);
  }

  return 0;
}
