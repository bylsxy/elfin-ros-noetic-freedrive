#ifndef ELFIN_FREEDRIVE_CONTROLLER_TOOL_BUTTON_LOGIC_H
#define ELFIN_FREEDRIVE_CONTROLLER_TOOL_BUTTON_LOGIC_H

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace elfin_freedrive_controller {

class ToolButtonLogic {
public:
  static constexpr unsigned int kPointBit = 4;
  static constexpr unsigned int kFreeBit = 5;
  enum : unsigned int { kDefaultFreePressSamples = 8 };
  enum : unsigned int { kDefaultFreeReleaseSamples = 2 };

  static constexpr double minimumFreePressSeconds() { return 0.70; }
  static constexpr double defaultFreePressSeconds() { return 0.70; }
  static constexpr double defaultMaximumFreeLowGapSeconds() { return 0.15; }
  static constexpr double defaultFreeReleaseSeconds() { return 0.08; }

  struct Events {
    bool point_pressed = false;
    bool free_pressed = false;
    bool free_released = false;
    bool free_confirmed_held = false;
    bool free_short_pulse = false;
    unsigned int free_high_samples = 0;
    double free_high_seconds = 0.0;
  };

  explicit ToolButtonLogic(
      double required_free_press_seconds = defaultFreePressSeconds(),
      unsigned int required_free_press_samples = kDefaultFreePressSamples,
      double maximum_free_low_gap_seconds =
          defaultMaximumFreeLowGapSeconds(),
      double required_free_release_seconds = defaultFreeReleaseSeconds(),
      unsigned int required_free_release_samples =
          kDefaultFreeReleaseSamples)
      : required_free_press_seconds_(
            std::isfinite(required_free_press_seconds) &&
                    required_free_press_seconds > 0.0
                ? required_free_press_seconds
                : defaultFreePressSeconds()),
        required_free_press_samples_(
            required_free_press_samples == 0 ? 1 : required_free_press_samples),
        maximum_free_low_gap_seconds_(
            std::isfinite(maximum_free_low_gap_seconds) &&
                    maximum_free_low_gap_seconds >= 0.0
                ? maximum_free_low_gap_seconds
                : defaultMaximumFreeLowGapSeconds()),
        required_free_release_seconds_(
            std::isfinite(required_free_release_seconds) &&
                    required_free_release_seconds >= 0.0
                ? required_free_release_seconds
                : defaultFreeReleaseSeconds()),
        required_free_release_samples_(
            required_free_release_samples == 0 ? 1
                                               : required_free_release_samples),
        initialized_(false),
        sample_time_valid_(false),
        last_sample_seconds_(0.0),
        point_pressed_(false),
        free_sample_high_(false),
        free_armed_(false),
        free_triggered_(false),
        free_high_samples_(0),
        free_high_started_seconds_(0.0),
        free_low_samples_(0),
        free_low_started_seconds_(0.0) {}

  Events update(uint16_t raw_input, double monotonic_seconds) {
    if (!std::isfinite(monotonic_seconds) ||
        (sample_time_valid_ && monotonic_seconds < last_sample_seconds_)) {
      return inputUnavailable();
    }
    sample_time_valid_ = true;
    last_sample_seconds_ = monotonic_seconds;

    Events events;
    const bool point_now = ((raw_input >> kPointBit) & 0x1U) != 0;
    const bool free_now = ((raw_input >> kFreeBit) & 0x1U) != 0;

    // A manager restart while FREE is already held must never enter torque
    // mode. The first sample only establishes the baseline.
    if (!initialized_) {
      initialized_ = true;
      point_pressed_ = point_now;
      free_sample_high_ = free_now;
      free_armed_ = !free_now;
      free_triggered_ = false;
      free_high_samples_ = 0;
      free_high_started_seconds_ = monotonic_seconds;
      free_low_samples_ = 0;
      free_low_started_seconds_ = monotonic_seconds;
      return events;
    }

    events.point_pressed = point_now && !point_pressed_;
    if (free_now) {
      if (free_armed_) {
        if (!free_sample_high_) {
          if (free_high_samples_ == 0) {
            free_high_started_seconds_ = monotonic_seconds;
          }
          free_low_samples_ = 0;
        }
        if (!free_triggered_ &&
            free_high_samples_ < required_free_press_samples_) {
          ++free_high_samples_;
        }
        const double high_seconds =
            std::max(0.0, monotonic_seconds - free_high_started_seconds_);
        if (!free_triggered_ &&
            free_high_samples_ >= required_free_press_samples_ &&
            high_seconds + 1e-9 >= required_free_press_seconds_) {
          free_triggered_ = true;
          events.free_pressed = true;
        }
      }
      events.free_confirmed_held = free_triggered_;
    } else if (free_triggered_) {
      if (free_sample_high_) {
        free_low_samples_ = 1;
        free_low_started_seconds_ = monotonic_seconds;
      } else if (free_low_samples_ < required_free_release_samples_) {
        ++free_low_samples_;
      }
      const double low_seconds =
          std::max(0.0, monotonic_seconds - free_low_started_seconds_);
      if (free_low_samples_ >= required_free_release_samples_ &&
          low_seconds + 1e-9 >= required_free_release_seconds_) {
        events.free_released = true;
        free_armed_ = true;
        free_triggered_ = false;
        free_high_samples_ = 0;
        free_low_samples_ = 0;
      } else {
        events.free_confirmed_held = true;
      }
    } else if (free_armed_ && free_high_samples_ > 0) {
      if (free_sample_high_) {
        free_low_samples_ = 1;
        free_low_started_seconds_ = monotonic_seconds;
      } else {
        ++free_low_samples_;
      }
      const double low_seconds =
          std::max(0.0, monotonic_seconds - free_low_started_seconds_);
      if (low_seconds + 1e-9 >= maximum_free_low_gap_seconds_) {
        events.free_short_pulse = true;
        events.free_high_samples = free_high_samples_;
        events.free_high_seconds =
            std::max(0.0, free_low_started_seconds_ -
                              free_high_started_seconds_);
        free_high_samples_ = 0;
        free_low_samples_ = 0;
      }
    } else {
      free_armed_ = true;
      free_triggered_ = false;
      free_high_samples_ = 0;
      free_low_samples_ = 0;
    }

    point_pressed_ = point_now;
    free_sample_high_ = free_now;
    return events;
  }

  Events inputUnavailable() {
    Events events;
    events.free_released = free_triggered_;
    initialized_ = true;
    sample_time_valid_ = false;
    free_sample_high_ = false;
    free_armed_ = false;
    free_triggered_ = false;
    free_high_samples_ = 0;
    free_low_samples_ = 0;
    return events;
  }

  bool freePressed() const { return free_triggered_; }
  double requiredFreePressSeconds() const {
    return required_free_press_seconds_;
  }
  unsigned int requiredFreePressSamples() const {
    return required_free_press_samples_;
  }
  double maximumFreeLowGapSeconds() const {
    return maximum_free_low_gap_seconds_;
  }
  double requiredFreeReleaseSeconds() const {
    return required_free_release_seconds_;
  }

private:
  double required_free_press_seconds_;
  unsigned int required_free_press_samples_;
  double maximum_free_low_gap_seconds_;
  double required_free_release_seconds_;
  unsigned int required_free_release_samples_;
  bool initialized_;
  bool sample_time_valid_;
  double last_sample_seconds_;
  bool point_pressed_;
  bool free_sample_high_;
  bool free_armed_;
  bool free_triggered_;
  unsigned int free_high_samples_;
  double free_high_started_seconds_;
  unsigned int free_low_samples_;
  double free_low_started_seconds_;
};

}  // namespace elfin_freedrive_controller

#endif
