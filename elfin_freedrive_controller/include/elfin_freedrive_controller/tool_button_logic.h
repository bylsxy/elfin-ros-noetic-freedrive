#ifndef ELFIN_FREEDRIVE_CONTROLLER_TOOL_BUTTON_LOGIC_H
#define ELFIN_FREEDRIVE_CONTROLLER_TOOL_BUTTON_LOGIC_H

#include <cstdint>

namespace elfin_freedrive_controller {

class ToolButtonLogic {
public:
  static constexpr unsigned int kPointBit = 4;
  static constexpr unsigned int kFreeBit = 5;

  struct Events {
    bool point_pressed = false;
    bool free_pressed = false;
    bool free_released = false;
  };

  ToolButtonLogic()
      : initialized_(false),
        point_pressed_(false),
        free_pressed_(false),
        free_armed_(false),
        free_triggered_(false) {}

  Events update(uint16_t raw_input) {
    Events events;
    const bool point_now = ((raw_input >> kPointBit) & 0x1U) != 0;
    const bool free_now = ((raw_input >> kFreeBit) & 0x1U) != 0;

    // A manager restart while FREE is already held must never enter torque
    // mode. The first sample only establishes the baseline.
    if (!initialized_) {
      initialized_ = true;
      point_pressed_ = point_now;
      free_pressed_ = free_now;
      free_armed_ = !free_now;
      free_triggered_ = false;
      return events;
    }

    events.point_pressed = point_now && !point_pressed_;
    if (free_now && !free_pressed_ && free_armed_) {
      free_triggered_ = true;
      events.free_pressed = true;
    }
    if (!free_now && free_pressed_) {
      events.free_released = free_triggered_;
      free_armed_ = true;
      free_triggered_ = false;
    }

    point_pressed_ = point_now;
    free_pressed_ = free_now;
    return events;
  }

  bool freePressed() const { return free_pressed_; }

private:
  bool initialized_;
  bool point_pressed_;
  bool free_pressed_;
  bool free_armed_;
  bool free_triggered_;
};

}  // namespace elfin_freedrive_controller

#endif
