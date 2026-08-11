#include <gtest/gtest.h>

#include <elfin_freedrive_controller/tool_button_logic.h>

namespace {
using elfin_freedrive_controller::ToolButtonLogic;

uint16_t bit(unsigned int index) {
  return static_cast<uint16_t>(1U << index);
}
}  // namespace

TEST(ToolButtonLogicTest, StartupWhileFreeHeldDoesNotTrigger) {
  ToolButtonLogic logic;
  const ToolButtonLogic::Events startup =
      logic.update(bit(ToolButtonLogic::kFreeBit), 0.0);
  EXPECT_FALSE(startup.free_pressed);
  EXPECT_FALSE(startup.free_confirmed_held);
  for (unsigned int sample = 1;
       sample < ToolButtonLogic::kDefaultFreePressSamples + 2; ++sample) {
    const ToolButtonLogic::Events held_at_startup =
        logic.update(bit(ToolButtonLogic::kFreeBit), 0.1 * sample);
    EXPECT_FALSE(held_at_startup.free_pressed);
    EXPECT_FALSE(held_at_startup.free_confirmed_held);
  }
  EXPECT_FALSE(logic.update(0, 1.1).free_released);
  logic.update(0, 1.3);
  for (unsigned int sample = 1;
       sample < ToolButtonLogic::kDefaultFreePressSamples; ++sample) {
    EXPECT_FALSE(logic.update(bit(ToolButtonLogic::kFreeBit),
                              2.0 + 0.1 * (sample - 1)).free_pressed);
  }
  EXPECT_TRUE(logic.update(bit(ToolButtonLogic::kFreeBit), 2.7).free_pressed);
}

TEST(ToolButtonLogicTest, IncidentLengthPulseNeverRequestsTorqueMode) {
  ToolButtonLogic logic;
  logic.update(0, 0.0);
  for (unsigned int sample = 0; sample < 6; ++sample) {
    EXPECT_FALSE(logic.update(bit(ToolButtonLogic::kFreeBit),
                              1.0 + 0.1 * sample).free_pressed);
  }
  const ToolButtonLogic::Events released = logic.update(0, 1.591680765);
  EXPECT_FALSE(released.free_short_pulse);
  const ToolButtonLogic::Events settled_low = logic.update(0, 1.8);
  EXPECT_FALSE(settled_low.free_released);
  EXPECT_TRUE(settled_low.free_short_pulse);
  EXPECT_EQ(settled_low.free_high_samples, 6U);
  EXPECT_NEAR(settled_low.free_high_seconds, 0.591680765, 1e-9);
}

TEST(ToolButtonLogicTest, CallbackCatchupCannotShortenHoldTime) {
  ToolButtonLogic logic;
  logic.update(0, 0.0);
  for (unsigned int sample = 0;
       sample < ToolButtonLogic::kDefaultFreePressSamples; ++sample) {
    EXPECT_FALSE(logic.update(bit(ToolButtonLogic::kFreeBit),
                              1.0 + 0.01 * sample).free_pressed);
  }
  const ToolButtonLogic::Events released = logic.update(0, 1.08);
  EXPECT_FALSE(released.free_short_pulse);
  const ToolButtonLogic::Events settled_low = logic.update(0, 1.30);
  EXPECT_TRUE(settled_low.free_short_pulse);
  EXPECT_EQ(settled_low.free_high_samples,
            logic.requiredFreePressSamples());
  EXPECT_NEAR(settled_low.free_high_seconds, 0.08, 1e-12);
}

TEST(ToolButtonLogicTest, OneLowSampleDoesNotBreakARealLongPress) {
  ToolButtonLogic logic;
  logic.update(0, 0.0);
  EXPECT_FALSE(logic.update(bit(ToolButtonLogic::kFreeBit), 1.0).free_pressed);
  EXPECT_FALSE(logic.update(bit(ToolButtonLogic::kFreeBit), 1.1).free_pressed);
  EXPECT_FALSE(logic.update(bit(ToolButtonLogic::kFreeBit), 1.2).free_pressed);
  EXPECT_FALSE(logic.update(0, 1.3).free_short_pulse);
  for (double stamp : {1.4, 1.5, 1.6, 1.7}) {
    EXPECT_FALSE(
        logic.update(bit(ToolButtonLogic::kFreeBit), stamp).free_pressed);
  }
  const ToolButtonLogic::Events confirmed =
      logic.update(bit(ToolButtonLogic::kFreeBit), 1.8);
  EXPECT_TRUE(confirmed.free_pressed);
  EXPECT_TRUE(confirmed.free_confirmed_held);
}

TEST(ToolButtonLogicTest, SustainedLowResetsAnInterruptedPress) {
  ToolButtonLogic logic;
  logic.update(0, 0.0);
  for (double stamp : {1.0, 1.1, 1.2, 1.3}) {
    EXPECT_FALSE(
        logic.update(bit(ToolButtonLogic::kFreeBit), stamp).free_pressed);
  }
  EXPECT_FALSE(logic.update(0, 1.4).free_short_pulse);
  EXPECT_TRUE(logic.update(0, 1.6).free_short_pulse);
  for (double stamp : {1.7, 1.8, 1.9, 2.0}) {
    EXPECT_FALSE(
        logic.update(bit(ToolButtonLogic::kFreeBit), stamp).free_pressed);
  }
}

TEST(ToolButtonLogicTest, ConfirmedFreePressAndReleaseRequestOneTransitionEach) {
  ToolButtonLogic logic;
  logic.update(0, 0.0);
  for (unsigned int sample = 0;
       sample + 1 < ToolButtonLogic::kDefaultFreePressSamples; ++sample) {
    EXPECT_FALSE(logic.update(bit(ToolButtonLogic::kFreeBit),
                              1.0 + 0.1 * sample).free_pressed);
  }
  const ToolButtonLogic::Events confirmed =
      logic.update(bit(ToolButtonLogic::kFreeBit), 1.7);
  EXPECT_TRUE(confirmed.free_pressed);
  EXPECT_TRUE(confirmed.free_confirmed_held);
  const ToolButtonLogic::Events held =
      logic.update(bit(ToolButtonLogic::kFreeBit), 1.8);
  EXPECT_FALSE(held.free_pressed);
  EXPECT_TRUE(held.free_confirmed_held);
  const ToolButtonLogic::Events released = logic.update(0, 1.9);
  EXPECT_FALSE(released.free_released);
  EXPECT_TRUE(released.free_confirmed_held);
  const ToolButtonLogic::Events confirmed_release = logic.update(0, 2.0);
  EXPECT_TRUE(confirmed_release.free_released);
  EXPECT_FALSE(confirmed_release.free_confirmed_held);
  EXPECT_FALSE(logic.update(0, 2.1).free_released);
}

TEST(ToolButtonLogicTest, InputLossExitsConfirmedFreeAndRequiresLowRearm) {
  ToolButtonLogic logic;
  logic.update(0, 0.0);
  for (unsigned int sample = 0;
       sample + 1 < ToolButtonLogic::kDefaultFreePressSamples; ++sample) {
    EXPECT_FALSE(logic.update(bit(ToolButtonLogic::kFreeBit),
                              1.0 + 0.1 * sample).free_pressed);
  }
  EXPECT_TRUE(logic.update(bit(ToolButtonLogic::kFreeBit), 1.7).free_pressed);
  const ToolButtonLogic::Events lost = logic.inputUnavailable();
  EXPECT_TRUE(lost.free_released);
  EXPECT_FALSE(lost.free_confirmed_held);

  EXPECT_FALSE(
      logic.update(bit(ToolButtonLogic::kFreeBit), 2.0).free_pressed);
  EXPECT_FALSE(logic.update(0, 2.1).free_released);
  for (unsigned int sample = 0;
       sample + 1 < ToolButtonLogic::kDefaultFreePressSamples; ++sample) {
    EXPECT_FALSE(logic.update(bit(ToolButtonLogic::kFreeBit),
                              3.0 + 0.1 * sample).free_pressed);
  }
  EXPECT_TRUE(logic.update(bit(ToolButtonLogic::kFreeBit), 3.7).free_pressed);
}

TEST(ToolButtonLogicTest, InputLossClearsUnconfirmedFreeCandidate) {
  ToolButtonLogic logic;
  logic.update(0, 0.0);
  EXPECT_FALSE(
      logic.update(bit(ToolButtonLogic::kFreeBit), 1.0).free_pressed);

  const ToolButtonLogic::Events lost = logic.inputUnavailable();
  EXPECT_FALSE(lost.free_pressed);
  EXPECT_FALSE(lost.free_released);
  EXPECT_FALSE(lost.free_short_pulse);

  EXPECT_FALSE(
      logic.update(bit(ToolButtonLogic::kFreeBit), 1.1).free_pressed);
  EXPECT_FALSE(logic.update(0, 1.2).free_released);
  for (unsigned int sample = 0;
       sample + 1 < ToolButtonLogic::kDefaultFreePressSamples; ++sample) {
    EXPECT_FALSE(logic.update(bit(ToolButtonLogic::kFreeBit),
                              2.0 + 0.1 * sample).free_pressed);
  }
  EXPECT_TRUE(logic.update(bit(ToolButtonLogic::kFreeBit), 2.7).free_pressed);
}

TEST(ToolButtonLogicTest, PointUsesRisingEdgeOnly) {
  ToolButtonLogic logic;
  logic.update(0, 0.0);
  EXPECT_TRUE(logic.update(bit(ToolButtonLogic::kPointBit), 0.1).point_pressed);
  EXPECT_FALSE(
      logic.update(bit(ToolButtonLogic::kPointBit), 0.2).point_pressed);
  logic.update(0, 0.3);
  EXPECT_TRUE(logic.update(bit(ToolButtonLogic::kPointBit), 0.4).point_pressed);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
