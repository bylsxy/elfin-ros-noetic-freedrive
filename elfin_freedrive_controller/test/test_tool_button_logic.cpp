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
  EXPECT_FALSE(logic.update(bit(ToolButtonLogic::kFreeBit)).free_pressed);
  EXPECT_FALSE(logic.update(bit(ToolButtonLogic::kFreeBit)).free_pressed);
  EXPECT_FALSE(logic.update(0).free_released);
  EXPECT_TRUE(logic.update(bit(ToolButtonLogic::kFreeBit)).free_pressed);
}

TEST(ToolButtonLogicTest, FreePressIsImmediateAndReleaseRequestsExit) {
  ToolButtonLogic logic;
  logic.update(0);
  EXPECT_TRUE(logic.update(bit(ToolButtonLogic::kFreeBit)).free_pressed);
  EXPECT_FALSE(logic.update(bit(ToolButtonLogic::kFreeBit)).free_pressed);
  EXPECT_TRUE(logic.update(0).free_released);
}

TEST(ToolButtonLogicTest, PointUsesRisingEdgeOnly) {
  ToolButtonLogic logic;
  logic.update(0);
  EXPECT_TRUE(logic.update(bit(ToolButtonLogic::kPointBit)).point_pressed);
  EXPECT_FALSE(logic.update(bit(ToolButtonLogic::kPointBit)).point_pressed);
  logic.update(0);
  EXPECT_TRUE(logic.update(bit(ToolButtonLogic::kPointBit)).point_pressed);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
