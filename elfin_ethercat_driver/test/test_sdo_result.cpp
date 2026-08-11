#include <gtest/gtest.h>

#include <cstdint>

#include <elfin_ethercat_driver/sdo_result.h>
#include <elfin_ethercat_driver/pdo_mapping.h>


namespace {

using elfin_ethercat_driver::detail::acceptSdoReadResult;
using elfin_ethercat_driver::detail::findPdoEntryByteOffset;


TEST(SdoReadResult, RejectsMissingSlaveResponseAndClearsOutput)
{
  int16_t output = 1234;
  EXPECT_FALSE(acceptSdoReadResult<int16_t>(0, sizeof(int16_t), 77, output));
  EXPECT_EQ(0, output);
}

TEST(SdoReadResult, RejectsPartialObjectAndClearsOutput)
{
  int32_t output = 1234;
  EXPECT_FALSE(acceptSdoReadResult<int32_t>(1, sizeof(int16_t), 77, output));
  EXPECT_EQ(0, output);
}

TEST(SdoReadResult, AcceptsCompleteObject)
{
  int32_t output = 0;
  EXPECT_TRUE(acceptSdoReadResult<int32_t>(1, sizeof(int32_t), -42, output));
  EXPECT_EQ(-42, output);
}

TEST(PdoMapping, ResolvesDigitalInputAfterEarlierMappedObjects)
{
  const std::vector<uint32_t> mappings = {
      0x60100120U,
      0x60010110U,
  };
  std::size_t offset = 0;
  EXPECT_TRUE(findPdoEntryByteOffset(mappings, 0, 0x6001, 0x01, 16, offset));
  EXPECT_EQ(4U, offset);
}

TEST(PdoMapping, RejectsAbsentUnalignedOrWrongWidthDigitalInput)
{
  std::size_t offset = 99;
  EXPECT_FALSE(findPdoEntryByteOffset(
      std::vector<uint32_t>(1, 0x60100120U),
      0, 0x6001, 0x01, 16, offset));
  EXPECT_FALSE(findPdoEntryByteOffset(
      std::vector<uint32_t>{0x60100101U, 0x60010110U},
      0, 0x6001, 0x01, 16, offset));
  EXPECT_FALSE(findPdoEntryByteOffset(
      std::vector<uint32_t>(1, 0x60010108U),
      0, 0x6001, 0x01, 16, offset));
  EXPECT_FALSE(findPdoEntryByteOffset(
      std::vector<uint32_t>(1, 0x60010110U),
      1, 0x6001, 0x01, 16, offset));

  EXPECT_TRUE(findPdoEntryByteOffset(
      std::vector<uint32_t>{0x00000004U, 0x60010110U},
      4, 0x6001, 0x01, 16, offset));
  EXPECT_EQ(1U, offset);
}

}  // namespace

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
