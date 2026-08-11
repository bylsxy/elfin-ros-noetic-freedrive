#ifndef ELFIN_ETHERCAT_DRIVER_PDO_MAPPING_H
#define ELFIN_ETHERCAT_DRIVER_PDO_MAPPING_H

#include <cstddef>
#include <cstdint>
#include <vector>

namespace elfin_ethercat_driver {
namespace detail {
inline bool findPdoEntryByteOffset(const std::vector<uint32_t>& mappings,
                                   uint8_t input_start_bit,
                                   uint16_t target_index,
                                   uint8_t target_subindex,
                                   uint8_t target_bit_length,
                                   std::size_t& byte_offset)
{
  byte_offset = 0;
  std::size_t bit_offset = input_start_bit;
  for (std::vector<uint32_t>::const_iterator it = mappings.begin();
       it != mappings.end(); ++it)
  {
    const uint32_t mapping = *it;
    const uint16_t index = static_cast<uint16_t>(mapping >> 16);
    const uint8_t subindex = static_cast<uint8_t>((mapping >> 8) & 0xffU);
    const uint8_t bit_length = static_cast<uint8_t>(mapping & 0xffU);
    if (index == target_index && subindex == target_subindex)
    {
      if (bit_length != target_bit_length || bit_offset % 8U != 0U)
      {
        return false;
      }
      byte_offset = bit_offset / 8U;
      return true;
    }
    bit_offset += bit_length;
  }
  return false;
}

}  // namespace detail
}  // namespace elfin_ethercat_driver

#endif
