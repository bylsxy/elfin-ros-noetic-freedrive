#ifndef ELFIN_ETHERCAT_DRIVER_SDO_RESULT_H
#define ELFIN_ETHERCAT_DRIVER_SDO_RESULT_H

#include <cstddef>

namespace elfin_ethercat_driver {
namespace detail {

template <typename T>
bool acceptSdoReadResult(int work_counter, int received_size,
                         const T& received_value, T& value)
{
  value = T();
  if (work_counter <= 0 || received_size != static_cast<int>(sizeof(T)))
  {
    return false;
  }
  value = received_value;
  return true;
}

}  // namespace detail
}  // namespace elfin_ethercat_driver

#endif
