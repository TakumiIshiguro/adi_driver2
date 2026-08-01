#ifndef ADI_DRIVER2_SERIAL_IO_HPP_
#define ADI_DRIVER2_SERIAL_IO_HPP_

#include <chrono>
#include <cstddef>
#include <sys/types.h>

namespace adi_driver2
{

ssize_t read_exact_with_timeout(
  int fd, void * buffer, size_t size, std::chrono::milliseconds timeout);

}  // namespace adi_driver2

#endif  // ADI_DRIVER2_SERIAL_IO_HPP_
