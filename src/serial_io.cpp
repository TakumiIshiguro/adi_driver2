#include "adi_driver2/serial_io.hpp"

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <poll.h>
#include <unistd.h>

namespace adi_driver2
{

ssize_t read_exact_with_timeout(
  int fd, void * buffer, size_t size, std::chrono::milliseconds timeout)
{
  auto * output = static_cast<uint8_t *>(buffer);
  size_t received = 0;
  const auto deadline = std::chrono::steady_clock::now() + timeout;

  while (received < size) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      errno = ETIMEDOUT;
      return -1;
    }

    const auto remaining_us =
      std::chrono::duration_cast<std::chrono::microseconds>(deadline - now).count();
    const int remaining_ms = static_cast<int>((remaining_us + 999) / 1000);
    pollfd descriptor{fd, POLLIN, 0};
    const int poll_result = poll(&descriptor, 1, remaining_ms);
    if (poll_result == 0) {
      errno = ETIMEDOUT;
      return -1;
    }
    if (poll_result < 0) {
      if (errno == EINTR) {
        continue;
      }
      return -1;
    }
    if ((descriptor.revents & (POLLERR | POLLNVAL)) != 0) {
      errno = EIO;
      return -1;
    }

    const ssize_t count = read(fd, output + received, size - received);
    if (count > 0) {
      received += static_cast<size_t>(count);
      continue;
    }
    if (count < 0 && errno == EINTR) {
      continue;
    }
    if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      continue;
    }

    errno = EIO;
    return -1;
  }

  return static_cast<ssize_t>(received);
}

}  // namespace adi_driver2
