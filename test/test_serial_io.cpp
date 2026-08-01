#include <gtest/gtest.h>

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <thread>
#include <unistd.h>

#include "adi_driver2/serial_io.hpp"

using namespace std::chrono_literals;

TEST(SerialIo, ReadsAResponseDeliveredInMultipleChunks)
{
  int descriptors[2];
  ASSERT_EQ(pipe(descriptors), 0);

  std::thread writer([write_fd = descriptors[1]]() {
      const uint8_t first[] = {1, 2};
      const uint8_t second[] = {3, 4, 5};
      EXPECT_EQ(write(write_fd, first, sizeof(first)), static_cast<ssize_t>(sizeof(first)));
      std::this_thread::sleep_for(1ms);
      EXPECT_EQ(write(write_fd, second, sizeof(second)), static_cast<ssize_t>(sizeof(second)));
    });

  uint8_t output[5] = {};
  EXPECT_EQ(
    adi_driver2::read_exact_with_timeout(descriptors[0], output, sizeof(output), 20ms),
    static_cast<ssize_t>(sizeof(output)));
  EXPECT_EQ(output[0], 1);
  EXPECT_EQ(output[4], 5);

  writer.join();
  close(descriptors[0]);
  close(descriptors[1]);
}

TEST(SerialIo, ReturnsTimeoutInsteadOfBlocking)
{
  int descriptors[2];
  ASSERT_EQ(pipe(descriptors), 0);

  uint8_t output = 0;
  errno = 0;
  EXPECT_EQ(adi_driver2::read_exact_with_timeout(descriptors[0], &output, 1, 2ms), -1);
  EXPECT_EQ(errno, ETIMEDOUT);

  close(descriptors[0]);
  close(descriptors[1]);
}
