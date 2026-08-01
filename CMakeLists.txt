cmake_minimum_required(VERSION 3.8)
project(adi_driver2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

include_directories(include)

ament_auto_add_executable(adis16465
  src/adis16465.cpp
  src/adis16465_node.cpp
  src/serial_io.cpp
)

target_link_libraries(adis16465)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_serial_io test/test_serial_io.cpp src/serial_io.cpp)
  target_include_directories(test_serial_io PRIVATE include)
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
  urdf
)
