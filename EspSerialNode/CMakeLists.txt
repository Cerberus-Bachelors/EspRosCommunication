cmake_minimum_required(VERSION 3.8)
project(espserialnode)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(cerberus_msgs REQUIRED)

# Include headers (optional, if needed)
include_directories(include)

# Add the executable
add_executable(sensorHandler
  src/sensorHandler.cpp
)

# Link libraries to the executable
ament_target_dependencies(sensorHandler
  rclcpp
  std_msgs
  serial
  cerberus_msgs
)

# Install the executable
install(TARGETS sensorHandler
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
