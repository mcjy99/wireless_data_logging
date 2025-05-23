cmake_minimum_required(VERSION 3.22.1)
project(uav_lora)

## Generate symbols for IDE indexer
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#find_package(ament_cmake_auto REQUIRED)
#ament_auto_find_build_dependencies()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(uav_comms_msgs REQUIRED)

include_directories(
  include
)

add_executable(uav_lora_node
  src/uav_lora_node.cpp 
)

# Dependencies
ament_target_dependencies(uav_lora_node
  rclcpp
  std_msgs
  uav_comms_msgs
)

# Install
install(TARGETS
  uav_lora_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and other files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()