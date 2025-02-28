cmake_minimum_required(VERSION 3.5)
project(bot_sdk_can)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(sensor_msgs REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/can_common.cpp
  src/can_id.cpp
  src/can_receiver.cpp
  src/can_sender.cpp
  src/can_cmd.cpp)

ament_auto_add_library(can_receiver_node SHARED
  src/can_receiver_node.cpp
)

target_link_libraries(can_receiver_node
  ${PROJECT_NAME}
)

rclcpp_components_register_node(can_receiver_node
  PLUGIN "bot::can::CanReceiverNode"
  EXECUTABLE can_receiver_node_exe
)

ament_auto_add_library(can_sender_node SHARED
  src/can_sender_node.cpp
)

target_link_libraries(can_sender_node
  ${PROJECT_NAME}
)

rclcpp_components_register_node(can_sender_node
  PLUGIN "bot::can::CanSenderNode"
  EXECUTABLE can_sender_node_exe
)

ament_auto_package(INSTALL_TO_SHARE
  launch
)
