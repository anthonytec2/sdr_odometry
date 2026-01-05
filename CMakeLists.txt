cmake_minimum_required(VERSION 3.8)
project(sdr_odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find Packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# FIX: Find librtlsdr manually (more robust than PkgConfig)
# This looks directly for the header and library file
find_path(RTLSDR_INCLUDE_DIR NAMES rtl-sdr.h)
find_library(RTLSDR_LIBRARY NAMES rtlsdr)

if(NOT RTLSDR_INCLUDE_DIR OR NOT RTLSDR_LIBRARY)
  message(FATAL_ERROR "Could not find librtlsdr! Please install it: sudo apt install librtlsdr-dev")
endif()

# 2. Include directories
include_directories(include)

# 3. Generate Custom Messages
# This creates a library target named "sdr_odometry" (same as project name)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SdrPacket.msg"
  DEPENDENCIES std_msgs
)

# 4. Component Library
add_library(sdr_node_component SHARED
  src/sdr_node.cpp
)

ament_target_dependencies(sdr_node_component
  rclcpp
  rclcpp_components
  pluginlib
  std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(sdr_node_component "${cpp_typesupport_target}")

target_include_directories(sdr_node_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${RTLSDR_INCLUDE_DIR}
)

target_link_libraries(sdr_node_component ${RTLSDR_LIBRARY})

rclcpp_components_register_nodes(sdr_node_component "SDRNode")
ament_export_targets(export_sdr_node_component HAS_LIBRARY_TARGET)

# 5. Standalone executable (optional)
add_executable(sdr_node src/sdr_node_main.cpp)
target_link_libraries(sdr_node sdr_node_component)

# 6. Install Rules
install(TARGETS
  sdr_node_component
  EXPORT export_sdr_node_component
  DESTINATION lib
)

install(TARGETS sdr_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(sdr_node_component)
ament_export_dependencies(rclcpp rclcpp_components pluginlib std_msgs)

ament_package()