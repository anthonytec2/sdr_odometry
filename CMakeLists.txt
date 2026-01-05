cmake_minimum_required(VERSION 3.8)
project(sdr_odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find Packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# FIX: Find librtlsdr manually (more robust than PkgConfig)
# This looks directly for the header and library file
find_path(RTLSDR_INCLUDE_DIR NAMES rtl-sdr.h)
find_library(RTLSDR_LIBRARY NAMES rtlsdr)

if(NOT RTLSDR_INCLUDE_DIR OR NOT RTLSDR_LIBRARY)
  message(FATAL_ERROR "Could not find librtlsdr! Please install it: sudo apt install librtlsdr-dev")
endif()

# 2. Generate Custom Messages
# This creates a library target named "sdr_odometry" (same as project name)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SdrPacket.msg"
  DEPENDENCIES std_msgs
)

# 3. Build the Node
add_executable(sdr_node src/sdr_node.cpp)

# 4. Link Dependencies
ament_target_dependencies(sdr_node 
  rclcpp 
  std_msgs 
)

# 5. Link the Custom Message (Updated for newer ROS 2 versions)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(sdr_node "${cpp_typesupport_target}")

# 6. Link System Libraries (RTL-SDR)
target_include_directories(sdr_node PUBLIC ${RTLSDR_INCLUDE_DIR})
target_link_libraries(sdr_node ${RTLSDR_LIBRARY})

# 7. Install Rules
install(TARGETS sdr_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()