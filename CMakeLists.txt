cmake_minimum_required(VERSION 3.5)
project(ros2_odrive_can)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Add msgs/srvs
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/OdriveStatus.msg"
  "srv/EncoderEstimates.srv"
)

include_directories(
  include
)
add_library(${PROJECT_NAME}_interface
  src/socketcan_interface.cpp
)


add_executable(odrive_can src/odrive_can.cpp)
ament_target_dependencies(odrive_can rclcpp std_msgs)
target_include_directories(odrive_can PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(odrive_can
  ${PROJECT_NAME}_interface
)

install(TARGETS odrive_can
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Allows the use of msgs/srvs defined in this package
rosidl_target_interfaces(odrive_can
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package() 
