cmake_minimum_required(VERSION 3.20)
project(rise_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorCommandArray.msg"
  "msg/MotorFeedbackArray.msg"
  DEPENDENCIES std_msgs
)

# Include directories
include_directories(
  include
)

set(SOEM_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/SOEM")


add_subdirectory(${SOEM_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR}/SOEM)

include_directories(${SOEM_INCLUDE_DIRS})



add_executable(ethercat_initializer
  src/ethercat_initializer.cpp
)

ament_target_dependencies(ethercat_initializer
  rclcpp
  std_msgs
)

# Link to generated messages
rosidl_target_interfaces(ethercat_initializer
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

target_link_libraries(ethercat_initializer soem pthread)


# Install targets
install(TARGETS
  ethercat_initializer
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(rclcpp rclcpp_components std_msgs)

ament_package()
