cmake_minimum_required(VERSION 3.20)
project(rise_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rise_motion_messages REQUIRED)
find_package(config QUIET)

include_directories(include)

add_subdirectory(SOEM)

add_executable(
	rise_motion_main
	src/main.cpp
	src/ec_manager.cpp
	src/ethercat_node.cpp
  src/cia402.cpp
)
ament_target_dependencies(
	rise_motion_main
	rclcpp
	rise_motion_messages
)
target_link_libraries(
	rise_motion_main
	soem
)

if(config_FOUND)
  message(NOTE "external config package found: using values from config package")

  target_link_libraries(
    rise_motion_main
    config::config_settings
  )

  target_compile_definitions(
    rise_motion_main
    PRIVATE EXTERNAL_CONFIG_AVAILABLE=1
  )
else()
  message(WARNING "no external config package found: using values from local rise_motion/config.hpp")

  target_compile_definitions(
    rise_motion_main
    PRIVATE EXTERNAL_CONFIG_AVAILABLE=0
  )
endif()

#target_link_options(rise_motion_main PRIVATE -Wl,-rpath,/hello/home/rise/Projects/rise-motion/rise_motion_dev_ws/install/rise_motion_messages/lib)
install(TARGETS
  rise_motion_main
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(testing_node
  src/testing_node.cpp
)

target_link_libraries(testing_node PUBLIC
  ${rise_motion_messages_TARGETS}
  rclcpp::rclcpp
)

install(TARGETS
  testing_node
  DESTINATION lib/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
