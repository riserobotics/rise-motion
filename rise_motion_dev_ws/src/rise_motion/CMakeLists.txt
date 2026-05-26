cmake_minimum_required(VERSION 3.20)
project(rise_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rise_motion_messages REQUIRED)
find_package(ament_cmake_python REQUIRED)

include_directories(include)

add_subdirectory(SOEM)

# --- sdo_serializer library ---
add_library(sdo_serializer_lib INTERFACE)
target_include_directories(sdo_serializer_lib INTERFACE include)

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

ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # --- sdo_serializer test ---
  ament_add_gtest(test_sdo_serializer
    test/test_sdo_serializer.cpp
  )
  ament_add_pytest_test(test_sdo_serializer_python 
    test/test_sdo_serializer.py
  )
  if(TARGET test_sdo_serializer)
    target_link_libraries(test_sdo_serializer sdo_serializer_lib)
  endif()

endif()

ament_package()
