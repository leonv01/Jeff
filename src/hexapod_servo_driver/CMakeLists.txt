cmake_minimum_required(VERSION 3.8)
project(hexapod_servo_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hexapod_hardware REQUIRED)

find_library(I2C_LIB i2c REQUIRED)

add_executable(servo_driver_node
  src/servo_driver_node.cpp
)

target_include_directories(servo_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(servo_driver_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(servo_driver_node
  rclcpp
  sensor_msgs
  std_msgs
  hexapod_hardware
)

target_link_libraries(servo_driver_node ${I2C_LIB})

install(TARGETS servo_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
