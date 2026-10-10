cmake_minimum_required(VERSION 3.8)
project(hexapod_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED) 
find_package(pluginlib REQUIRED)

find_library(PIPCA9685_LIB PiPCA9685 REQUIRED)
find_library(I2C_LIB i2c REQUIRED)

add_library(hexapod_hardware SHARED
  src/hexapod_system.cpp
  src/pca9685_concrete.cpp
  src/mock_concrete.cpp
)

target_include_directories(hexapod_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(hexapod_hardware
  rclcpp
  sensor_msgs
  std_msgs
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

target_link_libraries(hexapod_hardware ${PIPCA9685_LIB} ${I2C_LIB})

pluginlib_export_plugin_description_file(hardware_interface hexapod_hardware.xml)

add_executable(servo_driver_node
  src/servo_driver_node.cpp
)

target_include_directories(servo_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(servo_driver_node PUBLIC cxx_std_17)

ament_target_dependencies(servo_driver_node
  rclcpp
  sensor_msgs
  std_msgs
)

target_link_libraries(servo_driver_node hexapod_hardware ${I2C_LIB})

install(TARGETS hexapod_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS servo_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES hexapod_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(hexapod_hardware ${I2C_LIB})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
