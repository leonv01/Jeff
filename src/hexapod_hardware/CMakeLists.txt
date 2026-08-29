cmake_minimum_required(VERSION 3.8)
project(hexapod_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED) 
find_package(pluginlib REQUIRED)

find_library(PIPCA9685_LIB PiPCA9685 REQUIRED)

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
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

target_link_libraries(hexapod_hardware ${PIPCA9685_LIB})

pluginlib_export_plugin_description_file(hardware_interface hexapod_hardware.xml)

install(TARGETS hexapod_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
