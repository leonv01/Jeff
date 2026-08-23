cmake_minimum_required(VERSION 3.8)
project(hexapod_gait)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(hexapod_gait 
  src/hexapod_gait.cpp 
  src/hexapod_ik.cpp
  src/trajectory_generator.cpp
  src/gait_strategy.cpp
  src/gait_factory.cpp
  src/riple_gait_concrete.cpp
  src/wave_gait_concrete.cpp
  src/tripod_gait_concrete.cpp
  )
target_include_directories(hexapod_gait PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(hexapod_gait PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_link_libraries(hexapod_gait Eigen3::Eigen)

ament_target_dependencies(hexapod_gait
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
)

install(TARGETS hexapod_gait
  DESTINATION lib/${PROJECT_NAME})

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
