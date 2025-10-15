cmake_minimum_required(VERSION 3.8)
project(firefighter_flipper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(firefighter_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# Robotics 라이브러리
add_library(robotics_lib
  src/Robotics.cpp
  src/FirefighterControl.cpp
)
ament_target_dependencies(robotics_lib
  Eigen3
)
target_include_directories(robotics_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 플리퍼 자세 제어 노드
add_executable(flipper_attitude_control src/flipper_attitude_control.cpp)
ament_target_dependencies(flipper_attitude_control
  rclcpp
  sensor_msgs
  geometry_msgs
  firefighter_interfaces
)
target_link_libraries(flipper_attitude_control robotics_lib)

# Robotics 테스트 노드
add_executable(robotics_test src/robotics_test.cpp)
ament_target_dependencies(robotics_test
  rclcpp
  sensor_msgs
)
target_link_libraries(robotics_test robotics_lib)

# Flipper positions republisher node
add_executable(flipper_positions_node src/flipper_positions_node.cpp)
ament_target_dependencies(flipper_positions_node
  rclcpp
  sensor_msgs
  firefighter_interfaces
)

# 설치
install(TARGETS
  robotics_lib
  flipper_attitude_control
  robotics_test
  flipper_positions_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
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
