cmake_minimum_required(VERSION 3.8)
project(robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

# Build executable
add_executable(tracking_controller src/tracking_controller.cpp)
target_include_directories(tracking_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  tracking_controller
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2
)

# Install
install(TARGETS tracking_controller
  DESTINATION lib/${PROJECT_NAME})

add_executable(obstacle_avoider src/obstacle_avoider.cpp)
target_include_directories(obstacle_avoider PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  obstacle_avoider
  rclcpp
  geometry_msgs
  sensor_msgs
)

install(TARGETS obstacle_avoider
  DESTINATION lib/${PROJECT_NAME})

add_executable(person_mover src/person_mover.cpp)
target_include_directories(person_mover PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  person_mover
  rclcpp
  geometry_msgs
)

install(TARGETS person_mover
  DESTINATION lib/${PROJECT_NAME})

add_executable(ball_randomizer src/ball_randomizer.cpp)
target_include_directories(ball_randomizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  ball_randomizer
  rclcpp
  geometry_msgs
)

install(TARGETS ball_randomizer
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
