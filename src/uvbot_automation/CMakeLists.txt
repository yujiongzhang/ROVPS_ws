cmake_minimum_required(VERSION 3.8)
project(uvbot_automation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(demo src/demo.cpp)
ament_target_dependencies(demo rclcpp tf2_ros tf2_geometry_msgs geometry_msgs)

add_executable(carto_odometer src/carto_odometer.cpp)
ament_target_dependencies(carto_odometer rclcpp tf2_ros tf2_geometry_msgs geometry_msgs nav_msgs)

add_executable(carto_odometer2 src/carto_odometer2.cpp)
ament_target_dependencies(carto_odometer2 rclcpp tf2_ros tf2_geometry_msgs geometry_msgs nav_msgs)

add_executable(carto_odometer_speed src/carto_odometer_speed.cpp)
ament_target_dependencies(carto_odometer_speed rclcpp tf2_ros tf2_geometry_msgs geometry_msgs nav_msgs)

add_executable(cmdTransmit src/cmdTransmit.cpp)
ament_target_dependencies(cmdTransmit rclcpp tf2_ros tf2_geometry_msgs geometry_msgs nav_msgs)

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

install(TARGETS
  demo
  carto_odometer
  carto_odometer2
  carto_odometer_speed
  cmdTransmit
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
