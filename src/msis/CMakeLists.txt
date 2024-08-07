cmake_minimum_required(VERSION 3.8)
project(msis)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/msis
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

add_executable(test_node src/test_node.cpp)
ament_target_dependencies(test_node rclcpp sensor_msgs geometry_msgs)

add_executable(sts1000_dc src/sts1000_dc.cpp src/define_sts1000.cpp)
ament_target_dependencies(sts1000_dc rclcpp sensor_msgs geometry_msgs)

add_executable(sts1000_process src/sts1000_process.cpp src/user_math_lib.cpp src/define_sts1000.cpp)
ament_target_dependencies(sts1000_process rclcpp sensor_msgs geometry_msgs cv_bridge)
target_link_libraries(sts1000_process ${OpenCV_LIBS} )

add_executable(sts1000_ping_to_pointcloud src/sts1000_ping_to_pointcloud.cpp )
ament_target_dependencies(sts1000_ping_to_pointcloud rclcpp sensor_msgs geometry_msgs)

add_executable(sts1000_ping_to_pointcloud_cirs src/sts1000_ping_to_pointcloud_cirs.cpp )
ament_target_dependencies(sts1000_ping_to_pointcloud_cirs rclcpp sensor_msgs geometry_msgs)

install(TARGETS
  test_node
  sts1000_dc
  sts1000_process
  sts1000_ping_to_pointcloud
  sts1000_ping_to_pointcloud_cirs
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})



ament_package()
