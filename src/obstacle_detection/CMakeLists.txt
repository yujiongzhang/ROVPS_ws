cmake_minimum_required(VERSION 3.8)
project(obstacle_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(OpenCV REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/image_fun
  ${PROJECT_SOURCE_DIR}/include/obstacle_detection

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

add_executable(grid_demo src/grid_demo.cpp)
ament_target_dependencies(grid_demo rclcpp nav_msgs)

add_executable(msis_gripmap src/msis_gripmap.cpp)
ament_target_dependencies(msis_gripmap rclcpp nav_msgs OpenCV)

install(TARGETS
  grid_demo
  msis_gripmap
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
