cmake_minimum_required(VERSION 3.8)
project(ur20_display)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(ur_description REQUIRED)

# Secondary dependencies
find_package(Eigen3 REQUIRED)


# Register the LinearAlgebraServer node
add_executable(ur20_joint_configuration_publisher src/joint_configuration_publisher.cpp)
ament_target_dependencies(ur20_joint_configuration_publisher rclcpp sensor_msgs rcl_interfaces tf2 tf2_ros tf2_eigen rviz_visual_tools ur_description)
# Eigen is not an ament package and should be linked separately
# docs: https://eigen.tuxfamily.org/dox/TopicCMakeGuide.html
target_link_libraries(ur20_joint_configuration_publisher Eigen3::Eigen)

install(TARGETS
  ur20_joint_configuration_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}/
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
