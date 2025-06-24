cmake_minimum_required(VERSION 3.8)
project(linear_algebra_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(linear_algebra_resources REQUIRED)

# Secondary dependencies
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# Register the LinearAlgebraServer node
add_executable(service src/linear_algebra_server.cpp)
ament_target_dependencies(service rclcpp std_msgs geometry_msgs linear_algebra_resources)
# Eigen is not an ament package and should be linked separately
# docs: https://eigen.tuxfamily.org/dox/TopicCMakeGuide.html
target_link_libraries(service Eigen3::Eigen)

# Register the LinearAlgebraServer node
add_executable(client src/linear_algebra_client.cpp)
ament_target_dependencies(client rclcpp std_msgs geometry_msgs linear_algebra_resources)
target_link_libraries(client Eigen3::Eigen)
target_link_libraries(client ${YAML_CPP_LIBRARIES})

install(TARGETS
  service
  client
  DESTINATION lib/${PROJECT_NAME}
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
