cmake_minimum_required(VERSION 3.8)
project(TF_transform)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
endif()

add_executable(TF_transform src/TF_transform.cpp)
ament_target_dependencies(TF_transform rclcpp tf2 tf2_ros geometry_msgs tf2_geometry_msgs)

install(TARGETS
TF_transform
DESTINATION lib/${PROJECT_NAME})

ament_package()
