cmake_minimum_required(VERSION 3.8)
project(nav_decision_maker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)

# Add service definition
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetTarget.srv"
  DEPENDENCIES
  geometry_msgs
)

# Add executable
add_executable(nav_decision_maker_node src/nav_decision_maker.cpp)  # 修改目标名称
set_target_properties(nav_decision_maker_node PROPERTIES OUTPUT_NAME nav_decision_maker)
ament_target_dependencies(nav_decision_maker_node  # 修改目标名称
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  tf2_geometry_msgs
  tf2_ros
  rosidl_default_runtime
  rosidl_typesupport_cpp
)

# Make sure the service interface is generated before building the executable
add_dependencies(nav_decision_maker_node ${PROJECT_NAME}__rosidl_generator_c ${PROJECT_NAME}__rosidl_typesupport_cpp)

# Add include directories
target_include_directories(nav_decision_maker_node  # 修改目标名称
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_typesupport_cpp
)

# Link against the generated service typesupport
target_link_libraries(nav_decision_maker_node  # 修改目标名称
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# Install
install(TARGETS
  nav_decision_maker_node  # 修改目标名称
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

# Correct the copy command for the executable
install(CODE "
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    \$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/nav_decision_maker_node
    \$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/nav_decision_maker
  )
")

install(DIRECTORY
  srv
  DESTINATION share/${PROJECT_NAME}
)

include_directories(include)

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
