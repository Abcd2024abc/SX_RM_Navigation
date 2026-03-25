cmake_minimum_required(VERSION 3.10)
project(decision_tree)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# 添加库
ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN decision_tree::DecisionTree
  EXECUTABLE decision_tree_node
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  params
)