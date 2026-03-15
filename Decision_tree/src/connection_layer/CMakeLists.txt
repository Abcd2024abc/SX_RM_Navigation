cmake_minimum_required(VERSION 3.10)
project(connection_layer)

# 设置C++标准
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 强制代码规范（GND AND Clang）
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# 输出编译命令数据库
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## 自动查找依赖
find_package(ament_cmake_auto REQUIRED)
find_package(serial_driver REQUIRED)
ament_auto_find_build_dependencies()

## 生成自定义消息接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GameStatus.msg"
  "msg/RobotStatus.msg"
  DEPENDENCIES std_msgs
)

## 构建库
set(LIB_NAME ${PROJECT_NAME}_lib)
ament_auto_add_library(${LIB_NAME} SHARED
  DIRECTORY src
)

# 将自定义消息接口关联到库
rosidl_get_typesupport_target(
TYPESUPPORT_TARGET ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
target_link_libraries(${LIB_NAME} ${TYPESUPPORT_TARGET})

# 注册节点
rclcpp_components_register_node(${LIB_NAME}
  PLUGIN "connection_layer::ConnectionLayerNode"
  EXECUTABLE connection_layer_node
)

## 安装
ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  config
  launch
)
