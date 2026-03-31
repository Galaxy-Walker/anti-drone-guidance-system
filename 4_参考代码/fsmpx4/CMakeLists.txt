cmake_minimum_required(VERSION 3.8)
project(fsmpx4)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FSMDebug.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

add_library(fsmpx4_input STATIC
  src/input.cpp
)

target_include_directories(fsmpx4_input
  PUBLIC
    include
    ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(fsmpx4_input
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  px4_msgs
  tf2_msgs
  Eigen3
)

add_executable(fsmpx4_input_debug_node
  test/test_input.cpp
)

target_link_libraries(fsmpx4_input_debug_node
  fsmpx4_input
)

ament_target_dependencies(fsmpx4_input_debug_node
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  px4_msgs
  tf2_msgs
  Eigen3
)

# Minimal position->attitude controller
add_library(fsmpx4_control STATIC
  src/control.cpp
)

target_include_directories(fsmpx4_control
  PUBLIC
    include
    ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(fsmpx4_control
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  px4_msgs
  tf2_msgs
  Eigen3
)

# Parameter loader
add_library(fsmpx4_param_loader STATIC
  src/param_loader.cpp
)

target_include_directories(fsmpx4_param_loader
  PUBLIC
    include
    ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(fsmpx4_param_loader
  rclcpp
  Eigen3
)

# FSM core library（封装有限状态机逻辑）
add_library(fsmpx4_fsm STATIC
  src/fsmpx4.cpp
)

target_include_directories(fsmpx4_fsm
  PUBLIC
    include
    ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(fsmpx4_fsm
  fsmpx4_input
)

ament_target_dependencies(fsmpx4_fsm
  rclcpp
  px4_msgs
  tf2_msgs
  Eigen3
)

# FSM main node
add_executable(fsm_node
  src/fsm_node.cpp
)

target_link_libraries(fsm_node
  fsmpx4_fsm
  fsmpx4_param_loader
  fsmpx4_control
)

ament_target_dependencies(fsm_node
  rclcpp
)

add_executable(point_control_node
  test/point_control_node.cpp
)

target_link_libraries(point_control_node
  fsmpx4_input
  fsmpx4_control
  fsmpx4_param_loader
)

ament_target_dependencies(point_control_node
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  px4_msgs
  tf2_msgs
  Eigen3
)

# Minimal SE3 position->attitude controller library

install(TARGETS
  fsmpx4_input
  fsmpx4_input_debug_node
  fsmpx4_control
  fsmpx4_param_loader
  fsmpx4_fsm
  fsm_node
  point_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(fsmpx4_fsm "${cpp_typesupport_target}")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  px4_msgs
  tf2_msgs
  Eigen3
  rosidl_default_runtime
)

ament_package()
