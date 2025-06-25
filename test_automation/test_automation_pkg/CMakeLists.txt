cmake_minimum_required(VERSION 3.8)
project(test_automation_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Add the custom cmake modules
set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake-modules")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

# find dependencies
find_package(ZMQ)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ros2_bt_utils REQUIRED)

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


# ZMQ Optional
if( ZMQ_FOUND )
    message(STATUS "ZeroMQ found.")
    add_definitions( -DZMQ_FOUND )
else()
    message(WARNING "ZeroMQ NOT found. The execution of the BT will not be visible with Groot.")
endif()

if( ZMQ_FOUND )
  list(APPEND BUILD_TOOL_INCLUDE_DIRS ${ZMQ_INCLUDE_DIRS})
endif()

set(DEPENDENCIES
behaviortree_cpp_v3
nav_msgs
nav2_msgs
rclcpp
rclcpp_action
tf2
tf2_geometry_msgs
geometry_msgs
ros2_bt_utils
sensor_msgs
)

add_executable(test_automation_node src/test_automation.cpp)
ament_target_dependencies(test_automation_node ${DEPENDENCIES})

install (TARGETS 
  test_automation_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  bt_xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
