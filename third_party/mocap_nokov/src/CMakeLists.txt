add_executable(${PROJECT_NAME}_mocap_node
version.cpp
mocap_node.cpp
data_model.cpp
mocap_config.cpp
rigid_body_publisher.cpp
)

target_link_libraries(${PROJECT_NAME}_mocap_node 
${rclcpp_LIBRARIES}
${geometry_msgs_LIBRARIES}
${tf2_ros_LIBRARIES}
${nokov_sdk_LIBRARIES})

set_target_properties(${PROJECT_NAME}_mocap_node PROPERTIES
                    OUTPUT_NAME mocap_node PREFIX "")
set_target_properties(${PROJECT_NAME}_mocap_node PROPERTIES
  INSTALL_RPATH "$ORIGIN/../../share/mocap_nokov/x64"
)
#target_compile_definitions(
#  ${PROJECT_NAME}_mocap_node PRIVATE
#  RCLCPP_LOG_MIN_SEVERITY=RCLCPP_LOG_MIN_SEVERITY_FATAL
#)

install(
TARGETS ${PROJECT_NAME}_mocap_node
DESTINATION lib/${PROJECT_NAME}
)
