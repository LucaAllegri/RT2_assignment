cmake_minimum_required(VERSION 3.8)
project(robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_action REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(action_interfaces REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(rclcpp_components REQUIRED)
  find_package(message_custom REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)

  #add_library(action_client SHARED src/action_client.cpp)
  #add_library(action_server SHARED src/action_server.cpp)
  add_executable(broadcaster src/broadcaster.cpp)
  add_executable(action_client src/action_client.cpp)
  add_executable(action_server src/action_server.cpp)

  ament_target_dependencies(action_client rclcpp rclcpp_action action_interfaces message_custom)
  ament_target_dependencies(action_server rclcpp rclcpp_action rclcpp_components geometry_msgs nav_msgs action_interfaces message_custom tf2 tf2_ros)
  ament_target_dependencies(broadcaster rclcpp geometry_msgs message_custom tf2 tf2_ros)

  #rclcpp_components_register_node(action_client PLUGIN "robot_controller::RobotActionClient" EXECUTABLE action_client_exe) 
  #rclcpp_components_register_node(action_server PLUGIN "robot_controller::RobotActionServer" EXECUTABLE action_server_exe) 

  #ament_export_targets(export_${PROJECT_NAME}_action_client)
  #ament_export_targets(export_${PROJECT_NAME}_action_server)

  #install (TARGETS action_client
  #        EXPORT export_${PROJECT_NAME}_action_client
  #        ARCHIVE DESTINATION lib
  #        LIBRARY DESTINATION lib
  #        RUNTIME DESTINATION bin)

  install(TARGETS
    action_client
    action_server
    broadcaster
    DESTINATION lib/${PROJECT_NAME}
  )

  #install (TARGETS action_server
  #        EXPORT export_${PROJECT_NAME}_action_server
  #        ARCHIVE DESTINATION lib
  #        LIBRARY DESTINATION lib
  #        RUNTIME DESTINATION bin)



  install( 
    DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )




  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
