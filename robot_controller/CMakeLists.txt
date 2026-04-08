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
  find_package(robot_controller REQUIRED)

  add_library(client_component SHARED src/action_client.cpp)
  add_library(server_component SHARED src/action_server.cpp)
  add_library(broadcaster_component SHARED src/broadcaster.cpp)
  add_executable(main src/main.cpp)

  ament_target_dependencies(client_component rclcpp rclcpp_action rclcpp_components action_interfaces message_custom)
  ament_target_dependencies(server_component rclcpp rclcpp_action rclcpp_components geometry_msgs action_interfaces tf2 tf2_ros)
  ament_target_dependencies(broadcaster_component rclcpp rclcpp_components geometry_msgs message_custom tf2 tf2_ros nav_msgs)
  ament_target_dependencies(main rclcpp robot_controller)

  rclcpp_components_register_node(client_component PLUGIN "robot_controller::RobotActionClient" EXECUTABLE action_client_exe) 
  rclcpp_components_register_node(server_component PLUGIN "robot_controller::RobotActionServer" EXECUTABLE action_server_exe) 
  rclcpp_components_register_node(broadcaster_component PLUGIN "robot_controller::FramesPublisher" EXECUTABLE broadcaster_component_exe) 

  ament_export_targets(export_${PROJECT_NAME}_client_component)
  ament_export_targets(export_${PROJECT_NAME}_server_component)
  ament_export_targets(export_${PROJECT_NAME}_broadcaster_component)

  install (TARGETS client_component
          EXPORT export_${PROJECT_NAME}_client_component
          ARCHIVE DESTINATION lib
          LIBRARY DESTINATION lib
          RUNTIME DESTINATION bin)

  install (TARGETS server_component
          EXPORT export_${PROJECT_NAME}_server_component
          ARCHIVE DESTINATION lib
          LIBRARY DESTINATION lib
          RUNTIME DESTINATION bin)

  install (TARGETS broadcaster_component
          EXPORT export_${PROJECT_NAME}_broadcaster_component
          ARCHIVE DESTINATION lib
          LIBRARY DESTINATION lib
          RUNTIME DESTINATION bin)


  install(TARGETS
    main
    DESTINATION lib/${PROJECT_NAME}
  )


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
