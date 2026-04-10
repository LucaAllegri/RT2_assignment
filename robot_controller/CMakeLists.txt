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
  include_directories(include)
  find_package(ament_lint_auto REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_action REQUIRED)
  find_package(rclcpp_components REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(action_interfaces REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)
  
  add_library(client_component SHARED src/action_client.cpp)
  add_library(server_component SHARED src/action_server.cpp)
  add_executable(manual_composition src/manual_composition.cpp)

  target_link_libraries(manual_composition 
    client_component 
    server_component
  )

  ament_target_dependencies(client_component rclcpp rclcpp_action rclcpp_components geometry_msgs action_interfaces tf2 tf2_ros)
  ament_target_dependencies(server_component rclcpp rclcpp_action rclcpp_components geometry_msgs action_interfaces tf2 tf2_ros nav_msgs)
  ament_target_dependencies(manual_composition rclcpp rclcpp_action rclcpp_components geometry_msgs action_interfaces tf2 tf2_ros nav_msgs)

  rclcpp_components_register_node(client_component PLUGIN "robot_controller::RobotActionClient" EXECUTABLE action_client_exe) 
  rclcpp_components_register_node(server_component PLUGIN "robot_controller::RobotActionServer" EXECUTABLE action_server_exe) 

  ament_export_targets(export_${PROJECT_NAME}_client_component)
  ament_export_targets(export_${PROJECT_NAME}_server_component)

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

  install(TARGETS
    manual_composition
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
