cmake_minimum_required(VERSION 3.22)
project(fr3_replay_global_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Same macOS pluginlib dlopen()-under-ros2-launch RPATH finding as every
# other plugin package here (see fr3_b2_local_planner/CMakeLists.txt).
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "@loader_path")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_hybrid_planning REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(pluginlib REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_action
  moveit_hybrid_planning
  moveit_msgs
  pluginlib
)

add_library(replay_global_planner_plugin SHARED
  src/replay_global_planner.cpp
)
target_include_directories(replay_global_planner_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(replay_global_planner_plugin ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(moveit_hybrid_planning replay_global_planner_plugin.xml)

install(TARGETS replay_global_planner_plugin
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(replay_global_planner_plugin)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
