cmake_minimum_required(VERSION 3.22)
project(fr3_b3_local_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# @rpath resolution of sibling-package dylibs (fr3_dynamics) at dlopen time
# was found to be unreliable under ros2 launch's composable-node loading on
# macOS (DYLD_LIBRARY_PATH from the sourced overlay did not reliably reach
# the pluginlib dlopen() call inside the running container process). Embed
# an explicit, environment-independent install RPATH instead.
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "@loader_path;@loader_path/../../fr3_dynamics/lib")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_hybrid_planning REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(pluginlib REQUIRED)
find_package(fr3_dynamics REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(diagnostic_msgs REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  moveit_hybrid_planning
  moveit_core
  moveit_ros_planning
  pluginlib
  fr3_dynamics
  orocos_kdl_vendor
  diagnostic_msgs
)

add_library(horizon_trajectory_operator_plugin SHARED
  src/horizon_trajectory_operator.cpp
  src/torque_margin_certificate.cpp
  src/route_retime_search.cpp
  src/reshape_qp.cpp
  src/via_point_trajectory.cpp
)
target_include_directories(horizon_trajectory_operator_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(horizon_trajectory_operator_plugin ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(horizon_trajectory_operator_plugin OsqpEigen::OsqpEigen)

add_library(b3_constraint_solver_plugin SHARED
  src/b3_constraint_solver.cpp
  src/torque_margin_certificate.cpp
  src/reshape_qp.cpp
)
target_include_directories(b3_constraint_solver_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(b3_constraint_solver_plugin ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(b3_constraint_solver_plugin OsqpEigen::OsqpEigen)

pluginlib_export_plugin_description_file(moveit_hybrid_planning horizon_trajectory_operator_plugin.xml)
pluginlib_export_plugin_description_file(moveit_hybrid_planning b3_constraint_solver_plugin.xml)

install(TARGETS horizon_trajectory_operator_plugin b3_constraint_solver_plugin
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(horizon_trajectory_operator_plugin b3_constraint_solver_plugin)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
