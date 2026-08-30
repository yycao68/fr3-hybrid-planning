cmake_minimum_required(VERSION 3.22)
project(fr3_gravity_ff_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Deliberately NOT linking fr3_dynamics here (see kdl_chain_dynamics.hpp's
# own header comment): its moveit_core dependency was the FIRST suspect
# for a live dlopen "symbol not found in flat namespace '_PyExc_RuntimeError'"
# error, but removing it did NOT fix the error -- root cause turned out to
# be simpler: ament_target_dependencies() links each message package's
# FULL exported interface, which on this distro includes each package's
# own *_rosidl_generator_py library (real CPython C-API bindings, needing
# libpython at load time -- confirmed via `otool -L`, which shows this
# library linking a long chain of *_rosidl_generator_py libs that the
# officially pre-packaged joint_trajectory_controller.dylib does NOT link
# at all). component_container_mt (where B2/B3 load fine) apparently has
# Python loaded transitively via some other already-loaded plugin;
# mujoco_ros2_control_node -- a lean executable that never touches Python
# -- does not, so the flat-namespace symbol lookup fails there specifically.
# This package duplicates the small amount of KDL setup it needs directly
# (kdl_chain_dynamics.cpp) rather than depending on fr3_dynamics/moveit_core,
# which is unrelated to this fix but avoids an unnecessary heavy dependency.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(orocos_kdl REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  pluginlib
  realtime_tools
  trajectory_msgs
  kdl_parser
  urdf
  orocos_kdl_vendor
)

add_library(gravity_feedforward_controller SHARED
  src/gravity_feedforward_controller.cpp
  src/kdl_chain_dynamics.cpp
)
target_include_directories(gravity_feedforward_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(gravity_feedforward_controller ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Two related missing-symbol fixes, same root cause (bare/underlinked
# library names this distro's exported CMake configs reference that
# aren't on the default linker search path, even though the actual
# .dylib files DO exist in this conda env's own lib/): (1) rcutils's own
# exported INTERFACE_LINK_LIBRARIES bare-names "atomic" unconditionally
# (rcutilsExport.cmake) -- neither fr3_b2_local_planner nor
# fr3_b3_local_planner hit this (their dependency set never surfaces it),
# but this package's realtime_tools/controller_interface/hardware_interface
# deps do. (2) libpython, needed at dlopen/load time by the
# *_rosidl_generator_py libraries pulled in above (see this file's own
# comment on that) -- confirmed present via `find ... -iname libpython*`.
# Both resolved by pointing the linker at the conda env's own lib dir
# directly (derived from where rclcpp itself was actually found, robust
# to $CONDA_PREFIX resolving to the wrong (base) env in this shell setup
# -- confirmed live, same conda-activation quirk documented elsewhere in
# this workspace). External review finding: this used to link `python3.12`
# by name, which would silently stop resolving on a future RoboStack Python
# version bump; find_package(Python3)+Python3::Python below resolves
# whatever interpreter this env's rclcpp was actually built against instead.
get_filename_component(_ros_env_root "${rclcpp_DIR}/../../.." ABSOLUTE)  # share/rclcpp/cmake -> env root
target_link_directories(gravity_feedforward_controller PRIVATE "${_ros_env_root}/lib")
find_package(Python3 COMPONENTS Development REQUIRED)
target_link_libraries(gravity_feedforward_controller Python3::Python)

pluginlib_export_plugin_description_file(controller_interface gravity_feedforward_controller_plugin.xml)

install(TARGETS gravity_feedforward_controller
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(gravity_feedforward_controller)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
