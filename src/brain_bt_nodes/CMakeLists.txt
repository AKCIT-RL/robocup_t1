cmake_minimum_required(VERSION 3.16...3.27)
project(brain_bt_nodes)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unknown-pragmas)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(brain REQUIRED)  # For typesupport and msgs
find_package(booster_interface REQUIRED)
find_package(vision_interface REQUIRED)
find_package(game_controller_interface REQUIRED)
find_package(booster_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rerun_sdk REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(backward_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Get brain source directory
set(BRAIN_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../brain/src")
set(BRAIN_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../brain/include")

# Source files from brain package (compiled directly into plugin)
set(BRAIN_SOURCES
  ${BRAIN_SRC_DIR}/brain.cpp
  ${BRAIN_SRC_DIR}/brain_communication.cpp
  ${BRAIN_SRC_DIR}/brain_config.cpp
  ${BRAIN_SRC_DIR}/brain_data.cpp
  ${BRAIN_SRC_DIR}/brain_log.cpp
  ${BRAIN_SRC_DIR}/brain_tree.cpp
  ${BRAIN_SRC_DIR}/localization_fusion.cpp
  ${BRAIN_SRC_DIR}/locator.cpp
  ${BRAIN_SRC_DIR}/robot_client.cpp
)

# Plugin source files
set(PLUGIN_SOURCES
  src/plugin.cpp
  ${BRAIN_SOURCES}
)

# Create shared library (plugin)
add_library(${PROJECT_NAME} SHARED ${PLUGIN_SOURCES})

# Export BehaviorTree.CPP plugin symbols
target_compile_definitions(${PROJECT_NAME} PRIVATE BT_PLUGIN_EXPORT)

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${BRAIN_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

# Link libraries
# NOTE: Aggressive linker flags to force static library linking into shared library
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined")
target_link_libraries(${PROJECT_NAME}
  -Wl,--whole-archive
  rerun_sdk
  -Wl,--no-whole-archive
  ${OpenCV_LIBS}
  yaml-cpp
  ${backward_ros_LIBRARIES}
)

# Add compiler flags for position independent code
set_target_properties(${PROJECT_NAME} PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  COMPILE_FLAGS "-fPIC -fno-strict-aliasing"
)

# ROS 2 dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  behaviortree_cpp
  brain
  booster_interface
  vision_interface
  game_controller_interface
  booster_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Suppress warnings (matching brain package)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-narrowing -Wno-unused-parameter -Wno-sign-compare -Wno-missing-field-initializers -Wno-unused-variable")

# Install plugin library
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Export targets
ament_export_targets(${PROJECT_NAME}_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  behaviortree_cpp
  brain
  booster_interface
  vision_interface
  game_controller_interface
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
