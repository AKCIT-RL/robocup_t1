cmake_minimum_required(VERSION 3.5)
project(robocup_bringup)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install parameter files
install(DIRECTORY
  param
  DESTINATION share/${PROJECT_NAME}
)

# Install rviz configs (if exists)
# Uncomment when rviz config is created
# install(DIRECTORY
#   rviz
#   DESTINATION share/${PROJECT_NAME}
# )

ament_package()
