################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(cugo_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(JetsonGPIO REQUIRED)

################################################################################
# Build
################################################################################
include_directories(
  include
)

add_library(${PROJECT_NAME}_lib
  "src/cugo.cpp"
)

set(DEPENDENCIES
  "geometry_msgs"
  "message_filters"
  "nav_msgs"
  "rclcpp"
  "rcutils"
  "std_msgs"
  "std_srvs"
  "tf2"
  "tf2_ros"
  "JetsonGPIO"
)

target_link_libraries(${PROJECT_NAME}_lib)
ament_target_dependencies(${PROJECT_NAME}_lib ${DEPENDENCIES})

set(EXECUTABLE_NAME "cugo_ros")

add_executable(${EXECUTABLE_NAME} src/node_main.cpp)
target_link_libraries(${EXECUTABLE_NAME} ${PROJECT_NAME}_lib)
ament_target_dependencies(${EXECUTABLE_NAME} ${DEPENDENCIES})

################################################################################
# Install
################################################################################

install(TARGETS ${EXECUTABLE_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_package()
