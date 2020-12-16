cmake_minimum_required(VERSION 3.5)
project(ars408_radar)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



##########################################################
#######             INCLUDES AND LIBS            #########
#######             find dependencies
##########################################################
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(can_msgs REQUIRED)
find_package(ros2socketcan_bridge REQUIRED)
find_package(ars408_radar_msgs REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(builtin_interfaces REQUIRED)


##########################################################
#######             INCLUDES AND LIBS            #########
##########################################################
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${rclcpp_lifecycle_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${ros2socketcan_bridge_INCLUDE_DIRS}
  ${rmw_implementation_INCLUDE_DIRS}
)


##########################################################
#######         Component and Executable         #########
##########################################################

add_library(ars408_radar_component SHARED ${CMAKE_CURRENT_SOURCE_DIR}/src/ars408_radar_component.cpp)
target_compile_definitions(ars408_radar_component PRIVATE "ARS408_RADAR_BUILDING_DLL")
target_link_libraries(ars408_radar_component dl -lboost_system
    ${rclcpp_lifecycle_LIBRARIES}
    ${ament_index_cpp_LIBRARIES}
)

ament_target_dependencies(
  ars408_radar_component
  ars408_radar_msgs
  visualization_msgs
  tf2_ros
  rclcpp_lifecycle
  rclcpp
  std_msgs
  rclcpp_components
  geometry_msgs
  ament_index_cpp
  can_msgs
  ros2socketcan_bridge
)

rclcpp_components_register_nodes(ars408_radar_component "FHAC::ars408_radar")
set(node_plugins "${node_plugins}FHAC::ars408_radar;$<TARGET_FILE:ars408_radar_component>\n")


add_executable(ars408_radar_composition ${CMAKE_CURRENT_SOURCE_DIR}/src/ars408_radar_composition.cpp)
target_link_libraries(
  ars408_radar_composition
  ars408_radar_component
)

ament_target_dependencies(ars408_radar_composition
  "rclcpp"
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

##########################################################
##############         INSTALLS         ##################
##########################################################
# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
#launch file location : ~/robot_ws/install/ars408_radar/share/ars408_radar/launch 
##########################################################

##########################################################
###########      Register package       ##################
##########################################################
ament_package()
