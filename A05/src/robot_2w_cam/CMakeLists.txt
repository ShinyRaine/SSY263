cmake_minimum_required(VERSION 3.8)
project(robot_2w_cam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(robot_2w_interfaces REQUIRED)

# TF pacakges. Needed to convert Eigen into TF, for example Rotation to Quaternion msg
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# External libraries
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

## Build
#Library 

add_library(
  ${PROJECT_NAME}
  SHARED
  src/Robot2WCam.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  geometry_msgs
  nav_msgs
  Eigen3
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  robot_2w_interfaces
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Executables
set(APP_TARGETS
robot_2w_cam_sim_node
)

foreach(target ${APP_TARGETS})
  add_executable(${target} src/Applications/${target}.cpp)
  # Dependencies by package name
  ament_target_dependencies(${target} rclcpp Eigen3)
  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  # target_compile_options(${target} PRIVATE -Wall)
  # Other Libraries
  # Dependencies by lib name
  target_link_libraries(${target} ${PROJECT_NAME})
  # target_include_directories(${target} PRIVATE
  #     ${OTHER__INCLUDES}
  # )
  # NOTE: To link agains messages generated in the same package use: 
  # rosidl_target_interfaces(${target}
  # ${PROJECT_NAME} "rosidl_typesupport_cpp")
endforeach()


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

## Install 

install(DIRECTORY include/
  DESTINATION include)

install(
  TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS
${APP_TARGETS}
DESTINATION lib/${PROJECT_NAME}
)

# Install the launch and config file so ROS2 can find it
install(
  DIRECTORY configs launch
  DESTINATION share/${PROJECT_NAME}
)

# Downstream exports
ament_export_include_directories(include)

ament_export_libraries(${PROJECT_NAME})

ament_export_dependencies(ament_cmake rclcpp geometry_msgs nav_msgs tf2 tf2_ros tf2_eigen tf2_geometry_msgs robot_2w_interfaces)


ament_package()
