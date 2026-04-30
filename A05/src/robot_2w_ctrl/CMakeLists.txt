cmake_minimum_required(VERSION 3.5)
project(robot_2w_ctrl)

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


# find dependencies
set(DEPENDS
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  std_srvs
  robot_2w_interfaces
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


# External libraries
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

## Build
#Library 

add_library(
  ${PROJECT_NAME}
  SHARED
  src/Robot2WCtrl.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  ${PROJECT_NAME}
  ${DEPENDS}
  Eigen3
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)


# Executables
set(APP_TARGETS
robot_2w_control_node
)

foreach(target ${APP_TARGETS})
  add_executable(${target} src/Applications/${target}.cpp)
  # Dependencies by package name
  ament_target_dependencies(${target} rclcpp ${DEPENDS} Eigen3)
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
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
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
  DIRECTORY launch configs rviz
  DESTINATION share/${PROJECT_NAME}
)

# Downstream exports
ament_export_include_directories(include)

ament_export_libraries(${PROJECT_NAME})

ament_export_dependencies(ament_cmake ${DEPENDS})

ament_package()
