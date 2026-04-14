cmake_minimum_required(VERSION 3.8)
project(navigation_pkg)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(navi_interfaces REQUIRED)
add_library(
 ${PROJECT_NAME}
 SHARED
 src/NaviPublisher.cpp
 src/NaviSubscriber.cpp
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
 navi_interfaces
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

add_executable(navi_node src/navi_node.cpp)
ament_target_dependencies(navi_node rclcpp navi_interfaces)

add_executable(sub_navi_node src/sub_navi_node.cpp)
ament_target_dependencies(sub_navi_node rclcpp navi_interfaces)

target_include_directories(navi_node PUBLIC
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 $<INSTALL_INTERFACE:include>)
target_compile_features(navi_node PUBLIC c_std_99 cxx_std_17)

target_link_libraries(navi_node ${PROJECT_NAME}) 

target_include_directories(sub_navi_node PUBLIC
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 $<INSTALL_INTERFACE:include>)
target_compile_features(sub_navi_node PUBLIC c_std_99 cxx_std_17)

target_link_libraries(sub_navi_node ${PROJECT_NAME}) 

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

# Downstream exports
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(ament_cmake rclcpp navi_interfaces)

ament_package()
