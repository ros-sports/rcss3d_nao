cmake_minimum_required(VERSION 3.8)
project(rcss3d_nao)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rcss3d_agent REQUIRED)
find_package(rcss3d_agent_msgs_to_soccer_interfaces REQUIRED)
find_package(nao_lola_command_msgs REQUIRED)
find_package(nao_lola_sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(soccer_vision_3d_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rcss3d_agent
  rcss3d_agent_msgs_to_soccer_interfaces
  nao_lola_command_msgs
  nao_lola_sensor_msgs
  rclcpp_components
  soccer_vision_3d_msgs
  geometry_msgs)

# Build
add_library(${PROJECT_NAME}_node SHARED
  src/rcss3d_nao_node.cpp
  src/sim_to_nao.cpp
  src/nao_to_sim.cpp
  src/complementary_filter.cpp
  src/nao_joints_pid.cpp)
target_include_directories(rcss3d_nao_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "rcss3d_nao::Rcss3dNaoNode"
  EXECUTABLE ${PROJECT_NAME})

ament_target_dependencies(${PROJECT_NAME}_node ${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

# Install
install(
  DIRECTORY include/
  DESTINATION include)
install(
  TARGETS ${PROJECT_NAME}_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

# Test
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Package
ament_package()
