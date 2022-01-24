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
find_package(nao_command_msgs REQUIRED)
find_package(nao_sensor_msgs REQUIRED)

add_library(rcss3d_nao_lib
  src/sim_to_nao.cpp
  src/nao_to_sim.cpp
  src/complementary_filter.cpp
  src/nao_joints_pid.cpp)
ament_target_dependencies(rcss3d_nao_lib
  nao_sensor_msgs
  nao_command_msgs
  rcss3d_agent)

add_executable(rcss3d_nao
  src/main.cpp
  src/rcss3d_nao.cpp)
target_include_directories(rcss3d_nao PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(rcss3d_nao
  rcss3d_nao_lib)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS rcss3d_nao
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
