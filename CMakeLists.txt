cmake_minimum_required(VERSION 3.8)
project(rcss3d_agent_nao)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rcss3d_agent REQUIRED)
find_package(nao_command_msgs REQUIRED)
find_package(nao_sensor_msgs REQUIRED)

add_executable(rcss3d_agent_nao
  src/rcss3d_agent_nao.cpp
  src/main.cpp)
target_include_directories(rcss3d_agent_nao PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(rcss3d_agent_nao
  rcss3d_agent
  nao_command_msgs
  nao_sensor_msgs)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS rcss3d_agent_nao
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
