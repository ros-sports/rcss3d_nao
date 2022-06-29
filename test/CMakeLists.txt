# Build test_nao_to_sim
ament_add_gtest(test_nao_to_sim
  test_nao_to_sim.cpp)

target_link_libraries(test_nao_to_sim
  ${PROJECT_NAME}_node)

# Build test_sim_to_nao
ament_add_gtest(test_sim_to_nao
  test_sim_to_nao.cpp)

target_link_libraries(test_sim_to_nao
  ${PROJECT_NAME}_node)

# Build test_joint_pid
ament_add_gtest(test_joint_pid
  test_joint_pid.cpp)

target_link_libraries(test_joint_pid
  ${PROJECT_NAME}_node)
