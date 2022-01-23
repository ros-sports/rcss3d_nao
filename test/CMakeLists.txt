# Build test_nao_to_sim
# ament_add_gtest(test_nao_to_sim
#     test_nao_to_sim.cpp)

# target_link_libraries(test_nao_to_sim
#     rcss3d_nao_lib
# )

# Build test_sim_to_nao
ament_add_gtest(test_sim_to_nao
  test_sim_to_nao.cpp)

target_link_libraries(test_sim_to_nao
    rcss3d_agent_nao_lib
)