target_sources(${PROJECT_NAME}_plugins PRIVATE
  pnp_ik_solver.cpp
)

install(DIRECTORY resources DESTINATION share/${PROJECT_NAME}/2_pick_and_place_reach_study)
install(FILES reach_study.yaml DESTINATION share/${PROJECT_NAME}/2_pick_and_place_reach_study)
