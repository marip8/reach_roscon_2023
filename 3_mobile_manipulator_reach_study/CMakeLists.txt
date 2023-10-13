target_sources(${PROJECT_NAME}_plugins PRIVATE
  mm_evaluator.cpp
)

install(DIRECTORY resources DESTINATION share/${PROJECT_NAME}/3_mobile_manipulator_reach_study)
install(FILES reach_study.yaml DESTINATION share/${PROJECT_NAME}/3_mobile_manipulator_reach_study)
