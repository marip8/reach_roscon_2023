install(DIRECTORY resources DESTINATION share/${PROJECT_NAME}/3_mobile_manipulator_reach_study)
install(
  FILES
    reach_study.yaml
    reach_study.xacro
    reach_study.srdf
  DESTINATION
    share/${PROJECT_NAME}/3_mobile_manipulator_reach_study
)
