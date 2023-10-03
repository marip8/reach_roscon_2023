find_package(reach_ros REQUIRED)

# Add the plugin library
add_library(${PROJECT_NAME}_plugins SHARED
  PNP_IKSolver.cpp
)
ament_target_dependencies(${PROJECT_NAME}_plugins PUBLIC reach_ros)

install(TARGETS ${PROJECT_NAME}_plugins DESTINATION lib)
install(DIRECTORY resources DESTINATION share/${PROJECT_NAME}/2_pick_and_place_reach_study)
