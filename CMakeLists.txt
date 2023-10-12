cmake_minimum_required(VERSION 3.16)
project(reach_roscon_2023)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(reach REQUIRED)
find_package(reach_ros REQUIRED)

add_library(${PROJECT_NAME}_plugins SHARED)
ament_target_dependencies(${PROJECT_NAME}_plugins PUBLIC reach_ros)

add_subdirectory(2_pick_and_place_reach_study)
add_subdirectory(3_mobile_manipulator_reach_study)

install(TARGETS ${PROJECT_NAME}_plugins DESTINATION lib)

install(
  DIRECTORY
    0_setup
    1_standard_reach_study
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_package()
