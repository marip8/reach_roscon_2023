cmake_minimum_required(VERSION 3.5)
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

install(
  DIRECTORY
    0_setup
    1_standard_reach_study
    2_pick_and_place_reach_study
    3_mobile_manipulator_reach_study
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_package()
