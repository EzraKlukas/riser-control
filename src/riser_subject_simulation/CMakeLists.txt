cmake_minimum_required(VERSION 3.16)
project(riser_subject_simulation)

find_package(ament_cmake REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)

add_library(riser_subject_wrench SHARED src/wrench_plugin.cpp)
set_target_properties(riser_subject_wrench PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
  )
target_include_directories(riser_subject_wrench PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(riser_subject_wrench PRIVATE
  ignition-gazebo6::ignition-gazebo6
  ignition-plugin1::ignition-plugin1)

install(TARGETS riser_subject_wrench LIBRARY DESTINATION lib)
install(PROGRAMS scripts/wrench_input.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME simulation_wrench_input)

ament_package()
