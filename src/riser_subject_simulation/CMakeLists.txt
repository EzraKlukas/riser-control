cmake_minimum_required(VERSION 3.16)
project(riser_subject_simulation)

find_package(ament_cmake REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)

add_library(riser_subject_wrench SHARED src/wrench_plugin.cpp)
set_target_properties(riser_subject_wrench PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
  )
target_include_directories(riser_subject_wrench PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(riser_subject_wrench PRIVATE
  gz-sim::gz-sim
  gz-plugin::register
  gz-transport::gz-transport
  gz-msgs::gz-msgs)

install(TARGETS riser_subject_wrench LIBRARY DESTINATION lib)
install(PROGRAMS scripts/wrench_input.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME simulation_wrench_input)

ament_package()
