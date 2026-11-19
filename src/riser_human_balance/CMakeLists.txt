cmake_minimum_required(VERSION 3.16)
project(riser_human_balance)

find_package(ament_cmake REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)
find_package(riser_gazebo REQUIRED)

add_library(riser_human_balance SHARED src/human_balance_system.cpp)
target_include_directories(riser_human_balance PRIVATE include)
target_compile_features(riser_human_balance PRIVATE cxx_std_17)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(riser_human_balance PRIVATE -Wall -Wextra -Wpedantic)
endif()
ament_target_dependencies(riser_human_balance riser_gazebo)
target_link_libraries(riser_human_balance
  gz-sim::gz-sim gz-plugin::register gz-transport::gz-transport gz-msgs::gz-msgs)

install(TARGETS riser_human_balance LIBRARY DESTINATION lib)
install(DIRECTORY config models DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/spawn_participant.py DESTINATION lib/${PROJECT_NAME}
  RENAME spawn_participant)

include(CTest)
if(BUILD_TESTING)
  add_executable(ankle_balance_test test/ankle_balance_test.cpp)
  target_include_directories(ankle_balance_test PRIVATE include)
  target_compile_features(ankle_balance_test PRIVATE cxx_std_17)
  add_test(NAME ankle_balance COMMAND ankle_balance_test)
endif()

ament_package()
