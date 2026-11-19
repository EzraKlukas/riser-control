cmake_minimum_required(VERSION 3.8)
project(riser_wrench_aggregation)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_srvs REQUIRED)

add_library(wrench_aggregation_controller SHARED src/wrench_aggregation_controller.cpp)
target_include_directories(wrench_aggregation_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(wrench_aggregation_controller PROPERTIES
  CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO)
ament_target_dependencies(wrench_aggregation_controller
  controller_interface hardware_interface pluginlib rclcpp rclcpp_lifecycle std_srvs)
pluginlib_export_plugin_description_file(controller_interface plugins.xml)
install(TARGETS wrench_aggregation_controller LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

include(CTest)
if(BUILD_TESTING)
  add_executable(wrench_tare_test test/wrench_tare_test.cpp)
  target_include_directories(wrench_tare_test PRIVATE include)
  target_compile_features(wrench_tare_test PRIVATE cxx_std_17)
  add_test(NAME wrench_tare COMMAND wrench_tare_test)
  add_executable(wrench_perturbation_test test/wrench_perturbation_test.cpp)
  target_include_directories(wrench_perturbation_test PRIVATE include)
  target_compile_features(wrench_perturbation_test PRIVATE cxx_std_17)
  add_test(NAME wrench_perturbation COMMAND wrench_perturbation_test)
endif()

ament_package()
