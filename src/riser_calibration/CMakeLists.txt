cmake_minimum_required(VERSION 3.16)
project(riser_calibration)
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(calibration_controller SHARED src/calibration_controller.cpp)
target_include_directories(calibration_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(calibration_controller PUBLIC cxx_std_17)
ament_target_dependencies(calibration_controller controller_interface pluginlib hardware_interface rclcpp rclcpp_lifecycle)
pluginlib_export_plugin_description_file(controller_interface plugins.xml)
install(TARGETS calibration_controller LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
ament_package()
