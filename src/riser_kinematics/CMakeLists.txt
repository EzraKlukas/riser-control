cmake_minimum_required(VERSION 3.16)
project(riser_kinematics VERSION 0.1.0 LANGUAGES CXX)

# OFF allows the same library/tests to build on a host without ROS.
option(RISER_KINEMATICS_AMENT "Export the ROS package through ament" ON)
if(RISER_KINEMATICS_AMENT)
  find_package(ament_cmake REQUIRED)
endif()
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(geometry "${CMAKE_CURRENT_SOURCE_DIR}/../../evidence/model_reference/actuator_mapping.json")
add_custom_command(OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/geometry.cpp"
  COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake/generate_geometry.py"
    "${geometry}" "${CMAKE_CURRENT_BINARY_DIR}/geometry.cpp"
  DEPENDS "${geometry}" cmake/generate_geometry.py VERBATIM)
add_library(riser_kinematics src/mapping.cpp src/lateral.cpp "${CMAKE_CURRENT_BINARY_DIR}/geometry.cpp")
add_library(riser_kinematics::riser_kinematics ALIAS riser_kinematics)
set_target_properties(riser_kinematics PROPERTIES CXX_STANDARD 17
  CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO POSITION_INDEPENDENT_CODE YES)
target_compile_features(riser_kinematics PUBLIC cxx_std_17)
target_include_directories(riser_kinematics PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>" "$<INSTALL_INTERFACE:include>")
install(TARGETS riser_kinematics EXPORT export_riser_kinematics
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES "${geometry}" DESTINATION share/${PROJECT_NAME}/config)
install(FILES README.md LICENSE DESTINATION share/${PROJECT_NAME})

include(CTest)
if(BUILD_TESTING)
  add_executable(mapping_test test/mapping_test.cpp)
  target_link_libraries(mapping_test PRIVATE riser_kinematics)
  add_test(NAME mapping COMMAND mapping_test)
  add_test(NAME python_parity_and_provenance COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/test/parity.py" "$<TARGET_FILE:mapping_test>"
    "${CMAKE_CURRENT_SOURCE_DIR}/../..")
endif()
if(RISER_KINEMATICS_AMENT)
  ament_export_targets(export_riser_kinematics HAS_LIBRARY_TARGET NAMESPACE riser_kinematics::)
  ament_package()
else()
  install(EXPORT export_riser_kinematics NAMESPACE riser_kinematics::
    DESTINATION lib/cmake/riser_kinematics)
endif()
