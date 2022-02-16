cmake_minimum_required(VERSION 3.5)
project(sunrisedds_servoing_hardware)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(angles REQUIRED)
find_package(CycloneDDS REQUIRED)

add_executable(sunrisedds_servoing_hardware_sim
  src/sunrisedds_servoing_hardware_sim.cpp 
  src/Header.c
  src/JointPosition.c
  src/JointQuantity.c
  src/Time.c
  ) 
target_include_directories(sunrisedds_servoing_hardware_sim PUBLIC include)
target_link_libraries(sunrisedds_servoing_hardware_sim CycloneDDS::ddsc)

add_library(sunrisedds_servoing_hardware SHARED 
  src/sunrisedds_servoing_hardware.cpp 
  src/Header.c
  src/JointPosition.c
  src/JointQuantity.c
  src/Time.c
  ) 
target_include_directories(sunrisedds_servoing_hardware PUBLIC include)
target_link_libraries(sunrisedds_servoing_hardware CycloneDDS::ddsc)
ament_target_dependencies(sunrisedds_servoing_hardware
                          hardware_interface
                          pluginlib
                          rclcpp)

# Causes the visibility macros to use dllexport rather than dllimport, which is
# appropriate when building the dll but not consuming it.
target_compile_definitions(sunrisedds_servoing_hardware PRIVATE
                           "SUNRISEDDS_SERVOING_HARDWARE_BUILDING_DLL")

install(DIRECTORY include/ DESTINATION include)

install(TARGETS sunrisedds_servoing_hardware
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(TARGETS sunrisedds_servoing_hardware_sim
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(DIRECTORY config DESTINATION share/sunrisedds_servoing_hardware)

pluginlib_export_plugin_description_file(hardware_interface sunrisedds_servoing_hardware.xml)

ament_export_libraries(sunrisedds_servoing_hardware)
ament_export_include_directories(include)
ament_export_dependencies(hardware_interface pluginlib rclcpp)
ament_package()
