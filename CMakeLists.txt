# Copyright(c) 2020 eaibot limited.
cmake_minimum_required(VERSION 3.5)
project(syntron_driver)

##################ros2#############################################
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

####################find package#####################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(rmw REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(visualization_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

############## YDLIDAR SDK START#####################################
#find ydlidar_sdk package
# find_package(ydlidar_sdk REQUIRED)
############## YDLIDAR SDK END#####################################

#Include directories
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/include)

#link library directories
# link_directories(${YDLIDAR_SDK_LIBRARY_DIRS})

#---------------------------------------------------------------------------------------
# generate excutable and add libraries
#---------------------------------------------------------------------------------------
add_executable(syntron_driver_node
    src/syntron_driver_node.cpp
    src/syntron_driver.cpp
    src/syntron_motor.cpp
    src/syntron_can_com.cpp
    src/socketcan.cpp)
#---------------------------------------------------------------------------------------
# link libraries
#--------------------------------------------------------------------------------------
ament_target_dependencies(syntron_driver_node
    "rclcpp"
    # "sensor_msgs"
    # "visualization_msgs"
    # "geometry_msgs"
    "std_srvs"
    )

#---------------------------------------------------------------------------------------
# Install
#---------------------------------------------------------------------------------------
install(TARGETS
syntron_driver_node
    DESTINATION lib/${PROJECT_NAME})

# install(DIRECTORY launch
#     DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()

