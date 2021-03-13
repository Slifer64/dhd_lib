cmake_minimum_required(VERSION 3.0.0)

project(dhd_test)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wno-narrowing")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  dhd_lib
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(multiple_devices
  src/multiple_devices.cpp
)

add_dependencies(multiple_devices
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(multiple_devices
  ${catkin_LIBRARIES}
  dhd_lib
)


# ===================================================
