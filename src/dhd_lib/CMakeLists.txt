cmake_minimum_required(VERSION 2.8.3)

project(dhd_lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH} )

find_package(catkin REQUIRED)

find_package(DHDLibrary REQUIRED)

catkin_package(
        INCLUDE_DIRS include ${DHD_INCLUDE_DIR}
)

include_directories(
  include
  /usr/include/eigen3
  ${catkin_INCLUDE_DIRS}
  ${DHD_INCLUDE_DIR}
)

#link_directories(include/dhd_sdk/lib/)

add_library(${PROJECT_NAME}
  src/dhd.cpp
  src/sigma7.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${DHD_LIBRARY_DIR}
  #libdhd.a
  #libdrd.a
  pthread
  usb-1.0
  rt
  armadillo
)
