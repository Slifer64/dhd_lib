# Try to find DHDLibrary

find_path( DHD_INCLUDE_DIR dhdc.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/dhd_sdk/include/
)

find_library( DHD_LIBRARY_DIR
  LIBRARY_NAMES
    libdhd.a
    libdrd.a
  PATHS
  ${CMAKE_CURRENT_SOURCE_DIR}/include/dhd_sdk/lib/
)


if(DHD_INCLUDE_DIR AND DHD_LIBRARY_DIR)

  set( DHDLibrary_FOUND true )

endif()

IF(DHDLibrary_FOUND)
  MESSAGE(STATUS "Found DHD Library: ${DHD_LIBRARY_DIR}")
ELSE()
  MESSAGE(FATAL_ERROR "Could not find DHDLibrary library")
ENDIF()
