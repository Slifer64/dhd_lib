# Try to find DHDLibrary

find_path( DHD_INCLUDE_DIR dhdc.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/dhd_sdk/include/
)

find_library( DHD_LIBRARY
  LIBRARY_NAMES
    libdhd.a
  PATHS
  ${CMAKE_CURRENT_SOURCE_DIR}/include/dhd_sdk/lib/
)

find_library( DRD_LIBRARY
  LIBRARY_NAMES
    libdrd.a
  PATHS
  ${CMAKE_CURRENT_SOURCE_DIR}/include/dhd_sdk/lib/
)

if(DHD_INCLUDE_DIR AND DHD_LIBRARY AND DRD_LIBRARY)

  set(DHD_LIBRARIES
    ${DHD_LIBRARY}
    ${DRD_LIBRARY})

  set( DHDLibrary_FOUND true )

endif()

IF(DHDLibrary_FOUND)
  MESSAGE(STATUS "Found DHD Library: ${DHD_LIBRARIES}")
ELSE()
  MESSAGE(FATAL_ERROR "Could not find DHDLibrary library")
ENDIF()
