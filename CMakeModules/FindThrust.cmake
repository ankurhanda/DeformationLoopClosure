# - Try to find Thrust libray
#
#  THRUST_FOUND - system has Thrust
#  THRUST_INCLUDE_DIR - the Thrust include directories

FIND_PATH(
  THRUST_INCLUDE_DIR
  NAMES thrust/version.h
  PATHS
    ${CMAKE_SOURCE_DIR}/..
    /usr/include
    /usr/local/include
)

IF(THRUST_INCLUDE_DIR)
  SET(THRUST_FOUND TRUE)
ENDIF()

IF(THRUST_FOUND)
   IF(NOT THRUST_FIND_QUIETLY)
      MESSAGE(STATUS "Found Thrust: ${THRUST_INCLUDE_DIR}")
   ENDIF()
ELSE()
   IF(THRUST_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find Thrust")
   ENDIF()
ENDIF()
