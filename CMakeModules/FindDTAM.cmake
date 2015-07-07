# - Try to find libdtam
#
#  DTAM_FOUND - system has libdtam
#  DTAM_INCLUDE_DIR - the libdtam include directories
#  DTAM_LIBRARY - link these to use libdtam

FIND_PATH(
  DTAM_INCLUDE_DIR
  NAMES dtam/pang.h
  PATHS
    ${CMAKE_SOURCE_DIR}/../dtam
    /usr/include
    /usr/local/include
)


FIND_PATH(
  DTAM_INCLUDE_DIR_BUILD
  NAMES dtam/kernels/sdf/sdf.h
  PATHS
    ${CMAKE_SOURCE_DIR}/../dtam/release/
    ${CMAKE_SOURCE_DIR}/../dtam/build/
    ${CMAKE_SOURCE_DIR}/../dtam/release/
    ${CMAKE_SOURCE_DIR}/../dtam/build/
    /usr/include
    /usr/local/include
)

FIND_LIBRARY(
  DTAM_LIBRARY
  NAMES dtam
  PATHS
    ${CMAKE_SOURCE_DIR}/../dtam/release/dtam
    ${CMAKE_SOURCE_DIR}/../dtam/build/dtam
    ${CMAKE_SOURCE_DIR}/../dtam/release/dtam
    ${CMAKE_SOURCE_DIR}/../dtam/build/dtam
    /usr/lib
    /usr/local/lib
)


IF(DTAM_INCLUDE_DIR AND DTAM_LIBRARY)
  SET(DTAM_FOUND TRUE)
ENDIF()


IF(DTAM_FOUND)
   IF(NOT DTAM_FIND_QUIETLY)
      MESSAGE(STATUS "Found DTAM: ${DTAM_LIBRARY}")
   ENDIF()
ELSE()
   IF(DTAM_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find DTAM")
   ENDIF()
ENDIF()
