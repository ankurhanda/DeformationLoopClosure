# - Try to find libPTAM
#
#  PTAM_FOUND - system has libPTAM
#  PTAM_INCLUDE_DIR - the libPTAM include directories
#  PTAM_LIBRARY - link these to use libPTAM

FIND_PATH(
  PTAM_INCLUDE_DIR
  NAMES ptam.h
  PATHS
    ${CMAKE_SOURCE_DIR}/../libPTAM/src
    /usr/include
    /usr/local/include
)

FIND_LIBRARY(
  PTAM_LIBRARY
  NAMES ptam
  PATHS
    ${CMAKE_SOURCE_DIR}/../libPTAM/build
    /usr/lib
    /usr/local/lib
)

IF(PTAM_INCLUDE_DIR AND PTAM_LIBRARY)
  SET(PTAM_FOUND TRUE)
ENDIF()


IF(PTAM_FOUND)
   IF(NOT PTAM_FIND_QUIETLY)
      MESSAGE(STATUS "Found ptam: ${PTAM_LIBRARY}")
   ENDIF()
ELSE()
   IF(PTAM_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find PTAM")
   ENDIF()
ENDIF()
