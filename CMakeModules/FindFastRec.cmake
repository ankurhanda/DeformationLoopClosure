# - Try to find libFastRec
#
#  FASTREC_FOUND - system has libFastRec
#  FASTREC_INCLUDE_DIR - the libFastRec include directories
#  FASTREC_LIBRARY - link these to use libFastRec

FIND_PATH(
  FASTREC_INCLUDE_DIR
  NAMES FastRec/global_model_description.h
  PATHS
    ${CMAKE_SOURCE_DIR}/../FastRec
    /usr/include
    /usr/local/include
)


FIND_PATH(
  FASTREC_INCLUDE_DIR_BUILD
  NAMES FastRec/global_model_description.h
  PATHS
    ${CMAKE_SOURCE_DIR}/../FastRec/release/
    ${CMAKE_SOURCE_DIR}/../FastRec/build/
    ${CMAKE_SOURCE_DIR}/../FastRec/release/
    ${CMAKE_SOURCE_DIR}/../FastRec/build/
    /usr/include
    /usr/local/include
)

FIND_LIBRARY(
  FASTREC_LIBRARY
  NAMES FastRec
  PATHS
    ${CMAKE_SOURCE_DIR}/../FastRec/release/FastRec
    ${CMAKE_SOURCE_DIR}/../FastRec/build/FastRec
    ${CMAKE_SOURCE_DIR}/../FastRec/release/FastRec
    ${CMAKE_SOURCE_DIR}/../FastRec/build/FastRec
    /usr/lib
    /usr/local/lib
)


IF(FASTREC_INCLUDE_DIR AND FASTREC_LIBRARY)
  SET(FASTREC_FOUND TRUE)
ENDIF()


IF(FASTREC_FOUND)
   IF(NOT FASTREC_FIND_QUIETLY)
      MESSAGE(STATUS "Found FastRec: ${FASTREC_LIBRARY}")
   ENDIF()
ELSE()
   IF(FASTREC_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find FastRec")
   ENDIF()
ENDIF()
