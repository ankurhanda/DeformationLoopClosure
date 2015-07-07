# - Try to find libNazca
#
#  NAZCA_FOUND - system has libNazca
#  NAZCA_INCLUDE_DIR - the libNazca include directories
#  NAZCA_LIBRARY - link these to use libNazca

FIND_PATH(
  NAZCA_INCLUDE_DIR
  NAMES Nazca/nazca.h
  PATHS
    ${CMAKE_SOURCE_DIR}/../Nazca
    /usr/include
    /usr/local/include
)


FIND_PATH(
  NAZCA_INCLUDE_DIR_BUILD
  NAMES Nazca/nazca.h
  PATHS
    ${CMAKE_SOURCE_DIR}/../Nazca/release/
    ${CMAKE_SOURCE_DIR}/../Nazca/build/
    ${CMAKE_SOURCE_DIR}/../Nazca/release/
    ${CMAKE_SOURCE_DIR}/../Nazca/build/
    /usr/include
    /usr/local/include
)

FIND_LIBRARY(
  NAZCA_LIBRARY
  NAMES Nazca
  PATHS
    ${CMAKE_SOURCE_DIR}/../Nazca/release/Nazca
    ${CMAKE_SOURCE_DIR}/../Nazca/build/Nazca
    ${CMAKE_SOURCE_DIR}/../Nazca/release/Nazca
    ${CMAKE_SOURCE_DIR}/../Nazca/build/Nazca
    /usr/lib
    /usr/local/lib
)


IF(NAZCA_INCLUDE_DIR AND NAZCA_LIBRARY)
  SET(NAZCA_FOUND TRUE)
ENDIF()


IF(NAZCA_FOUND)
   IF(NOT NAZCA_FIND_QUIETLY)
      MESSAGE(STATUS "Found Nazca: ${NAZCA_LIBRARY}")
   ENDIF()
ELSE()
   IF(NAZCA_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find Nazca")
   ENDIF()
ENDIF()
