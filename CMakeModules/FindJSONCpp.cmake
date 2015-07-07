# - Try to find JSONCpp
#
#  JSONCpp_FOUND - system has JSONCpp
#  JSONCpp_INCLUDE_DIR - the JSONCpp include directories
#  JSONCpp_LIBRARY - link these to use JSONCpp

FIND_PATH(
  JSONCpp_INCLUDE_DIR
  NAMES json/json.h
  PATHS
    ${CMAKE_SOURCE_DIR}/external/jsoncpp/include
    /usr/include
    /usr/local/include
)

FIND_LIBRARY(
  JSONCpp_LIBRARY
  NAMES json
  PATHS
    ${CMAKE_SOURCE_DIR}/external/jsoncpp/lib
    /usr/lib
    /usr/local/lib
) 

IF(JSONCpp_INCLUDE_DIR AND JSONCpp_LIBRARY)
  SET(JSONCpp_FOUND TRUE)
ENDIF()

IF(JSONCpp_FOUND)
   IF(NOT JSONCpp_FIND_QUIETLY)
      MESSAGE(STATUS "Found JSONCpp: ${JSONCpp_LIBRARY}")
   ENDIF()
ELSE()
   IF(JSONCpp_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find JSONCpp")
   ENDIF()
ENDIF()
