# - Try to find libfreenect
#

#MESSAGE(STATUS ${CMAKE_SOURCE_DIR}/../libfreenect/include)

FIND_PATH(
  freenect_INCLUDE_DIR
  NAMES libfreenect.h
  PATHS
    ${CMAKE_SOURCE_DIR}/../libfreenect/src
    ${CMAKE_SOURCE_DIR}/../libfreenect/include
    /usr/include
    /usr/local/include
)

FIND_LIBRARY(
  freenect_LIBRARY
  NAMES freenect
  PATHS
    ${CMAKE_SOURCE_DIR}/../libfreenect/build/lib
    /usr/lib
    /usr/local/lib
)


IF(freenect_INCLUDE_DIR AND freenect_LIBRARY)
  SET(freenect_FOUND TRUE)
ENDIF()


IF(freenect_FOUND)
   IF(NOT freenect_FIND_QUIETLY)
      MESSAGE(STATUS "Found freenect: ${freenect_LIBRARY}")
   ENDIF()
ELSE()
   IF(freenect_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find freenect")
   ENDIF()
ENDIF()

