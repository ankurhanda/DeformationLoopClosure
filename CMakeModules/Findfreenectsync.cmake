# - Try to find libfreenect_sync
#

#MESSAGE(STATUS ${CMAKE_SOURCE_DIR}/../libfreenect/include)


FIND_PATH(
  freenectsync_INCLUDE_DIR
  NAMES libfreenect_sync.h
  PATHS
    ${CMAKE_SOURCE_DIR}/../libfreenect/wrappers/c_sync
    /usr/include
    /usr/local/include
)

FIND_LIBRARY(
  freenectsync_LIBRARY
  NAMES freenect_sync
  PATHS
    ${CMAKE_SOURCE_DIR}/../libfreenect/build/lib
    /usr/lib
    /usr/local/lib
)

MESSAGE(STATUS ${freenectsync_INCLUDE_DIR})
MESSAGE(STATUS ${freenectsync_LIBRARY})


IF(freenectsync_INCLUDE_DIR AND freenectsync_LIBRARY)
  SET(freenectsync_FOUND TRUE)
ENDIF()

IF(freenectsync_FOUND)
   IF(NOT freenectsync_FIND_QUIETLY)
      MESSAGE(STATUS "Found freenect: ${freenectsync_LIBRARY}")
   ENDIF()
ELSE()
   IF(freenectsync_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find freenectsync")
   ENDIF()
ENDIF()


