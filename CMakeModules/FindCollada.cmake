# - Try to find libCollada
#
#  Collada_FOUND - system has libCollada
#  Collada_INCLUDE_DIRS - the libCollada include directories
#  Collada_LIBRARIES - link these to use libCollada

SET(CSFX "")

FIND_PATH(
  Collada_BASE
  NAMES dom/include/1.4/dom/domAccessor.h
  PATHS ${CMAKE_SOURCE_DIR}/../collada-dom /usr/include/collada-dom /usr/local/include/collada-dom
)

FIND_LIBRARY(
  ColladaDom_LIBRARY
  NAMES collada14dom${CSFX}
  PATHS ${Collada_BASE}/dom/build/linux-1.4${CSFX} /usr/lib /usr/local/lib
)

FIND_LIBRARY(
  CMinizip_LIBRARY
  NAMES minizip${CSFX}
  PATHS ${Collada_BASE}/dom/build/linux-1.4${CSFX} /usr/lib /usr/local/lib
)

FIND_LIBRARY(
  ColladaCRT_LIBRARY
  NAMES collada14rt${CSFX}
  PATHS ${Collada_BASE}/rt/build/linux-1.4${CSFX} /usr/lib /usr/local/lib
)

FIND_LIBRARY(
  ColladaFX_LIBRARY
  NAMES collada14fx${CSFX}
  PATHS ${Collada_BASE}/fx/build/linux-1.4${CSFX} /usr/lib /usr/local/lib
)

IF(Collada_BASE AND ColladaDom_LIBRARY AND CMinizip_LIBRARY AND ColladaCRT_LIBRARY AND ColladaFX_LIBRARY)
   SET(Collada_FOUND TRUE)
   SET(Collada_INCLUDE_DIRS
      ${Collada_BASE}/dom/include/
      ${Collada_BASE}/dom/include/1.4
      ${Collada_BASE}/rt/include
      ${Collada_BASE}/fx/include
   )
   SET(Collada_LIBRARIES
      ${ColladaDom_LIBRARY}
      ${CMinizip_LIBRARY}
      ${ColladaCRT_LIBRARY}
      ${ColladaFX_LIBRARY}
   )
   ADD_DEFINITIONS(-DCFX_PLATFORM_INCLUDE=<cfxLinux.h>)
   ADD_DEFINITIONS(-DCRT_PLATFORM_INCLUDE=<CrtLinux.h>)
ENDIF()

IF(Collada_FOUND)
   IF(NOT Collada_FIND_QUIETLY)
      MESSAGE(STATUS "Found Collada: ${Collada_LIBRARIES}")
   ENDIF(NOT Collada_FIND_QUIETLY)
ELSE(Collada_FOUND)
   IF(Collada_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find Collada")
   ENDIF(Collada_FIND_REQUIRED)
ENDIF(Collada_FOUND)
