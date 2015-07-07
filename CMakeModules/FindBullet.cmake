# - Try to find libBullet
#
#  Bullet_FOUND - system has libBullet
#  Bullet_INCLUDE_DIRS - the libBullet include directories
#  Bullet_LIBRARIES - link these to use libBullet

FIND_PATH(
  Bullet_BASE
  NAMES btBulletDynamicsCommon.h
  PATHS ${CMAKE_SOURCE_DIR}/../bullet/src
  /usr/include/bullet
  /usr/local/include/bullet
)

FIND_LIBRARY(
  BulletDynamics_LIBRARY
  NAMES BulletDynamics
  PATHS ${Bullet_BASE}/.libs
  /usr/lib
  /usr/local/lib
)

FIND_LIBRARY(
  BulletCollision_LIBRARY
  NAMES BulletCollision
  PATHS ${Bullet_BASE}/.libs
  /usr/lib
  /usr/local/lib
)

FIND_LIBRARY(
  LinearMath_LIBRARY
  NAMES LinearMath
  PATHS ${Bullet_BASE}/.libs
  /usr/lib
  /usr/local/lib
)

IF(Bullet_BASE AND BulletDynamics_LIBRARY AND BulletCollision_LIBRARY AND LinearMath_LIBRARY)
   SET(Bullet_LIBRARIES ${BulletDynamics_LIBRARY} ${BulletCollision_LIBRARY} ${LinearMath_LIBRARY})
   SET(Bullet_FOUND TRUE)
   SET(Bullet_INCLUDE_DIRS ${Bullet_BASE})
ENDIF(Bullet_BASE AND BulletDynamics_LIBRARY AND BulletCollision_LIBRARY AND LinearMath_LIBRARY)

IF(Bullet_FOUND)
   IF(NOT Bullet_FIND_QUIETLY)
      MESSAGE(STATUS "Found Bullet: ${Bullet_LIBRARIES}")
   ENDIF(NOT Bullet_FIND_QUIETLY)
ELSE(Bullet_FOUND)
   IF(Bullet_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find Bullet")
   ENDIF(Bullet_FIND_REQUIRED)
ENDIF(Bullet_FOUND)
