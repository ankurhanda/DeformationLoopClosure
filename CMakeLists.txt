# -DCMAKE_BUILD_TYPE=Debug
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
project( uniform_sampling_pointcloud )

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# Overide with cmake -DCMAKE_BUILD_TYPE=Debug {dir}
IF( NOT CMAKE_BUILD_TYPE )
   SET( CMAKE_BUILD_TYPE Release )
ENDIF()

# Platform configuration vars
#INCLUDE(SetPlatformVars)

FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(CSparse)
FIND_PACKAGE(CVD REQUIRED)
FIND_PACKAGE(PCL REQUIRED)



SET(
  SOURCES
)


INCLUDE_DIRECTORIES(
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}
        ${CMAKE_CURRENT_BINARY_DIR}
)

INCLUDE_DIRECTORIES(
  ${OpenCV2_INCLUDE_PATH}
  ${EIGEN3_INCLUDE_DIR}
  ${CSPARSE_INCLUDE_DIR}
  ${CVD_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OPENCV_INCLUDE_DIRS}
)

LINK_DIRECTORIES(
  ${OPENCV_LIBRARY_DIRS}
  ${CUDA_SDK_ROOT_DIR}/lib
  ${CUDA_SDK_ROOT_DIR}/common/lib
  /usr/local/lib
  /usr/local/lib64
)

message("PCL LIBRARY = " ${PCL_LIBRARIES})
message("PCL INCLUDE LIBRARY = " ${PCL_INCLUDE_DIRS})

LINK_LIBRARIES(
  ${LIBRARY_NAME}
  ${Eigen3_LIBRARY}
  ${CSPARSE_LIBRARY}
  ${CVD_LIBRARY}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  -lGLEW
  -lglut
  -lGLU
  -lGL
  -lboost_system
  -lpcl_search
  -lboost_filesystem
  -lboost_thread
  -lblas
  -llapack
  -lcholmod
  -lboost_timer
  -lGVars3
   /usr/lib/x86_64-linux-gnu/libopencv_core.so
   /usr/lib/x86_64-linux-gnu/libopencv_features2d.so
)


LINK_LIBRARIES(
  ${PCL_LIBRARIES}
  ${Eigen3_LIBRARY}
  ${CSPARSE_LIBRARY}
  -lpcl_search
  -lcholmod
)

IF(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
ENDIF()

ADD_EXECUTABLE(deformable_lc
               ./src/main_deformable_lc.cpp)
