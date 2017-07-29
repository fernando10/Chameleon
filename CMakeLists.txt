project(ChangeDetection)
cmake_minimum_required(VERSION 2.8)
# Mac ONLY. Define Relative Path on Mac OS
if(NOT DEFINED CMAKE_MACOSX_RPATH)
  set(CMAKE_MACOSX_RPATH 0)
endif()

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules/")

include(install_package)
include(SetPlatformVars)

set( CMAKE_CXX_FLAGS "-std=c++0x -Wall ${CMAKE_CXX_FLAGS}" )
if(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
endif()

if( NOT CMAKE_BUILD_TYPE )
    set( CMAKE_BUILD_TYPE Release )
endif()

# guard against in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there. You may need to remove CMakeCache.txt. ")
endif()

#####################################################################
## Find required packages
find_package(Pangolin REQUIRED)
find_package(SceneGraph REQUIRED)
find_package(Sophus REQUIRED)
find_package(GLog REQUIRED)
find_package(Ceres REQUIRED)

set( REQUIRED_INCLUDE_DIRS
    ${Pangolin_INCLUDE_DIRS}
    ${SceneGraph_INCLUDE_DIRS}
    ${Sophus_INCLUDE_DIRS}
    ${GLOG_INCLUDE_DIRS}
   )

include_directories( ${REQUIRED_INCLUDE_DIRS} )
include_directories( "external" )
include_directories( "include" )


set( REQUIRED_LIBRARIES
    ${Pangolin_LIBRARIES}
    ${CERES_LIBRARIES}
    ${SceneGraph_LIBRARIES}
    ${Sophus_LIBRARIES}
    ${GLOG_LIBRARIES}
    )

set(SRCS
   src/main.cpp
   src/visualizer.cpp
   )

 set(TEST_SRCS
   tests/tests-main.cpp
   tests/tests-math.cpp
   )

add_executable(change_detection ${SRCS})
add_executable(tests ${TEST_SRCS} external/catch.hpp)


target_link_libraries( change_detection
  ${REQUIRED_LIBRARIES}
  )

target_link_libraries( tests
  ${REQUIRED_LIBRARIES}
  )

