project(ChangeDetection)
cmake_minimum_required(VERSION 2.8)
# Mac ONLY. Define Relative Path on Mac OS
if(NOT DEFINED CMAKE_MACOSX_RPATH)
  set(CMAKE_MACOSX_RPATH 0)
endif()

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules/")

include(install_package)
include(SetPlatformVars)
include(ExternalProject)

# This is for external dependencies we may have to download
include_directories(${CMAKE_BINARY_DIR}/include)
link_directories(${CMAKE_BINARY_DIR}/lib)

option (USE_SYSTEM_CERES
  "Use Ceres via find_package" OFF)
SET(N_CORES 8 CACHE STRING "Cores for parallel build")


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

find_package(GLog REQUIRED)
include_directories(BEFORE ${GLOG_INCLUDE_DIRS})

################# CERES #########################
find_package(SuiteSparse REQUIRED QUIET)
find_package(CXSparse QUIET)
if (${USE_SYSTEM_CERES})
  find_package( Ceres REQUIRED )
  include_directories(BEFORE ${CERES_INCLUDE_DIRS})
  message(STATUS "Using system Ceres.")
else ()
  set(CERES_VERSION 1.13.0)
  set(CERES_TAG 72790f1a25a8fa71d5ea0ff32126290c4c00029d)
  ExternalProject_Add(ceres_external
    GIT_REPOSITORY https://ceres-solver.googlesource.com/ceres-solver
    GIT_TAG ${CERES_TAG}
    UPDATE_COMMAND ""
    INSTALL_DIR ${CMAKE_BINARY_DIR}
    CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
    -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
    -DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}
    -DBUILD_EXAMPLES:BOOL=OFF
    -DBUILD_TESTING:BOOL=OFF
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/ceres
    BUILD_IN_SOURCE 0
    BUILD_COMMAND make -j${N_CORES}
    INSTALL_COMMAND make install
    )

  add_library(ceres UNKNOWN IMPORTED)
  set(CERES_LIB_PREFIX "lib")
  if (CMAKE_SYSTEM_NAME MATCHES "Linux" AND
      NOT DEFINED LIB_SUFFIX AND
      NOT CMAKE_CROSSCOMPILING AND
      CMAKE_SIZEOF_VOID_P EQUAL "8" AND
      NOT EXISTS "/etc/debian_version" AND
      NOT EXISTS "/etc/arch-release")
    set(CERES_LIB_PREFIX "lib64")
  endif ()

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
  set_property(TARGET ceres PROPERTY IMPORTED_LOCATION
             ${CMAKE_BINARY_DIR}/${CERES_LIB_PREFIX}/libceres-debug.a )
else ()
  set_property(TARGET ceres PROPERTY IMPORTED_LOCATION
             ${CMAKE_BINARY_DIR}/${CERES_LIB_PREFIX}/libceres.a )
endif ()

find_package(OpenMP QUIET)
if (OPENMP_FOUND)
  message(STATUS "OpenMP found")
  add_definitions(-DUSE_OPENMP)
  set(CERES_LIBRARIES ceres gomp ${SUITESPARSE_LIBRARIES} ${CXSPARSE_LIBRARIES} ${Eigen_LIBS} ${GLOG_LIBRARY})
else ()
  message(STATUS "OpenMP NOT found")
  set(CERES_LIBRARIES ceres ${SUITESPARSE_LIBRARIES} ${CXSPARSE_LIBRARIES}  ${Eigen_LIBS} ${GLOG_LIBRARY})
endif ()
add_dependencies(${CERES_LIBRARIES} ceres_external)

# install it once built
install(FILES ${CMAKE_BINARY_DIR}/${CERES_LIB_PREFIX}/libceres.a
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
install(DIRECTORY ${CMAKE_BINARY_DIR}/include/ceres/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/ceres/)
install(DIRECTORY ${CMAKE_BINARY_DIR}/share/Ceres/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/Ceres/)
endif ()

#########################################################

#####################################################################
## Find required packages
find_package(Pangolin REQUIRED)
find_package(SceneGraph REQUIRED)
find_package(Sophus REQUIRED)
find_package(GFlags REQUIRED)

set( REQUIRED_INCLUDE_DIRS
  ${Pangolin_INCLUDE_DIRS}
  ${SceneGraph_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${GFlag_INCLUDE_DIRS}
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
  ${GFLAGS_LIBRARIES}
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
