project(Chameleon)
cmake_minimum_required(VERSION 2.8.2)

# Mac ONLY. Define Relative Path on Mac OS
if(NOT DEFINED CMAKE_MACOSX_RPATH)
  set(CMAKE_MACOSX_RPATH 0)
endif()

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules/")

string(TOLOWER ${PROJECT_NAME} LIBRARY_NAME)

set(Chameleon_VERSION_MAJOR 0)
set(Chameleon_VERSION_MINOR 1)
set(PACKAGE_VERSION ${Compass_VERSION_MAJOR}.${Compass_VERSION_MINOR})
set(Chameleon_VERSION ${PACKAGE_VERSION})

include(install_package)
include(SetPlatformVars)
include(ExternalProject)

# This is for external dependencies we may have to download
include_directories(${CMAKE_BINARY_DIR}/include)
link_directories(${CMAKE_BINARY_DIR}/lib)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option (USE_SYSTEM_CERES
  "Use Ceres via find_package" OFF)
SET(N_CORES 8 CACHE STRING "Cores for parallel build")
option(BUILD_APPLICATIONS "Build Applications" ON)



set( CMAKE_CXX_FLAGS "-std=c++11 -Wall ${CMAKE_CXX_FLAGS}" )
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

# Prepare "Catch" library for other executables
set(CATCH_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/catch)
add_library(Catch INTERFACE)
target_include_directories(Catch INTERFACE ${CATCH_INCLUDE_DIR})

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

endif()

#########################################################

############### FMT (formatting library) #################
set(FMT_VERSION 40000)
set(FMT_TAG 398343897f98b88ade80bbebdcbe82a36c65a980)
ExternalProject_Add(fmt_external
  GIT_REPOSITORY https://github.com/fmtlib/fmt.git
  GIT_TAG ${FMT_TAG}
  UPDATE_COMMAND ""
  INSTALL_DIR ${CMAKE_BINARY_DIR}
  CMAKE_ARGS
  -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
  -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
  -DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}
  PREFIX ${CMAKE_CURRENT_BINARY_DIR}/fmt
  BUILD_IN_SOURCE 0
  BUILD_COMMAND make -j${N_CORES}
  INSTALL_COMMAND make install
  )

add_library(fmt UNKNOWN IMPORTED)
set_property(TARGET fmt PROPERTY IMPORTED_LOCATION
             ${CMAKE_BINARY_DIR}/lib/libfmt.a )

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

set( REQUIRED_LIBRARIES
  ${Pangolin_LIBRARIES}
  ${CERES_LIBRARIES}
  ${SceneGraph_LIBRARIES}
  ${Sophus_LIBRARIES}
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  fmt
  pthread
  )

set(LIB_SRCS
  src/visualizer.cpp
  src/path_generator.cpp
  src/odometry_generator.cpp
  src/world_generator.cpp
  src/observation_generator.cpp
  src/data_generator.cpp
  )

set( INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/${LIBRARY_NAME} )
set(LIB_HEADERS
  ${INC_DIR}/data_generator.h
  ${INC_DIR}/data_provider_base.h
  ${INC_DIR}/math_utils.h
  ${INC_DIR}/util.h
  ${INC_DIR}/types.h
  ${INC_DIR}/odometry_generator.h
  ${INC_DIR}/observation_generator.h
  ${INC_DIR}/path_generator.h
  ${INC_DIR}/world_generator.h
  ${INC_DIR}/viewer/visualizer.h
  )

include_directories( ${REQUIRED_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include )


add_library(${LIBRARY_NAME} ${LIB_SRCS} ${LIB_HEADERS})
set_target_properties( ${LIBRARY_NAME} PROPERTIES VERSION "${PACKAGE_VERSION}" SOVERSION "${PACKAGE_VERSION}" )
target_link_libraries( ${LIBRARY_NAME} ${REQUIRED_LIBRARIES} )

# Also build tests
set(TEST_SRCS
  tests/tests-main.cpp
  tests/tests-math.cpp
  tests/tests-odometry_generator.cpp
  tests/tests-observation_generator.cpp
  )

# Make test executable and link to our library
add_executable(tests ${TEST_SRCS})
target_link_libraries(tests Catch ${LIBRARY_NAME})

# Build application
if(BUILD_APPLICATIONS)
    MESSAGE(STATUS "BUILDING APPLICATIONS")
    add_executable(simulator applications/simulator/main.cpp)
    target_link_libraries(simulator ${LIBRARY_NAME})
endif()

