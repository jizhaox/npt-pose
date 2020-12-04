cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(npt_relpose)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()
set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3" )

find_package(Eigen3 QUIET)

include_directories(
            ${PROJECT_SOURCE_DIR}/include
            ${PROJECT_SOURCE_DIR}/src
            ${EIGEN3_INCLUDE_DIR}
            )

add_library(npt_lib SHARED
            src/construct_coeff_constraint.cpp
            src/construct_coeff_objective.cpp
            src/create2D2DExperiment.cpp
            src/essential_matrix.cpp
            src/npt_pose.cpp
            )
target_link_libraries(npt_lib)

##
find_package(SDPA REQUIRED)
include_directories(${SDPA_INCLUDE_DIRS})
#set(LIBS ${LIBS} ${SDPA_LIBRARIES})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -fPIC -funroll-all-loops")
set(CMAKE_Fortran_FLAGS "-g -O2 -Wall -fPIC -funroll-all-loops")
##

add_executable(demo test/demo.cpp)
target_link_libraries(demo npt_lib ${SDPA_LIBRARIES})

