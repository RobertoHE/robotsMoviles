cmake_minimum_required (VERSION 2.6) 
project (FMM) 

# Set a default build type for single-configuration
# CMake generators if no build type is set.
IF(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
   SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)

# Select flags.
SET(CMAKE_CXX_FLAGS "-std=c++11")
#SET(CMAKE_CXX_FLAGS "-lwiringPi")
SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g")
SET(CMAKE_CXX_FLAGS_RELEASE "-Ofast -fno-finite-math-only")
SET(CMAKE_CXX_FLAGS_DEBUG  "-Wall -Wno-unused-local-typedefs -g")

# Find OpenCV
#find_package(OpenCV REQUIRED)

# Self-made includes
include_directories (${FMM_SOURCE_DIR}/console)
include_directories (${FMM_SOURCE_DIR}/ndgridmap)
include_directories (${FMM_SOURCE_DIR}/fmm)
include_directories (${FMM_SOURCE_DIR}/fmm/fmdata) 
include_directories (${FMM_SOURCE_DIR}/fm2)
include_directories (${FMM_SOURCE_DIR}/io)
include_directories (${FMM_SOURCE_DIR}/gradientdescent)
#include_directories (${FMM_SOURCE_DIR}/wiringPi)
include_directories (home/pi/repos/UUGear/RaspberryPi/src/)

# Third party internal includes
include_directories (${FMM_SOURCE_DIR}/thirdparty) 


# External dependencies
include_directories (~/boost_1_55_0)

add_executable (fmm main_console_navegacion_arduino.cpp
                    console/console.cpp
                    ndgridmap/cell.cpp
                    fmm/fmdata/fmcell.cpp
		  #  home/pi/repos/UUGear/RaspberryPi/src/UUGear.c

                    ) 

# Linking CImg dependencies.
#target_link_libraries(fmm ${OpenCV_LIBS})
#target_link_libraries(fmm wiringPi)
target_link_libraries (fmm X11 pthread ) 
