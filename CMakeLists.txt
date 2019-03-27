cmake_minimum_required (VERSION 2.8.12)
project (INERTIAL_PARAMERTER_APP)
SET(CMAKE_CXX_FLAGS "-std=c++11")
SET(CMAKE_BUILD_TYPE "Debug")
# SET(CMAKE_BUILD_TYPE "Release")


# - eigen3
set(EIGEN3_INCLUDE_DIR $ENV{EIGEN3_INCLUDE_DIR})
if(NOT EIGEN3_INCLUDE_DIR)
	find_package(Eigen3 REQUIRED)
	# TODO: Find automatic way to find for Ubuntu 14.04
	if(NOT EIGEN3_INCLUDE_DIR)
		set(EIGEN3_INCLUDE_DIR /usr/include/eigen3)
	endif()
endif()

# add inertial parameter estimation
SET(PARAMETER_ESTIMATION_SOURCE
 	${PROJECT_SOURCE_DIR}/src/parameter_estimation/RecursiveLeastSquare.cpp
 	${PROJECT_SOURCE_DIR}/src/parameter_estimation/LeastSquare.cpp
	)

# add filters
SET(FILTERS_SOURCE
	${PROJECT_SOURCE_DIR}/src/filters/ExtendedKalmanFilter.cpp
 	${PROJECT_SOURCE_DIR}/src/filters/KalmanFilter.cpp
 	${PROJECT_SOURCE_DIR}/src/filters/QuaternionBasedEKF.cpp
 	${PROJECT_SOURCE_DIR}/src/filters/QuaternionBasedEKF2.cpp
	)
# add header files 
SET(INERTIAL_PARAMETER_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/src)


INCLUDE_DIRECTORIES(${INERTIAL_PARAMETER_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR} 
	)
#create library
ADD_LIBRARY (inertial_parameter_app STATIC
	${PARAMETER_ESTIMATION_SOURCE}
	${FILTERS_SOURCE}
	)

set(INERTIAL_PARAMETER_LIBRARIES
	inertial_parameter_app
	)

# set common dependencies
# - CHAI3D
find_package(CHAI3D REQUIRED)
include_directories(${CHAI3D_INCLUDE_DIRS})
add_definitions(${CHAI3D_DEFINITIONS})
if (CMAKE_SYSTEM_NAME MATCHES Darwin)
  add_definitions(-D_MACOSX -DMACOSX)
elseif (CMAKE_SYSTEM_NAME MATCHES Linux)
  add_definitions(-D_LINUX -DLINUX)
endif () #NOTE: Windows is currently unsupported
link_directories(${CHAI3D_LIBRARY_DIRS})

# - SAI2-SIMULATION. TODO: this should not be needed. something fishy.
find_package(SAI2-SIMULATION REQUIRED)
include_directories(${SAI2-SIMULATION_INCLUDE_DIRS})
add_definitions(${SAI2-SIMULATION_DEFINITIONS})

# - SAI2-COMMON
find_package(SAI2-COMMON REQUIRED)
include_directories(${SAI2-COMMON_INCLUDE_DIRS})
add_definitions(${SAI2-COMMON_DEFINITIONS})

# - SAI2-MODEL
find_package(SAI2-MODEL REQUIRED)
include_directories(${SAI2-MODEL_INCLUDE_DIRS})
add_definitions(${SAI2-MODEL_DEFINITIONS})

# - SAI2-GRAPHICS
find_package(SAI2-GRAPHICS REQUIRED)
include_directories(${SAI2-GRAPHICS_INCLUDE_DIRS})
add_definitions(${SAI2-GRAPHICS_DEFINITIONS})

# - SAI2-URDF
find_package(SAI2-URDF REQUIRED)
include_directories(${SAI2-URDF_INCLUDE_DIRS})
add_definitions(${SAI2-URDF_DEFINITIONS})

# - SAI2-URDF
find_package(SAI2-PRIMITIVES REQUIRED)
include_directories(${SAI2-PRIMITIVES_INCLUDE_DIRS})
add_definitions(${SAI2-PRIMITIVES_DEFINITIONS})

# - glfw3
find_package(glfw3 QUIET)
find_library(GLFW_LIBRARY glfw)

# - hiredis
# - hiredis
find_path(HIREDIS_INCLUDE_DIR NAMES hiredis/hiredis.h)
find_library(HIREDIS_LIBRARY NAMES hiredis)

set(HIREDIS_LIBRARIES ${HIREDIS_LIBRARY})
set(HIREDIS_INCLUDE_DIR ${LHIREDIS_INCLUDE_DIR})
include_directories(${HIREDIS_INCLUDE_DIR})
find_library(HIREDIS_LIBRARY hiredis)
# find_library(HIREDIS_LIBRARY hiredis)



# - jsoncpp
if (CMAKE_SYSTEM_NAME MATCHES Darwin) 
	find_library(JSONCPP_LIBRARY jsoncpp)
elseif (CMAKE_SYSTEM_NAME MATCHES Linux)
	find_package(PkgConfig REQUIRED)
	pkg_check_modules(JSONCPP jsoncpp)
	include_directories(${JSONCPP_INCLUDE_DIRS})
	link_directories(${JSONCPP_LIBRARY_DIRS})
	set(JSONCPP_LIBRARY ${JSONCPP_LIBRARIES})
endif ()


# set common dependencies
set(INERTIAL_PARAMERTER_APP_COMMON_LIBRARIES
	${CHAI3D_LIBARIES}
	${SAI2-COMMON_LIBRARIES}
	${SAI2-MODEL_LIBRARIES}
	${SAI2-GRAPHICS_LIBRARIES}
	${SAI2-URDF_LIBRARIES}
	${SAI2-SIMULATION_LIBRARIES}
	${SAI2-PRIMITIVES_LIBRARIES}
	${HIREDIS_LIBRARY}
	${GLFW_LIBRARY}
	${JSONCPP_LIBRARY}
	${HIREDIS_LIBRARY}
	)

# add apps
set (INERTIAL_PARAMERTER_APP_BINARY_DIR  
	${PROJECT_SOURCE_DIR}/bin)


add_subdirectory(01-panda_force_control)
add_subdirectory(02-utilities)
