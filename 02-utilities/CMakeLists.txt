# create an executabl
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${INERTIAL_PARAMERTER_APP_BINARY_DIR}/02-utilities)

# add the include directory to the include paths
INCLUDE_DIRECTORIES (
	${INERTIAL_PARAMETER_INCLUDE_DIRS})
find_library(HIREDIS_LIBRARY hiredis)


#controller for checking if the transformation between the IMU frame and the force sensor frame is correct
ADD_EXECUTABLE (find_IMU_frame find_IMU_frame.cpp)

#controller for force calibration
ADD_EXECUTABLE (calibration calibration.cpp)

#simulation file
ADD_EXECUTABLE (simviz00 simviz.cpp)




# and link the library against the executable
TARGET_LINK_LIBRARIES (find_IMU_frame ${INERTIAL_PARAMERTER_APP_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (calibration ${INERTIAL_PARAMERTER_APP_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (simviz00 ${INERTIAL_PARAMERTER_APP_COMMON_LIBRARIES})


# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${INERTIAL_PARAMERTER_APP_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
SET(APP_RESOURCE_DIR ${RESOURCE_DIR}/02-utilities)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world.urdf panda_arm.urdf panda_arm_FT_sensor.urdf panda_arm_force_sensor.urdf DESTINATION ${APP_RESOURCE_DIR})