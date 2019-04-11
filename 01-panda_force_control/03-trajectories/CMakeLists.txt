# create an executabl
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${INERTIAL_PARAMERTER_APP_BINARY_DIR}/01-panda_force_control/03-trajectories)

# add the include directory to the include paths
INCLUDE_DIRECTORIES (
	${INERTIAL_PARAMETER_INCLUDE_DIRS})


ADD_EXECUTABLE (trajectory_testing trajectory_testing.cpp)
ADD_EXECUTABLE (sinusoidal sinusoidal_trajectories.cpp)
# ADD_EXECUTABLE (tune_sinusoidal tune_sinusoidal_trajectories.cpp)


TARGET_LINK_LIBRARIES (sinusoidal ${INERTIAL_PARAMERTER_APP_COMMON_LIBRARIES} ${INERTIAL_PARAMETER_LIBRARIES})
TARGET_LINK_LIBRARIES (trajectory_testing ${INERTIAL_PARAMERTER_APP_COMMON_LIBRARIES} ${INERTIAL_PARAMETER_LIBRARIES})
# TARGET_LINK_LIBRARIES (tune_sinusoidal ${INERTIAL_PARAMERTER_APP_COMMON_LIBRARIES} ${INERTIAL_PARAMETER_LIBRARIES})

