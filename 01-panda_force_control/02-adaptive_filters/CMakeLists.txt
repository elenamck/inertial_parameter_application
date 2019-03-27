# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${INERTIAL_PARAMERTER_APP_BINARY_DIR}/01-panda_force_control/02-adaptive_filters)

# add the include directory to the include paths
INCLUDE_DIRECTORIES (
	${INERTIAL_PARAMETER_INCLUDE_DIRS})

#controller with least square parameter estimation
ADD_EXECUTABLE (LScontroller controllerLS.cpp)
#controller with recursive least square parameter estimation
ADD_EXECUTABLE (RLScontroller controllerRLS.cpp)

# and link the library against the executable

TARGET_LINK_LIBRARIES (RLScontroller ${INERTIAL_PARAMERTER_APP_COMMON_LIBRARIES} ${INERTIAL_PARAMETER_LIBRARIES})
TARGET_LINK_LIBRARIES (LScontroller ${INERTIAL_PARAMERTER_APP_COMMON_LIBRARIES} ${INERTIAL_PARAMETER_LIBRARIES})