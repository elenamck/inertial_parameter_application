# create an executabl
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${INERTIAL_PARAMERTER_APP_BINARY_DIR}/01-panda_force_control/05-estimation_controllers)

# add the include directory to the include paths
INCLUDE_DIRECTORIES (
	${INERTIAL_PARAMETER_INCLUDE_DIRS})

ADD_EXECUTABLE (estimation_controller_debugging estimation_controller_debugging.cpp)
ADD_EXECUTABLE (estimation_controller estimation_controller.cpp)
ADD_EXECUTABLE (estimation_test estimation_test.cpp)
ADD_EXECUTABLE (inertia_estimation inertia_estimation.cpp)



# and link the library against the executable

TARGET_LINK_LIBRARIES (estimation_test ${INERTIAL_PARAMERTER_APP_COMMON_LIBRARIES} ${INERTIAL_PARAMETER_LIBRARIES})

TARGET_LINK_LIBRARIES (estimation_controller_debugging ${INERTIAL_PARAMERTER_APP_COMMON_LIBRARIES} ${INERTIAL_PARAMETER_LIBRARIES})

TARGET_LINK_LIBRARIES (estimation_controller ${INERTIAL_PARAMERTER_APP_COMMON_LIBRARIES} ${INERTIAL_PARAMETER_LIBRARIES})
TARGET_LINK_LIBRARIES (inertia_estimation ${INERTIAL_PARAMERTER_APP_COMMON_LIBRARIES} ${INERTIAL_PARAMETER_LIBRARIES})