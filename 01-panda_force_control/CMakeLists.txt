# create an executabl
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${INERTIAL_PARAMERTER_APP_BINARY_DIR}/01-panda_force_control)

# add the include directory to the include paths
INCLUDE_DIRECTORIES (
	${INERTIAL_PARAMETER_INCLUDE_DIRS})

#template controller
ADD_EXECUTABLE (template_controller template_controller.cpp)

#simulation file
ADD_EXECUTABLE (simviz01 simviz.cpp)


# and link the library against the executable

TARGET_LINK_LIBRARIES (simviz01 ${INERTIAL_PARAMERTER_APP_COMMON_LIBRARIES} ${INERTIAL_PARAMETER_LIBRARIES})


TARGET_LINK_LIBRARIES (template_controller ${INERTIAL_PARAMERTER_APP_COMMON_LIBRARIES} ${INERTIAL_PARAMETER_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${INERTIAL_PARAMERTER_APP_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
SET(APP_RESOURCE_DIR ${RESOURCE_DIR}/01-panda_force_control)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world.urdf panda_arm.urdf panda_arm_FT_sensor.urdf panda_arm_force_sensor.urdf DESTINATION ${APP_RESOURCE_DIR})

#add subdirectories

add_subdirectory(01-kalman_filters)
add_subdirectory(02-adaptive_filters)
add_subdirectory(03-trajectories)
add_subdirectory(04-SVH_hand)
add_subdirectory(05-estimation_controllers)

