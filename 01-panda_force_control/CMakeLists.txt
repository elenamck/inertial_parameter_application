# create an executabl
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${INERTIAL_PARAMERTER_APP_BINARY_DIR}/01-panda_force_control)

# add the include directory to the include paths
INCLUDE_DIRECTORIES (
	${INERTIAL_PARAMETER_INCLUDE_DIRS})

#template controller
# ADD_EXECUTABLE (template_controller template_controller.cpp)


# and link the library against the executable

# TARGET_LINK_LIBRARIES (template_controller ${INERTIAL_PARAMERTER_APP_COMMON_LIBRARIES} ${INERTIAL_PARAMETER_LIBRARIES})



#add subdirectories

# add_subdirectory(01-kalman_filters)
# add_subdirectory(02-adaptive_filters)
add_subdirectory(03-trajectories)
# add_subdirectory(04-SVH_hand)
# add_subdirectory(05-estimation_controllers)
add_subdirectory(06-urdf_files)
add_subdirectory(07-simulations)
