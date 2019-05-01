# create an executabl
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${INERTIAL_PARAMERTER_APP_BINARY_DIR}/01-panda_force_control/07-simulations)

# add the include directory to the include paths
INCLUDE_DIRECTORIES (
	${INERTIAL_PARAMETER_INCLUDE_DIRS})

#simulation file
ADD_EXECUTABLE(simviz01 simviz.cpp)
#simulation file
ADD_EXECUTABLE (simviz_orig simviz_orig.cpp)
# ADD_EXECUTABLE (simviz_modeled_sensor simviz_modeled_sensor.cpp)
# ADD_EXECUTABLE (simviz_svh simviz_svh.cpp)

ADD_EXECUTABLE (sim_gravity_compensation sim_gravity_compensation.cpp)

# and link the library against the executable

TARGET_LINK_LIBRARIES(simviz01 ${INERTIAL_PARAMERTER_APP_COMMON_LIBRARIES} ${INERTIAL_PARAMETER_LIBRARIES})
# TARGET_LINK_LIBRARIES (simviz_modeled_sensor ${INERTIAL_PARAMERTER_APP_COMMON_LIBRARIES} ${INERTIAL_PARAMETER_LIBRARIES})

TARGET_LINK_LIBRARIES (simviz_orig ${INERTIAL_PARAMERTER_APP_COMMON_LIBRARIES} ${INERTIAL_PARAMETER_LIBRARIES})
# TARGET_LINK_LIBRARIES (simviz_svh ${INERTIAL_PARAMERTER_APP_COMMON_LIBRARIES} ${INERTIAL_PARAMETER_LIBRARIES})
TARGET_LINK_LIBRARIES (sim_gravity_compensation ${INERTIAL_PARAMERTER_APP_COMMON_LIBRARIES} ${INERTIAL_PARAMETER_LIBRARIES})
