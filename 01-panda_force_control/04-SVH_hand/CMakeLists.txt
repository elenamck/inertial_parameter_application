# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${INERTIAL_PARAMERTER_APP_BINARY_DIR}/01-panda_force_control/04-SVH_hand)

# add the include directory to the include paths
INCLUDE_DIRECTORIES (
	${INERTIAL_PARAMETER_INCLUDE_DIRS})

# ADD_EXECUTABLE (grasping_tests grasping_tests.cpp)
# ADD_EXECUTABLE (testing_hand testing_hand.cpp)

# and link the library against the executable
# TARGET_LINK_LIBRARIES (grasping_tests ${INERTIAL_PARAMERTER_APP_COMMON_LIBRARIES} ${INERTIAL_PARAMETER_LIBRARIES})
# TARGET_LINK_LIBRARIES (testing_hand ${INERTIAL_PARAMERTER_APP_COMMON_LIBRARIES} ${INERTIAL_PARAMETER_LIBRARIES})

