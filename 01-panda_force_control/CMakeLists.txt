# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${DEMO_APPLICATION_BINARY_DIR}/01-panda_force_control)

#controller initial config then move
ADD_EXECUTABLE (02controller controller02.cpp)

#controller with least square parameter estimation
ADD_EXECUTABLE (LScontroller controllerLS.cpp)
#controller with recursive least square parameter estimation
ADD_EXECUTABLE (RLScontroller controllerRLS.cpp)

#controller with recursive total least square parameter estimation
ADD_EXECUTABLE (controller01 controller.cpp)
ADD_EXECUTABLE (simviz01 simviz.cpp)

# and link the library against the executable
TARGET_LINK_LIBRARIES (02controller ${DEMO_APPLICATION_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (RLScontroller ${DEMO_APPLICATION_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (LScontroller ${DEMO_APPLICATION_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (controller01 ${DEMO_APPLICATION_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (simviz01 ${DEMO_APPLICATION_COMMON_LIBRARIES})
# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${DEMO_APPLICATION_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
SET(APP_RESOURCE_DIR ${RESOURCE_DIR}/01-panda_force_control)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world.urdf panda_arm.urdf panda_arm_FT_sensor.urdf panda_arm_force_sensor.urdf DESTINATION ${APP_RESOURCE_DIR})