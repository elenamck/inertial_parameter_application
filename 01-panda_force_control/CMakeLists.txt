# create an executabl
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${DEMO_APPLICATION_BINARY_DIR}/01-panda_force_control)
#template controller
ADD_EXECUTABLE (template_controller template_controller.cpp)
ADD_EXECUTABLE (find_IMU_frame find_IMU_frame.cpp)
ADD_EXECUTABLE (estimation_controller estimation_controller.cpp)
ADD_EXECUTABLE (estimation_controller_h estimation_controller_h.cpp)

#ADD_EXECUTABLE (kf_four_states kf_four_states.cpp)
#ADD_EXECUTABLE (kf_test kf_test.cpp)
ADD_EXECUTABLE (ekf_test ekf_test.cpp)
ADD_EXECUTABLE (kf_tuning kf_tuning.cpp)
#ADD_EXECUTABLE (ekf_test_2 ekf_test_2.cpp)
#controller for force calibration
ADD_EXECUTABLE (calibration calibration.cpp)
#controller initial config then move
#ADD_EXECUTABLE (02controller controller02.cpp)

#controller with least square parameter estimation
ADD_EXECUTABLE (LScontroller controllerLS.cpp)
#controller with recursive least square parameter estimation
ADD_EXECUTABLE (RLScontroller controllerRLS.cpp)

#controller with recursive total least square parameter estimation
#ADD_EXECUTABLE (controller01 controller.cpp)
ADD_EXECUTABLE (simviz01 simviz.cpp)
ADD_EXECUTABLE (trajectory_testing trajectory_testing.cpp)


# and link the library against the executable
TARGET_LINK_LIBRARIES (kf_tuning ${DEMO_APPLICATION_COMMON_LIBRARIES})

TARGET_LINK_LIBRARIES (estimation_controller_h ${DEMO_APPLICATION_COMMON_LIBRARIES})

TARGET_LINK_LIBRARIES (trajectory_testing ${DEMO_APPLICATION_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (estimation_controller ${DEMO_APPLICATION_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (find_IMU_frame ${DEMO_APPLICATION_COMMON_LIBRARIES})
#TARGET_LINK_LIBRARIES (kf_test ${DEMO_APPLICATION_COMMON_LIBRARIES})
#TARGET_LINK_LIBRARIES (kf_four_states ${DEMO_APPLICATION_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (ekf_test ${DEMO_APPLICATION_COMMON_LIBRARIES})
#TARGET_LINK_LIBRARIES (ekf_test_2 ${DEMO_APPLICATION_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (template_controller ${DEMO_APPLICATION_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (calibration ${DEMO_APPLICATION_COMMON_LIBRARIES})
#TARGET_LINK_LIBRARIES (02controller ${DEMO_APPLICATION_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (RLScontroller ${DEMO_APPLICATION_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (LScontroller ${DEMO_APPLICATION_COMMON_LIBRARIES})
#TARGET_LINK_LIBRARIES (controller01 ${DEMO_APPLICATION_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (simviz01 ${DEMO_APPLICATION_COMMON_LIBRARIES})
# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${DEMO_APPLICATION_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
SET(APP_RESOURCE_DIR ${RESOURCE_DIR}/01-panda_force_control)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world.urdf panda_arm.urdf panda_arm_FT_sensor.urdf panda_arm_force_sensor.urdf DESTINATION ${APP_RESOURCE_DIR})