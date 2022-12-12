cmake_minimum_required(VERSION 3.5)
project(laikago_gazebo)
#set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD 17)
include_directories(${CMAKE_BINARY_DIR})
set(CMAKE_C_FLAGS "-O3  -ggdb  -march=native -std=gnu99 -I.")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-lboost_system -fext-numeric-literals -O3") 


find_package(catkin REQUIRED COMPONENTS
    controller_manager
    genmsg
    joint_state_controller
    robot_state_publisher
    roscpp
    gazebo_ros
    gazebo_msgs
    std_msgs
    tf
    geometry_msgs
    laikago_msgs  
)
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED)
#find_package(qpoases)
catkin_package(
    CATKIN_DEPENDS 
    laikago_msgs 
)

include_directories(
    # include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    $ENV{HOME}/libtensorflow/include
)

link_directories(${GAZEBO_LIBRARY_DIRS})


#  $ENV{HOME}/libtensorflow/include/tensorflow
#include_directories($ENV{HOME}/libtensorflow/include)
# Add tensorflow
find_library(TENSORFLOW_LIB tensorflow HINT $ENV{HOME}/libtensorflow/lib)

#target_include_directories(${PROJECT_NAME} PRIVATE $ENV{HOME}/libtensorflow/include)
#target_link_libraries (${PROJECT_NAME} "${TENSORFLOW_LIB}")


add_subdirectory(qpOASES)
add_subdirectory(JCQP)


# Declare a C++ library
add_library(${PROJECT_NAME}
    include/body.h
    include/StateEstimatorContainer.h
    src/DesiredCommand.cpp
    BalanceController/BalanceController.cpp
    BalanceController/BalanceControllerWrapper.cpp
    ConvexMPC/RobotState.cpp
    ConvexMPC/SolverMPC.cpp
    ConvexMPC/convexMPC_interface.cpp
    ConvexMPC/ConvexMPCLocomotion.cpp
    LeastSquareSolution/LeastSquareSolution.cpp
    src/QPLocomotion.cpp
    src/body.cpp 
    src/CurrentState.cpp
    src/LegController.cpp
    src/teleCmd.cpp
    src/Gait.cpp
    src/FSM.cpp
    include/TransitionData.h
    include/ControlFSMData.h
    include/Dynamics/FloatingBaseModel.h
    src/FootSwingTrajectory.cpp
    src/PositionVelocityEstimator.cpp
    src/OrientationEstimator.cpp
    # cppflow/src/Tensor.cpp
    # cppflow/src/Model.cpp
    cppflow/include/cppflow/tensor.h
    cppflow/include/cppflow/model.h
    PybulletInterface/PybulletInterface.cpp
    PybulletInterface/Jumping.cpp
)
file(GLOB_RECURSE sources "src/*.cpp")

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} ${EXTRA_LIBS}
)


add_library(LaikagoFootContactPlugin SHARED plugin/foot_contact_plugin.cc)

target_link_libraries(LaikagoFootContactPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-O3  -ggdb -no-pie")
target_link_libraries(${PROJECT_NAME} qpOASES JCQP "${TENSORFLOW_LIB}")

add_library(LaikagoDrawForcePlugin SHARED plugin/draw_force_plugin.cc)
target_link_libraries(LaikagoDrawForcePlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(laikago_external_force src/exe/external_force.cpp)
target_link_libraries(laikago_external_force ${catkin_LIBRARIES})

add_executable(laikago_servo src/exe/servo.cpp)
target_link_libraries(laikago_servo ${PROJECT_NAME} ${catkin_LIBRARIES} qpOASES)

add_executable(laikago_pyb_stand src/exe/pyb_stand.cpp)
target_link_libraries(laikago_pyb_stand ${PROJECT_NAME} ${catkin_LIBRARIES} qpOASES)

add_executable(laikago_pyb_send_state src/exe/pyb_send_state.cpp)
target_link_libraries(laikago_pyb_send_state ${PROJECT_NAME} ${catkin_LIBRARIES} qpOASES)

add_executable(laikago_pyb_mpc src/exe/pyb_mpc_interface_notes.cpp)
target_link_libraries(laikago_pyb_mpc ${PROJECT_NAME} ${catkin_LIBRARIES} qpOASES)

add_executable(pyb_mpc src/exe/pyb_mpc.cpp)
target_link_libraries(pyb_mpc ${PROJECT_NAME} ${catkin_LIBRARIES} qpOASES)

add_executable(run_rl src/exe/run_rl_actions.cpp)
target_link_libraries(run_rl ${PROJECT_NAME} ${catkin_LIBRARIES} qpOASES)

add_executable(run src/exe/run.cpp)
target_link_libraries(run ${PROJECT_NAME} ${catkin_LIBRARIES} qpOASES)

add_executable(rl_gym_helper src/exe/rl_gym_helper.cpp)
target_link_libraries(rl_gym_helper ${PROJECT_NAME} ${catkin_LIBRARIES} qpOASES)

add_executable(hri_fsm src/exe/hri_fsm.cpp)
target_link_libraries(hri_fsm ${PROJECT_NAME} ${catkin_LIBRARIES} qpOASES)

add_executable(jump_rl src/exe/jump_rl.cpp)
target_link_libraries(jump_rl ${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} qpOASES)

# add_executable(test_tf src/exe/test_tf.cpp)
# target_link_libraries(test_tf ${PROJECT_NAME} ${catkin_LIBRARIES} qpOASES)

add_executable(leg_test src/exe/leg_test.cpp)
target_link_libraries(leg_test ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(ori_test src/exe/ori_test.cpp)
target_link_libraries(ori_test ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(a1_jump src/exe/jump.cpp)
target_link_libraries(a1_jump ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(cube_spawner src/exe/cube_spawner.cpp)
target_link_libraries(cube_spawner ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(run_cpg src/exe/run_cpg.cpp)
target_link_libraries(run_cpg ${PROJECT_NAME} ${catkin_LIBRARIES})