/*!
 * @file leg_test.cpp
 * @brief test leg controller classes and objects in ROS
 * 
 * Test legcontroller, state estimate and etc.
 */

#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include "laikago_msgs/HighCmd.h"
#include "laikago_msgs/HighState.h"
#include "laikago_msgs/LowCmd.h"
#include "laikago_msgs/LowState.h"
#include "laikago_msgs/MotorCmd.h"
#include "laikago_msgs/MotorState.h"
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <string.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include "../../include/body.h"
#include "../../include/OrientationEstimator.h"
#include "../../include/PositionVelocityEstimator.h"
#include "../../include/ContactEstimator.h"
#include "../../include/LegController.h"
#include "../../include/FSM.h"
#include "../../include/FootSwingTrajectory.h"
#include "fstream"

using namespace std;
using namespace laikago_model;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "aliengo_gazebo_leg");
    ros::Publisher lowState_pub;
    ros::NodeHandle n;

    ofstream myfile;
    myfile.open("footForce.txt");

    CurrentState listen_publish_obj;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    usleep(300000);
    ros::Rate r(1000);
    lowState_pub = n.advertise<laikago_msgs::LowState>("/laikago_gazebo/lowState/state", 1);
    servo_pub[0] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/FR_hip_controller/command", 1);
    servo_pub[1] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/FR_thigh_controller/command", 1);
    servo_pub[2] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/FR_calf_controller/command", 1);
    servo_pub[3] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/FL_hip_controller/command", 1);
    servo_pub[4] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/FL_thigh_controller/command", 1);
    servo_pub[5] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/FL_calf_controller/command", 1);
    servo_pub[6] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/RR_hip_controller/command", 1);
    servo_pub[7] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/RR_thigh_controller/command", 1);
    servo_pub[8] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/RR_calf_controller/command", 1);
    servo_pub[9] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/RL_hip_controller/command", 1);
    servo_pub[10] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/RL_thigh_controller/command", 1);
    servo_pub[11] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/RL_calf_controller/command", 1);

    std::cout << "init Joint Parameters" << std::endl;
    motion_init();
    //test();
    
    double dt = 0.001;
    Quadruped quad;
    std::cout << "init Leg Controller" << std::endl;
    LegController* legController = new LegController(quad);

    std::cout << "init state estimator" << std::endl;
    StateEstimate stateEstimate;
    StateEstimatorContainer* stateEstimator = new StateEstimatorContainer(
                       &lowState.imu, legController->data,&stateEstimate);
    stateEstimator->addEstimator<ContactEstimator>();
    stateEstimator->addEstimator<CheaterOrientationEstimator>();
    stateEstimator->addEstimator<CheaterPositionVelocityEstimator>();
    ros::spinOnce();
   // std::cout << "simple torque command" << std::endl;
   // legController->updateData();
    int iter = 0;

    //double f = -43; 
    double h = 0.4;
    Mat34<double> init_foot_pos;
    legController->updateData();
     for(int i = 0; i < 4; i++){
       init_foot_pos.col(i)= legController->data[i].p;
    }

    // std::cout << init_foot_pos << std::endl;
    double side_sign[4] = {-1, 1, -1, 1};
    Vec3<double> ff_force_world(0, 0, -50.65);

    while(ros::ok()){
          
    legController->updateData();  
    stateEstimator->run();

     double progress = 0.2*iter * 0.001;
     if(progress > 1.)  {progress = 1.;}
      
      for(int i = 0; i < 4; i++){
        legController->commands[i].kpCartesian = Vec3<double>(300,300,900).asDiagonal();
        legController->commands[i].kdCartesian = Vec3<double>(30, 30, 30).asDiagonal();

        legController->commands[i].pDes << 0.0, side_sign[i] * 0.083, 0;
        //legController->commands[i].pDes[2] = -0.4;
        
        legController->commands[i].pDes[2] = -h*progress + (1. - progress) * init_foot_pos(2, i);
        //legController->commands[i].feedforwardForce = stateEstimator->getResult().rBody.transpose() * ff_force_world;
      }
      
    //legController->commands[0].feedforwardForce = Vec3<double>(0, 0, f);
    //legController->commands[1].feedforwardForce = Vec3<double>(0, 0, f);
    //legController->commands[2].feedforwardForce = Vec3<double>(0, 0, f);
    //legController->commands[3].feedforwardForce = Vec3<double>(0, 0, f);
    legController->updateCommand(); // ros::spinOnce() executed in every sendServoCmd()
    iter++;
   // for (int i =0; i < 4;  i++){
     //   std::cout << "foot pos: " << i << std::endl;
     //   std::cout << legController->data[i].p << std::endl;
   //}
    
    r.sleep();
    //ros::spinOnce(); 
   
    //f+= 0.05;
    //std::cout << "thigh pos" << std::endl;
    
    }
    myfile.close();
    delete legController;
    delete stateEstimator;
    
    return 0;
}

