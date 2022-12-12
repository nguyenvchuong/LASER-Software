/************************************************************************

CPG with Hopf Oscillators in polar coordinates

************************************************************************/

#include "ros/ros.h"
//#include <gazebo_ros/gazebo_ros_api_plugin.h>
//#include <physics/physics.hh>
#include <stdio.h>
#include <stdlib.h>
#include "laikago_msgs/HighCmd.h"
#include "laikago_msgs/HighState.h"
#include "laikago_msgs/LowCmd.h"
#include "laikago_msgs/LowState.h"
#include "laikago_msgs/MotorCmd.h"
#include "laikago_msgs/MotorState.h"
#include "laikago_msgs/LegCmdArray.h"
#include "laikago_msgs/LegCmd.h"
// #include "laikago_msgs/PybulletState.h"
// #include "laikago_msgs/PybulletFullState.h"
// #include "laikago_msgs/MPCService.h"
// #include "laikago_msgs/RLService.h"
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Empty.h>
#include <gazebo_msgs/SetModelConfiguration.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/SetLinkState.h>
#include <gazebo_msgs/GetJointProperties.h>
#include <gazebo_msgs/GetModelProperties.h>
#include <vector>
#include <string.h>
#include <math.h>
#include <random>
#include <unsupported/Eigen/MatrixFunctions>
#include <nav_msgs/Odometry.h>
#include "../../include/body.h"
#include "../../include/OrientationEstimator.h"
#include "../../include/PositionVelocityEstimator.h"
#include "../../include/ContactEstimator.h"
#include "../../include/LegController.h"
#include "../../include/FSM.h"
#include "../../include/FootSwingTrajectory.h"
//#include "../../include/Dynamics/AlienGo.h"
// #include "../../ConvexMPC/ConvexMPCLocomotion.h"
// #include "../../PybulletInterface/PybulletInterface.h"


using namespace std;
using namespace laikago_model;

double get_random()
{
    static std::default_random_engine e;
    static std::uniform_real_distribution<> dis(0, 1); // rage 0 - 1
    return dis(e);
}

class HopfPolar
{
public:
    HopfPolar() { // : loop_rate(1000) {
        std::cout << "init HopfPolar" << std::endl;
        std::cout << "... HopfPolar object created" << std::endl;
    }
    void init(){
        std::cout << "[HopfPolar] init function" << std::endl;
        SetGait(gait);
        // init CPG amplitude and phase
        // std::cout << get_random() * 0.01 << std::endl; 
        for(int i=0; i<4; i++){
            X(0,i) = get_random() * 0.01;
            X(1,i) = PHI(0,i);
        }
        std::cout << "[HopfPolar] init done" << std::endl;
    }

    /*
    Gaits:
    1 bounding
    2 trotting
    3 walking
    4 pacing;
    5 galloping;
    6 pronking
    */
    void SetGait(int gaitNum )
    {
        if (gaitNum == 1){ // bounding 
            PHI <<   0, 0, 1, 1,
                     0, 0, 1, 1,
                    -1,-1, 0, 0,
                    -1,-1, 0, 0;
            PHI = -M_PI * PHI;
            K_tegotae << 0, 0, 1, 1,
                         0, 0, 1, 1,
                         1, 1, 0, 0,
                         1, 1, 0, 0; 
        }
        else if (gaitNum == 2){ // trotting 
            PHI <<   0, 1, 1, 0,
                    -1, 0, 0,-1,
                    -1, 0, 0,-1,
                     0, 1, 1, 0;
            PHI = M_PI * PHI;
            K_tegotae << 0, 1, 1, 0,
                         1, 0, 0, 1,
                         1, 0, 0, 1,
                         0, 1, 1, 0;
        }
        else if (gaitNum == 3){ // walking 
            PHI <<       0,   M_PI,-M_PI/2, M_PI/2,
                      M_PI,      0, M_PI/2,-M_PI/2,
                    M_PI/2,-M_PI/2,      0,   M_PI,
                   -M_PI/2, M_PI/2,   M_PI,      0;
            PHI = -PHI;
            K_tegotae << 0, 1, 1, 0,
                         1, 0, 0, 1,
                         1, 0, 0, 1,
                         0, 1, 1, 0;
        }
        else if (gaitNum == 4){ // pace 
            PHI <<   0, 1, 0, 1,
                    -1, 0,-1, 0,
                     0, 1, 0, 1,
                    -1, 0,-1, 0;
            PHI = M_PI * PHI;
            K_tegotae << 0, 1, 0, 1,
                         1, 0, 1, 0,
                         0, 1, 0, 1,
                         1, 0, 1, 0;
        }
        else{
            std::cout << "INVALID GAIT NUMBER" << gaitNum << std::endl; 
        }
    }

    void update(Vec4<double>& x_out,Vec4<double>& y_out){
        Integrate();
        Vec4<double> x = Vec4<double>::Zero();
        Vec4<double> y = Vec4<double>::Zero();
        // x = X.row(0) * X.cos().row(1);
        // y = X.row(0) * X.sin().row(1);
        for (int i=0; i<4 ; i++){
            x[i] = X(0,i) * std::cos(X(1,i));
            y[i] = X(0,i) * std::sin(X(1,i));
        }


        Vec4<double> y_temp = Vec4<double>::Zero(); // bookkeepping
        //just IK2 for now
        for(int i=0; i<4; i++){
            double phi = X(1,i);
            if (std::sin(phi) > 0){//in swing
                y_temp[i] = -h_max + ground_clearance * std::sin(phi);
            }
            else{
                y_temp[i] = -h_max + ground_penetration * std::sin(phi);
            }
        }

        // return normal indices
        for (int i=0; i<4; i++){
            x_out[i] = -des_step_len * x[LEG_INDICES[i]];
            y_out[i] = y_temp[LEG_INDICES[i]];
        }

    }

    void Integrate()
    {
        MatrixXd X_copy, X_dot_prev;
        X_copy = X.replicate(1,1);
        X_dot_prev = X_dot.replicate(1,1);
        X_dot.setZero();
        // get foot forces
        auto Fn = lowState.footForce;
        for (int i=0; i<4; i++){
            Fn[i] = lowState.footForce[LEG_INDICES[i]];
        }

        double r, phi, r_alpha, omega;
        r_alpha = 5;

        for(int i=0; i<4; i++){
            omega = 0;
            r   = X(0,i);
            phi = X(1,i);
            // amplitude
            double R_dot = r_alpha * (mu - pow(r,2)) * r;
            // phase
            phi = std::fmod(phi, 2*M_PI);
            if (phi > M_PI){
                omega = omega_stance;
            }
            else{
                omega = omega_swing;
            }
            if (couple){
                //omega += X.row(0).dot(coupling_strength * sin(X.row(1) - phi - PHI.row(i)));
                for (int j=0; j<4; j++){
                    omega += X(0,j) * coupling_strength * std::sin(X(1,j) - phi - PHI(i,j));
                }
                if (tegotae_feedback){
                    // add normal force feedback (Tegotae)
                    omega += -0.05 * Fn[i] * std::cos(phi);
                }
                if (coupled_tegotae_feedback){
                    //coupled normal forces
                    for (int j=0; j<4; j++){
                        omega += 0.1 * K_tegotae(i,j)*Fn[i];
                    }
                }
            }
            X_dot.col(i) << R_dot, omega;
        }

        // integrate 
        X = X + (X_dot_prev + X_dot) * dt / 2;
        for (int i=0; i<4; i++)
            X(1,i) = fmod(X(1,i),2*M_PI);
        // std::cout << "X" << std::endl;
        // std::cout << X << std::endl;
        // std::cout << "X_dot_prev" << std::endl;
        // std::cout << X_dot_prev << std::endl;
        // std::cout << "X_dot" << std::endl;
        // std::cout << X_dot << std::endl;
    }

    void sendAction(){

        // kpCartesian.diagonal() << 2500, 2500, 2500;
        // kdCartesian.diagonal() << 40, 40, 40;
        // kpCartesian.diagonal() << 700, 700, 700;
        // kdCartesian.diagonal() << 12, 12, 12;
        // kpJoint.diagonal() << 150, 70, 70;
        // kdJoint.diagonal() <<  2, 0.5, 0.5;

        kpCartesian.diagonal() << 1200, 1200, 1200;
        kdCartesian.diagonal() << 20, 20, 20;

        kpJoint.diagonal() << 150, 100, 100;
        kdJoint.diagonal() << 2, 1, 1;


        Vec4<double> x_out, z_out;
        update(x_out,z_out);

        for (int i=0; i<4; i++){

            // desired foot position and corresponding joint angles
            Vec3<double> pDes, qDes, tau;
            pDes << x_out[i], sideSign[i] * foot_y, z_out[i];
            computeInverseKinematics(_controlData->_legController->_quadruped, pDes, i, &qDes);
            tau = kpJoint * (qDes - _controlData->_legController->data[i].q) + 
                  kdJoint * ( - _controlData->_legController->data[i].qd);
            // std::cout << "i " << i << " pDes " << pDes << std::endl;
            
            if (ADD_CARTESIAN_PD){
                // get corresponding foot position in leg frame, for the desired joint angles
                Mat3<double> J_qdes; // ignored
                Vec3<double> pDesLocal; // desired foot xyz position
                computeLegJacobianAndPosition(_controlData->_legController->_quadruped, 
                                            qDes, 
                                            &J_qdes, 
                                            &pDesLocal, i);

                // get current ACTUAL measurements 
                computeLegJacobianAndPosition(_controlData->_legController->_quadruped, 
                                            _controlData->_legController->data[i].q, 
                                            &(_controlData->_legController->data[i].J), 
                                            &(_controlData->_legController->data[i].p), i);

                auto J = _controlData->_legController->data[i].J;
                auto ee_pos_legFrame = _controlData->_legController->data[i].p;
                auto foot_linvel = J * _controlData->_legController->data[i].qd;
                auto ffwd = ( kpCartesian * (pDesLocal-ee_pos_legFrame) + kdCartesian *(-foot_linvel));
                //tau += J.transpose() * ( kpCartesian * (pDesLocal-ee_pos_legFrame) + kdCartesian *(-foot_linvel)); 
                tau += J.transpose() * ffwd;
            }

            for (int j = 0; j < 3; j++){
                // _controlData->_legController->commands[i].pDes[j] = act_arr[i*3+j]; 
                // _controlData->_legController->commands[i].vDes[j] = 0; 
                // // _controlData->_legController->commands[i].kpCartesian(j,j) = 500;//1000;//800; //500;
                // // _controlData->_legController->commands[i].kdCartesian(j,j) = 10; // 30; //10; 
                // _controlData->_legController->commands[i].kpCartesian(j,j) = 700;
                // _controlData->_legController->commands[i].kdCartesian(j,j) = 12;
                // _controlData->_legController->commands[i].tau[j] = tau[j]; 
                lowCmd.motorCmd[i*3+j].torque = tau[j];
            }
        }
        sendServoCmd();
    }


    int hasData = 0;
    Vec3<double> v_des;
    double yaw_rate;
    StateEstimate result;
    ControlFSMData* _controlData;

private:

    MatrixXd X = MatrixXd::Zero(2,4);
    MatrixXd X_prev = MatrixXd::Zero(2,4);
    MatrixXd X_dot = MatrixXd::Zero(2,4);
    double dt = 0.001;
    double mu = 1;
    // These are for Cartesian hopf oscillators only (alpha, beta, b)
    // double alpha = 1;
    // double beta = 1;
    // double b = 1;

    // walk
    // double omega_swing = 3.2*2*M_PI;
    // double omega_stance = 0.8*2*M_PI;
    // double omega_swing = 8*2*M_PI;
    // double omega_stance = 2*2*M_PI;

    // trot
    // double omega_swing = 4*2*M_PI;
    // double omega_stance = 2*2*M_PI;
    // double omega_swing = 5*2*M_PI;
    // double omega_stance = 0.5*2*M_PI;

    //bound
    double omega_swing = 5*2*M_PI;
    double omega_stance = 1.3*2*M_PI;
    
    /*
    Gaits:
    1 bounding
    2 trotting
    3 walking
    4 pacing;
    5 galloping;
    6 pronking
    */
    int gait = 1;
    Mat4<double>PHI        = Mat4<double>::Zero();
    Mat4<double>K_tegotae  = Mat4<double>::Zero();
    double coupling_strength = 1;
    bool couple = true;
    bool tegotae_feedback = true;
    bool coupled_tegotae_feedback = false;

    double ground_penetration = 0.01;
    double ground_clearance = 0.07;
    double h_max = 0.25; // robot_height
    double des_step_len = 0.05; 

    // control
    int LEG_INDICES[4] = { 1, 0, 3, 2};
    // double foot_y = 0.0838; //local y pos
    double foot_y = 0.07;
    double sideSign[4] = {-1, 1, -1, 1};
    Mat3<double> kpCartesian, kdCartesian;
    Mat3<double> kpJoint, kdJoint;
    bool ADD_CARTESIAN_PD = true;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "cpg_test");
    ros::Publisher lowCmd_pub, highCmd_pub; 
    ros::NodeHandle n;
    ros::Publisher lowState_pub; // full robot state
    // ros::Publisher legCmds_pub, pybState_pub;  // desired leg commands from stand/MPC
    ros::ServiceClient reset_sim_srv; // reset sim at start of script
    ros::ServiceClient reset_world_srv; // reset world at start of script (difference with above?)
    ros::ServiceClient reset_joint_srv; 
    ros::ServiceClient gazebo_pause_srv, gazebo_unpause_srv;
    ros::ServiceClient set_model_state_serv;
    ros::ServiceClient get_link_state_srv, get_model_properties_srv, set_link_state_srv;
   
    ros::AsyncSpinner spinner(1); // one threads
    ros::Rate loop_rate(1000);
    spinner.start();

    // the following nodes have been initialized by "gazebo.launch"
    lowState_pub = n.advertise<laikago_msgs::LowState>("/laikago_gazebo/lowState/state", 1);
    // legCmds_pub = n.advertise<laikago_msgs::LegCmdArray>("/laikago_gazebo/legCmds", 1);
    // pybState_pub = n.advertise<laikago_msgs::PybulletState>("/laikago_gazebo/ros2pybstate", 1);
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


    // add Gazebo simulation reset, so we don't have to keep relaunching ROS if robot falls over
    // pause/unpause
    gazebo_pause_srv = n.serviceClient<std_srvs::Empty>("/gazebo/pause_physics"); 
    gazebo_unpause_srv = n.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics"); 
    //reset sim/world
    reset_sim_srv = n.serviceClient<std_srvs::Empty>("/gazebo/reset_simulation"); 
    reset_world_srv = n.serviceClient<std_srvs::Empty>("/gazebo/reset_world");
    set_model_state_serv = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    //reset joint positions 
    reset_joint_srv = n.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration"); 
    // reset world/sim (what's the difference? does not reset joints?)
    std_srvs::Empty empty_srv;
    // get link states
    get_link_state_srv = n.serviceClient<gazebo_msgs::GetJointProperties>("/gazebo/get_joint_properties");
    get_model_properties_srv = n.serviceClient<gazebo_msgs::GetModelProperties>("/gazebo/get_model_properties");
    set_link_state_srv = n.serviceClient<gazebo_msgs::SetLinkState>("/gazebo/set_link_state");

    std::cout << "motion_reset" << std::endl;
    motion_reset();
    usleep(1000000);
    // std::cout << "set joints" << std::endl;
    //reset_joint_srv.call(model_config_srv);
    //std::cout << reset_joint_srv.call(model_config_srv) << std::endl;

    gazebo_msgs::SetModelState model_state;
    model_state.request.model_state.model_name = "laikago_gazebo";
    model_state.request.model_state.pose.position.z = 0.5;
    //model_state.request.model_state.pose.position.x = -5;
    std::cout << "set model" << std::endl;
    std::cout << set_model_state_serv.call(model_state) << std::endl;
    usleep(1000000);

    // lowState must be updated with pybullet data
    CurrentState listen_publish_obj;
    usleep(300000); // must wait 300ms, to get first state

    std::cout << "stand up" << std::endl;
    double dt = 0.001;
    Quadruped quad;
    quad.setQuadruped(2); // 1 for Aliengo, 2 for A1
    // initialize new leg controller and state estimate object
    StateEstimate stateEstimate;
    std::cout << "start Leg Controller" << std::endl;
    LegController* legController = new LegController(quad);

    // zero out
    legController->zeroCommand();
    sendServoCmd();

    std::cout << "start state estimate" << std::endl;
    StateEstimatorContainer* stateEstimator = new StateEstimatorContainer(
                       &lowState.imu, legController->data,&stateEstimate);
    stateEstimator->addEstimator<ContactEstimator>();
    stateEstimator->addEstimator<CheaterOrientationEstimator>();
    stateEstimator->addEstimator<CheaterPositionVelocityEstimator>();

    // initialize new Gait scheduler object
    std::cout << "start gait" << std::endl;
    GaitScheduler<double>* gaitScheduler = new GaitScheduler<double>(dt);
    gaitScheduler->initialize();

    // initialize desired command
    std::cout << "start cmd" << std::endl;
    teleCmd* _tele = new teleCmd();
    DesiredStateCommand* desiredStateCommand = new DesiredStateCommand(_tele, &stateEstimate, dt);

    // initialize FSMData
    std::cout << "start controlFSMData" << std::endl;
    ControlFSMData* _controlData = new ControlFSMData;
    _controlData->_quadruped = &quad;
    _controlData->_stateEstimator = stateEstimator;
    _controlData->_gaitScheduler = gaitScheduler;
    _controlData->_legController = legController;
    _controlData->_desiredStateCommand = desiredStateCommand;

    std::cout << "start fsm and init" << std::endl;
    FSM_State* fsm = new FSM_State(_controlData);
    std::cout << "FSM_Started" << std::endl;
    motion_init();
    
    // Start interface obj
    //PybulletStateListener pyb_listen_obj; 
    //pyb_listen_obj._controlData = _controlData;
    HopfPolar cpg;
    cpg.init();
    cpg._controlData = _controlData;
    std::cout << "CPG started" << std::endl;

    // extraneous
    legController->updateData();
    stateEstimator->run();
    int counter = 0;

    std::cout << "motion_reset" << std::endl;
    motion_reset();
    paramInit();
    counter = 0;

    double des_run_vel = 2;
    double run_vel_time = 3;
    std::cout << "start loop" << std::endl;
    while(ros::ok() && counter<20000) {
        // update current robot state
        _controlData->_legController->updateData();
        _controlData->_stateEstimator->run();
        // call cpg
        cpg.sendAction();
        loop_rate.sleep();
        counter++;
    }

    motion_init();
    std::cout << "finished" << std::endl;
    //delete quad;
    //delete &pyb_listen_obj;
    delete desiredStateCommand;
    delete gaitScheduler;
    delete legController;
    delete stateEstimator;
    delete _tele;
    delete _controlData;
    delete fsm;

    return 0;
}
