/************************************************************************

This is the class and executable for the HRI motion library.

Pass in strings to interface with MPC and balance controller, as well as a few scripted motions.  


Implemented motions:
Stand
Sit
Laydown
Move right/left (MPC)


TODO:
Lean right/left (balance controller)
Wave (port from python)


************************************************************************/

#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include "laikago_msgs/HighCmd.h"
#include "laikago_msgs/HighState.h"
#include "laikago_msgs/LowCmd.h"
#include "laikago_msgs/LowState.h"
#include "laikago_msgs/MotorCmd.h"
#include "laikago_msgs/MotorState.h"
// #include "laikago_msgs/LegCmdArray.h"
// #include "laikago_msgs/LegCmd.h"
// #include "laikago_msgs/PybulletState.h"
// #include "laikago_msgs/PybulletFullState.h"
// #include "laikago_msgs/MPCService.h"
// #include "laikago_msgs/RLFootPos.h"
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Empty.h>
#include <gazebo_msgs/SetModelConfiguration.h>
#include <gazebo_msgs/SetModelState.h>
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
//#include "../../include/Dynamics/AlienGo.h"
#include "../../ConvexMPC/ConvexMPCLocomotion.h"
#include "../../BalanceController/BalanceController.hpp"
// #include "../../PybulletInterface/PybulletInterface.h"

using namespace std;
using namespace laikago_model;

class HRIFSM
{
public:
    HRIFSM() : Cmpc(0.001, 30), v_des(0, 0, 0), loop_rate(1000) {
        //gazebo specific, for reset
        reset_sim_srv = nm.serviceClient<std_srvs::Empty>("/gazebo/reset_simulation"); 
        reset_world_srv = nm.serviceClient<std_srvs::Empty>("/gazebo/reset_world");
        set_model_state_serv = nm.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    }
    
    /*
     * Set FSM state with string, and reset firstRun flag for balance controller to avoid drift
     */
    void SetFSMState(const string _fsm_state){  
        if (fsm_state.compare(_fsm_state) == 0){
            firstRunBalanceController = false;
        }     
        else{
            firstRunBalanceController = true;
        }
        fsm_state = _fsm_state;
    }

    /*
    Execute actions based on current state:

    Implemented motions:
    Stand
    Sit
    Laydown
    Move right/left (MPC)
    Lean right/left (balance controller)
    Wave

    TODO:
    follow incoming commands to pitch/yaw to keep human in frame 

    */
    void Execute(){
        if (fsm_state.compare("STAND") == 0 || fsm_state.compare("DEFAULT") == 0){
            std::cout << "Stand: run balance controller" << std::endl;
            RunBalanceController();
        }
        if (fsm_state.compare("SIT") == 0){
            std::cout << "Sit: run balance controller, verify pitch" << std::endl;
            std::vector<double> _rpy{ 0, -0.7, 0 };
            std::vector<double> _p_des{ 0, 0, 0.15 };
            RunBalanceController(_rpy, _p_des);
        }
        if (fsm_state.compare("LAYDOWN") == 0){
            std::cout << "Laydown: run balance controller" << std::endl;
            std::vector<double> _rpy{ 0, 0, 0 };
            std::vector<double> _p_des{ 0, 0, 0.15 };
            RunBalanceController(_rpy, _p_des);
        }
        if (fsm_state.compare("STEP") == 0){
            std::cout << "Step (in place): run mpc" << std::endl;
            RunMPC();
        }
        if (fsm_state.compare("MOVELEFT") == 0){
            std::cout << "Move left: run mpc" << std::endl;
            MoveLeft();
        }
        if (fsm_state.compare("MOVERIGHT") == 0){
            std::cout << "Move right: run mpc" << std::endl;
            MoveRight();
        }
        if (fsm_state.compare("WAVE") == 0){
            std::cout << "Wave: run scripted motion" << std::endl;
            Wave();
        }
    }
    /*
     * Move left for x seconds
     */
    void MoveLeft(){
        int counter = 0;
        std::vector<double> _mpc_vel{ 0, 0.1, 0 };
        double _mpc_yaw_rate = 0;
        while(ros::ok() && counter < lateral_move_duration) {
            RunMPC(_mpc_vel, _mpc_yaw_rate);
            counter += 1;
            loop_rate.sleep();
        }
    }
    /*
     * Move right for x seconds
     */
    void MoveRight(){
        int counter = 0;
        std::vector<double> _mpc_vel{ 0, -0.5, 0 };
        double _mpc_yaw_rate = 0;
        while(ros::ok() && counter < lateral_move_duration) {
            RunMPC(_mpc_vel, _mpc_yaw_rate);
            counter += 1;
            loop_rate.sleep();
        }
    }

    /*
     * Wave
     */
    void Wave(){
        // set joint gains
        kpJoint << 100, 0, 0,
                    0, 100, 0,
                    0, 0, 100; //100
        kdJoint<<   2, 0, 0,
                    0, 2, 0,
                    0, 0, 2;
        int traj_len = 7;
        double x_off = 0.25;
        double z_off = 0.2; 
        double x_traj[traj_len] = {0, 0, x_off, x_off-0.02, x_off+0.02, x_off, 0, 0};
        double y_traj[traj_len] = {0, 0,     0,     -0.005,      0.005,     0, 0, 0};
        double z_traj[traj_len] = {0, 0, z_off, z_off-0.02, z_off+0.02, z_off, 0, 0};
        // original
        // double t_arr[traj_len] =  {0, 0.5, 0.65, 0.8, 0.9, 1, 1.5};
        double t_arr[traj_len] =  {0, 0.5, 1.0, 1.4, 1.8, 2.2, 3.0, 3.5};
        int t_idx = 0;
        double wave_duration = t_arr[traj_len-1];

        // get current foot positions 
        LegControllerData feet_pos[4];
        _controlData->_legController->updateData();
        _controlData->_stateEstimator->run();
        for (int i = 0; i <4; i++){
            // updates Jacobian (not really needed here?)
            computeLegJacobianAndPosition(_controlData->_legController->_quadruped, 
                                        _controlData->_legController->data[i].q, 
                                        &(_controlData->_legController->data[i].J), 
                                        &(feet_pos[i].p), i);
            if (i==0 or i==2)
                feet_pos[i].p << 0, -0.0838, -0.3;
            else
                feet_pos[i].p << 0, 0.0838, -0.3;
        }

        int counter = 0;
        double t = 0;
        double dt = 0.001;
        double alpha = 0; // blend
        Vec3<double> contact_xyz_offset(0.06,-0.05,0);
        while(ros::ok() && counter < (int)wave_duration/dt) {
            _controlData->_legController->updateData();
            _controlData->_stateEstimator->run();

            t = counter * dt;
            if (t > t_arr[t_idx+1])
                t_idx++;
            if (t_idx >= traj_len)
                break;

            for(int i=0; i<4; i++){
                Vec3<double> xyz(0,0,0);
                // FR special case
                if (i==0){
                    alpha = t - t_arr[t_idx];
                    Vec3<double> FR_xyz(x_traj[t_idx], y_traj[t_idx], z_traj[t_idx]);
                    Vec3<double> FR_xyz1(x_traj[t_idx+1], y_traj[t_idx+1], z_traj[t_idx+1]);
                    xyz = FR_xyz + alpha*(FR_xyz1 - FR_xyz) / (t_arr[t_idx+1] - t_arr[t_idx]);
                }
                else{
                    xyz = min(2*t,1.0) * contact_xyz_offset;
                }
                xyz += feet_pos[i].p;

                // get desired angles
                Vec3<double> qDes(0,0,0);
                Vec3<double> qdDes(0,0,0);
                computeInverseKinematics(_controlData->_legController->_quadruped, xyz, i, &qDes);

                Vec3<double> tau_ff = kpJoint * ( qDes  - _controlData->_legController->data[i].q  )
                                    + kdJoint * ( qdDes - _controlData->_legController->data[i].qd  );

                // update commands 
                for (int j = 0; j<3; j++){
                    lowCmd.motorCmd[i*3+j].torque = tau_ff[j]; 
                }
            }
            sendServoCmd();

            counter += 1;
            loop_rate.sleep();
        }

        // shift body back

        // run balance controller for a second
        counter = 0;
        while(ros::ok() && counter < 1000) {
            RunBalanceController();
            counter += 1;
            loop_rate.sleep();

        }
    }

    /*
     *  Gazebo helper: reset whole simulation, standup, etc.
     */ 
    void ResetEnv(){
        resetting_flag = true;
        v_des << 0, 0, 0;
        motion_init();
        motion_reset();
        std::cout << "MOVE TO CROUCH" << std::endl;
        //motion_reset_ground();

        // reset to "crouch" init instead
        // double pos_laydown[12] = {  0.0, 0.5, -1., -0.0, 0.5, -1., 
        //                             0.0, 0.5, -1., -0.0, 0.5, -1.};
        // moveAllPosition(pos_laydown, 5000);

        //motion_init();
        //paramInit();
        //std::cout << "reset world" << std::endl;
        //reset_world_srv.call(empty_srv); // just puts base at origin
        //usleep(1000000);
        std::cout << "reset sim" << std::endl;
        reset_sim_srv.call(empty_srv); // also sets joints to 0
        //usleep(1000000);
        std::cout << "motion init" << std::endl;
        motion_init();
        //motion_reset_ground();
        //paramInit();
        sendServoCmd();
        for (int i=0; i < 2000; i++){
            _controlData->_legController->updateData();
            _controlData->_stateEstimator->run();
            sendServoCmd();
            loop_rate.sleep();
        }
        //motion_init();


        // add gazebo reset
        // gazebo_msgs::SetModelState model_state;
        // model_state.request.model_state.model_name = "laikago_gazebo";
        // model_state.request.model_state.pose.position.z = 0.4;
        // std::cout << "set model" << std::endl;
        // std::cout << set_model_state_serv.call(model_state) << std::endl;
        // usleep(10000);

        //usleep(1000000);
        _controlData->_legController->updateData();
        _controlData->_stateEstimator->run();
        // stand up
        //StandUpImpedance();
        RunBalanceController();
        //usleep(10000);
        resetting_flag = false;
    }

    /****************************************************************************************************************************
    ********************************************************* MPC ***************************************************************
    *****************************************************************************************************************************/
    /*
    * Default MPC, step in place.
    */
    void RunMPC(){

        _controlData->_legController->updateData();
        _controlData->_stateEstimator->run();

        v_des << 0, 0, 0;
        yaw_rate = 0;
        //Cmpc.setGaitNum(2);
        if(fabs(_controlData->_stateEstimator->getResult().rpy[2]) > 0.01){
            if(_controlData->_stateEstimator->getResult().rpy[2] < 0)
                yaw_rate = 0.5;
            else yaw_rate = -0.5;
        }
        _solveMPC();

    }
    /*
    * Set velocity and yaw rate for MPC.
    */
    void RunMPC(std::vector<double>& _mpc_vel, double _mpc_yaw_rate){

        _controlData->_legController->updateData();
        _controlData->_stateEstimator->run();

        v_des << _mpc_vel[0], _mpc_vel[1], _mpc_vel[2];
        yaw_rate = _mpc_yaw_rate;

        _solveMPC();

    }

    /*
     * Actual MPC interface and solve
     */ 
    void _solveMPC(){
        // this updates leg q,qd,tau from motorState 
        _controlData->_legController->updateData();
        // set desired body vel (v_des) and yaw_rate
        _controlData->_desiredStateCommand->setStateCommands(v_des, yaw_rate);
        // set gait (trot)
        Cmpc.setGaitNum(gaitNum);
        // run MPC
        Cmpc.run(*_controlData);
        // apply commands
        _controlData->_legController->updateCommand();

    }

    /**************************************************************************************************************************
    ************************************************* Balance Controller ******************************************************
    ***************************************************************************************************************************/
    /*
     * Default
     */ 
    void RunBalanceController(){
        // avoid drift in position over time
        if (firstRunBalanceController){
            _controlData->_legController->updateData();
            _controlData->_stateEstimator->run();
            // position & rpy desired
            for(int i = 0; i < 3; i++){
                p_des[i] = _controlData->_stateEstimator->getResult().position(i); 
                v_des_b[i] = 0;
                omegaDes[i] = 0;
                rpy[i] = 0;
            }
            rpy[2] = _controlData->_stateEstimator->getResult().rpy(2);
            p_des[2] = 0.3;
            firstRunBalanceController = false;
        }

        _solveBalanceController();
    }

    /*
     * Call balance controller with a desired rpy and p_des
     */ 
    void RunBalanceController(std::vector<double>& _rpy, std::vector<double>& _p_des){
        if (firstRunBalanceController){
            _controlData->_legController->updateData();
            _controlData->_stateEstimator->run();
            // position & rpy desired
            for(int i = 0; i < 3; i++){
                p_des[i] = _controlData->_stateEstimator->getResult().position(i); 
                v_des_b[i] = 0;
                omegaDes[i] = 0;
                rpy[i] = _rpy[i];
            }
            firstRunBalanceController = false;
        }

        // update commands 
        for(int i = 0; i < 3; i++){
            rpy[i] = _rpy[i];
        }
        p_des[2] = _p_des[2];

        _solveBalanceController();
    }

    /*
     * Actual balance controller interface and solve
     */ 
    void _solveBalanceController(){
        //counter = 0;
        //while(ros::ok() && counter < 1000) {
            _controlData->_legController->updateData();
            _controlData->_stateEstimator->run();

            for (int i = 0; i < 4; i++) {
                se_xfb[i] = _controlData->_stateEstimator->getResult().orientation(i);
            }

            for (int i = 0; i < 3; i++) {
                //rpy[i] = 0;
                //rpy[i] = _data->_stateEstimator->getResult().rpy(i);
                //p_des[i] = _data->_stateEstimator->getResult().position(i);
                p_act[i] = _controlData->_stateEstimator->getResult().position(i);
                v_act[i] = _controlData->_stateEstimator->getResult().vBody(i);
                // v_des[i] = 0;
                //v_des[2] = 0.005;

                se_xfb[4 + i] = _controlData->_stateEstimator->getResult().position(i);
                se_xfb[7 + i] = _controlData->_stateEstimator->getResult().omegaBody(i);
                se_xfb[10 + i] = _controlData->_stateEstimator->getResult().vBody(i);

                // Set the translational and orientation gains
                kpCOM[i] = 30;   //_data->controlParameters->kpCOM(i);
                kdCOM[i] = 10;     //_data->controlParameters->kdCOM(i);
                kpBase[i] = 80;     //_data->controlParameters->kpBase(i);
                kdBase[i] = 50; //  _data->controlParameters->kdBase(i);
            }

            kpCOM[2] = 50;
            //kdCOM[2] = 10;

            //Vec3<double> pFeetVec;
            Vec3<double> pFeetVecCOM;

            // Get the foot locations relative to COM
            for (int leg = 0; leg < 4; leg++) {
                // computeLegJacobianAndPosition(&_data->_quadruped, _data->_legController->data[leg].q,
                //                           (Mat3<double>*)nullptr, &pFeetVec, leg);
                //pFeetVecCOM = _data->_stateEstimator->getResult().rBody.transpose() *
                          //(_data->_quadruped->getHipLocation(leg) + pFeetVec);

                pFeetVecCOM =  _controlData->_stateEstimator->getResult().rBody.transpose() *
                        (_controlData->_quadruped->getHipLocation(leg) + _controlData->_legController->data[leg].p);


                pFeet[leg * 3] = pFeetVecCOM[0];
                pFeet[leg * 3 + 1] = pFeetVecCOM[1];
                pFeet[leg * 3 + 2] = pFeetVecCOM[2];
                //std::cout << "pFeet" << leg << std::endl;
            }

            // balance controller options
            balanceController.set_alpha_control(0.01);
            balanceController.set_friction(0.6);
            balanceController.set_mass(_controlData->_quadruped->mass);
            balanceController.set_wrench_weights(COM_weights_stance, Base_weights_stance);
            balanceController.set_PDgains(kpCOM, kdCOM, kpBase, kdBase);
            balanceController.set_desiredTrajectoryData(rpy, p_des, omegaDes, v_des_b);
            balanceController.SetContactData(contactStateScheduled, minForces, maxForces);
            balanceController.updateProblemData(se_xfb, pFeet, p_des, p_act, v_des_b, v_act,
                                          O_err, _controlData->_stateEstimator->getResult().rpy(2));
            // balanceController.print_QPData();
            double fOpt[12];
            balanceController.solveQP_nonThreaded(fOpt);

            for (int leg = 0; leg < 4; leg++) {
                footFeedForwardForces.col(leg) << fOpt[leg * 3], fOpt[leg * 3 + 1],
                fOpt[leg * 3 + 2]; // force in world frame, need to convert to body frame

                _controlData->_legController->commands[leg].feedforwardForce = _controlData->_stateEstimator->getResult().rBody.transpose() *
                                                                        footFeedForwardForces.col(leg);
                //_data->_stateEstimator->getResult().rBody.transpose() * footFeedForwardForces.col(leg); 
               // QP << _data->_legController->commands[leg].feedforwardForce[2] << " ";
            }
            _controlData->_legController->updateCommand();
            // loop_rate.sleep();
            // counter += 1;
        
        //}
        //counter = 0;
    }

    int hasData = 0;
    Vec3<double> v_des;
    double yaw_rate;
    StateEstimate result;
    ControlFSMData* _controlData;
    ConvexMPCLocomotion Cmpc;
    BalanceController balanceController;

    double rlFootPos[8];
    bool resetting_flag = true;
    int counter = 0;


private:
    ros::NodeHandle nm;
    ros::Rate loop_rate;
    int gaitNum = 2;

    // gazebo, for reset
    ros::ServiceClient reset_sim_srv; // reset sim at start of script
    ros::ServiceClient reset_world_srv; // reset world at start of script (difference with above?)
    ros::ServiceClient set_model_state_serv;
    std_srvs::Empty empty_srv;
    std_msgs::Empty empty_msg;

    // ===================== FSM variables
    string fsm_state = "DEFAULT";
    int lateral_move_duration = 2000;
    bool firstRunBalanceController = true;

    Mat3<double> kpCartesian, kdCartesian;
    Mat3<double> kpJoint, kdJoint;

    // ===================== Balance Controller
    Mat34<double>footFeedForwardForces = Mat34<double>::Zero();
    Mat34<double>footstepLocations = Mat34<double>::Zero();
    double minForce = 5;
    double maxForce = 500;
    double contactStateScheduled[4] = {1, 1, 1, 1};
    double minForces[4] = {minForce, minForce, minForce, minForce};
    double maxForces[4] = {maxForce, maxForce, maxForce, maxForce};

    //double COM_weights_stance[3] = {50, 50, 50};
    //double Base_weights_stance[3] = {500, 150, 30};// for QP locomotion
    double COM_weights_stance[3] = {5, 5, 10};
    double Base_weights_stance[3] = {10, 10, 20};
    double pFeet[12], p_act[3], v_act[3], O_err[3], rpy[3], p_des[3], v_des_b[3], omegaDes[3]; //v_des[3]
    double se_xfb[13];
    double kpCOM[3], kdCOM[3], kpBase[3], kdBase[3];
    double b_control[6];
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "hri_fsm");
    ros::Publisher lowCmd_pub, highCmd_pub; 
    ros::NodeHandle n;
    ros::Publisher lowState_pub; // full robot state
    ros::Publisher legCmds_pub, pybState_pub;  // desired leg commands from stand/MPC
   
    ros::AsyncSpinner spinner(1); // one threads
    ros::Rate loop_rate(1000);
    spinner.start();

    // must create this object to update lowState with model state
    CurrentState listen_publish_obj;

    usleep(300000); // must wait 300ms, to get first state
   
    //laikago_msgs::LegCmdArray legCmdArray;
    //laikago_msgs::PybulletState ros2pybstate;

    // the following nodes have been initialized by "gazebo.launch"
    lowState_pub = n.advertise<laikago_msgs::LowState>("/laikago_gazebo/lowState/state", 1);
    // legCmds_pub = n.advertise<laikago_msgs::LegCmdArray>("/laikago_gazebo/legCmds", 1);
    //pybState_pub = n.advertise<laikago_msgs::PybulletState>("/laikago_gazebo/ros2pybstate", 1);
    // needed to update commands
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

    
    std::cout << "stand up" << std::endl;
    double dt = 0.001;
    Quadruped quad ;
    quad.setQuadruped(2); // 1 for Aliengo, 2 for A1
    // initialize new leg controller and state estimate object
    StateEstimate stateEstimate;
    std::cout << "start Leg Controller" << std::endl;
    LegController* legController = new LegController(quad);

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

    std::cout << "init" << std::endl;
    // FSM_State* fsm = new FSM_State(_controlData);
    // std::cout << "FSM_Started" << std::endl;
    motion_init();

    legController->updateData();
    stateEstimator->run();


    // Start FSM
    HRIFSM hri_fsm_obj;
    hri_fsm_obj._controlData = _controlData;

    // reset environment,  standup
    hri_fsm_obj.ResetEnv();
    int counter = 0;

    /*
    Available motions:
    Stand
    Sit
    Laydown
    Move right/left (MPC)
    Lean right/left (balance controller)
    Wave

    TODO:
    follow incoming commands to pitch/yaw to keep human in frame 

    */
    int motion_test_len = 7;
    string motion_tests[motion_test_len] ={   "STAND",
                                "WAVE",
                                "STAND",
                                "SIT",
                                "STAND",
                                "STEP",
                                "MOVELEFT",
                                "STAND",
                            };
    int motion_test_idx = 0;
    hri_fsm_obj.SetFSMState(motion_tests[motion_test_idx]);
    counter = 1;

    /*======================== Run FSM ====================*/
    // Note some scripted actions will cause the loop to not execute at loop_rate, but this is ok for tests
    
    while(ros::ok() ){ 

        if (counter % 2000 == 0 || motion_tests[motion_test_idx].compare("WAVE") == 0){
            motion_test_idx++;
            if (motion_test_idx == motion_test_len)
                motion_test_idx = 0;
            std::cout << "Next action: " << motion_tests[motion_test_idx] << std::endl;
        }

        hri_fsm_obj.SetFSMState(motion_tests[motion_test_idx]);

        hri_fsm_obj.Execute();

        loop_rate.sleep();
        counter++; 

    }
    
 
    //std::cout << "finished" << std::endl;
    //delete quad;
    //delete &pyb_listen_obj;
    delete desiredStateCommand;
    delete gaitScheduler;
    delete legController;
    delete stateEstimator;
    delete _tele;
    delete _controlData;
    // delete fsm;

    return 0;
}
