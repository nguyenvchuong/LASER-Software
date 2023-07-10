/************************************************************************

Test jumping trajectories with RL

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
#include "laikago_msgs/PybulletState.h"
#include "laikago_msgs/PybulletFullState.h"
#include "laikago_msgs/MPCService.h"
#include "laikago_msgs/RLService.h"
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
//#include "../../PybulletInterface/PybulletInterface.h"
#include "../../PybulletInterface/Jumping.h"

#include "gazebo/common/KeyFrame.hh"
#include "gazebo/common/Animation.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/CommonTypes.hh"
#include "gazebo/common/URI.hh"

#include "gazebo/physics/Gripper.hh"
#include "gazebo/physics/Joint.hh"
#include "gazebo/physics/JointController.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/Contact.hh"

#include "gazebo/transport/Node.hh"

#include "gazebo/util/IntrospectionManager.hh"
#include "gazebo/util/OpenAL.hh"


#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/SpawnModelRequest.h>
#include <gazebo_msgs/SpawnModelResponse.h>
#include <sstream>

using namespace gazebo;
using namespace physics;


using namespace std;
using namespace laikago_model;
#define SPAWN_OBJECT_TOPIC "gazebo/spawn_sdf_model"
ros::ServiceClient spawn_object;

void spawnCube(const std::string& name, const std::string& frame_id,
    float x, float y, float z, float qx, float qy, float qz, float qw,
    float width, float height, float depth, float mass);
void spawnPrimitive(const std::string& name, const bool doCube,
    const std::string& frame_id,
    float x, float y, float z, float qx, float qy, float qz, float qw,
    float widthOrRadius, float height, float depth, float _mass);
// GazeboCubeSpawner::GazeboCubeSpawner(NodeHandle &n) : nh(n){
//     spawn_object = n.serviceClient<gazebo_msgs::SpawnModel>(SPAWN_OBJECT_TOPIC);
// }

void spawnCube(const std::string& name, const std::string& frame_id,
    float x, float y, float z, float qx, float qy, float qz, float qw,
    float width, float height, float depth, float mass)
{
    spawnPrimitive(name, true, frame_id, x, y, z, qx, qy, qz, qw, width, height, depth, mass);
}

void spawnPrimitive(const std::string& name, const bool doCube,
    const std::string& frame_id,
    float x, float y, float z, float qx, float qy, float qz, float qw,
    float widthOrRadius, float height, float depth, float _mass)
{

    geometry_msgs::Pose pose;
    pose.position.x=x;
    pose.position.y=y;
    pose.position.z=z;
    pose.orientation.x=qx;
    pose.orientation.y=qy;
    pose.orientation.z=qz;
    pose.orientation.w=qw;

    gazebo_msgs::SpawnModel spawn;
    spawn.request.model_name=name;

    // just so the variable names are shorter..
    float w=widthOrRadius;
    float h=height;
    float d=depth;

    std::stringstream _s;
    if (doCube)
    {
          _s<<"<box>\
            <size>"<<w<<" "<<h<<" "<<d<<"</size>\
          </box>";
    }else{

          _s<<"<cylinder>\
                <length>"<<h<<"</length>\
                <radius>"<<w<<"</radius>\
            </cylinder>";
    }
    std::string geometryString = _s.str();


    float mass=_mass;
    float mass12=mass/12.0;

    // double mu1=500; //500 for PR2 finger tip. In first experiment had it on 1000000
    // double mu2=mu1;
    double mu1=0.8; //500 for PR2 finger tip. In first experiment had it on 1000000
    double mu2=mu1;
    double kp=10000000; //10000000 for PR2 finger tip
    double kd=1; //100 for rubber? 1 fir OR2 finger tip

    bool do_surface=true;
    bool do_inertia=true;

    std::stringstream s;\
    s<<"<?xml version='1.0'?>\
    <sdf version='1.4'>\
    <model name='"<<name<<"'>\
        <static>false</static>\
        <link name='link'>";

    // inertia according to https://en.wikipedia.org/wiki/List_of_moments_of_inertia
    if (do_inertia) 
    {
        double xx, yy, zz;
        if (doCube)
        {
            xx=mass12*(h*h+d*d);
            yy=mass12*(w*w+d*d);
            zz=mass12*(w*w+h*h);
        }
        else
        {
            xx=mass12*(3*w*w + h*h);
            yy=mass12*(3*w*w + h*h);
            zz=0.5*mass*w*w;
        }
        s<<"<inertial>\
        <mass>"<<mass<<"</mass>\
        <inertia>\
          <ixx>"<<xx<<"</ixx>\
          <ixy>0.0</ixy>\
          <ixz>0.0</ixz>\
          <iyy>"<<yy<<"</iyy>\
          <iyz>0.0</iyz>\
          <izz>"<<zz<<"</izz>\
        </inertia>\
          </inertial>";
    }    
    s<<"<collision name='collision'>\
        <geometry>"<<geometryString;
    s<<"</geometry>";
    if (do_surface)
        s<<"<surface>\
            <friction>\
              <ode>\
            <mu>"<<mu1<<"</mu>\
            <mu2>"<<mu2<<"</mu2>\
            <fdir1>0.000000 0.000000 0.000000</fdir1>\
            <slip1>0.000000</slip1>\
            <slip2>0.000000</slip2>\
              </ode>\
            </friction>\
            <bounce>\
              <restitution_coefficient>0.000000</restitution_coefficient>\
              <threshold>100000.000000</threshold>\
            </bounce>\
            <contact>\
              <ode>\
            <soft_cfm>0.000000</soft_cfm>\
            <soft_erp>0.200000</soft_erp>\
            <kp>"<<kp<<"</kp>\
            <kd>"<<kd<<"</kd>\
            <max_vel>100.000000</max_vel>\
            <min_depth>0.001000</min_depth>\
              </ode>\
            </contact>\
        </surface>";
      s<<"</collision>\
          <visual name='visual'>";
      s<<"<geometry>"<<geometryString;
      s<<"</geometry>\
        <material>\
            <script>\
                <uri>file://media/materials/scripts/gazebo.material</uri> \
                <name>Gazebo/Gray</name>\
            </script>\
        </material>\
          </visual>\
        </link>\
      </model>\
    </sdf>";
    
    spawn.request.model_xml=s.str();
    spawn.request.robot_namespace="cube_spawner";
    spawn.request.initial_pose=pose;
    spawn.request.reference_frame=frame_id;

    //ROS_INFO("Resulting model: \n %s",s.str().c_str());

    //ROS_INFO("Waiting for service");
    spawn_object.waitForExistence();
    //ROS_INFO("Calling service");

    //std::cout<<spawn.request<<std::endl;

    if (!spawn_object.call(spawn)) {
        ROS_ERROR("Failed to call service %s",SPAWN_OBJECT_TOPIC);
    }
    ROS_INFO("Result: %s, code %u",spawn.response.status_message.c_str(), spawn.response.success);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_mpc_interface");
    ros::Publisher lowCmd_pub, highCmd_pub; 
    ros::NodeHandle n;
    // ros::ServiceClient 
    spawn_object = n.serviceClient<gazebo_msgs::SpawnModel>(SPAWN_OBJECT_TOPIC);;

    ros::Publisher lowState_pub; // full robot state
    ros::Publisher legCmds_pub, pybState_pub;  // desired leg commands from stand/MPC
    ros::ServiceClient reset_sim_srv; // reset sim at start of script
    ros::ServiceClient reset_world_srv; // reset world at start of script (difference with above?)
    ros::ServiceClient reset_joint_srv; 
    ros::ServiceClient gazebo_pause_srv, gazebo_unpause_srv;
    ros::ServiceClient set_model_state_serv;
    ros::ServiceClient get_link_state_srv, get_model_properties_srv, set_link_state_srv;
   
    ros::AsyncSpinner spinner(1); // one threads
    ros::Rate loop_rate(1000);
    spinner.start();

    //reset sim/world
    set_model_state_serv = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    // the following nodes have been initialized by "gazebo.launch"
    lowState_pub = n.advertise<laikago_msgs::LowState>("/laikago_gazebo/lowState/state", 1);
    legCmds_pub = n.advertise<laikago_msgs::LegCmdArray>("/laikago_gazebo/legCmds", 1);
    pybState_pub = n.advertise<laikago_msgs::PybulletState>("/laikago_gazebo/ros2pybstate", 1);
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


    // add simulation reset, so we don't have to keep relaunching ROS if robot falls over
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


    gazebo_msgs::SetLinkState lnk;
    
    motion_reset();

    // gazebo_msgs::SetModelState model_state;
    // model_state.request.model_state.model_name = "laikago_gazebo";
    // model_state.request.model_state.pose.position.z = 0.5;
    // std::cout << "set model" << std::endl;
    // std::cout << set_model_state_serv.call(model_state) << std::endl;
    // usleep(1000000);

    // Chuong

    // string jump_trajectories[1] ={"src/LASER-Software/laikago_ros/jumping/data9_forward/jumpingFull_A1_1ms_h00_d60_full_state.csv"};
    string jump_trajectories[1] ={"src/LASER-Software/laikago_ros/jumping/data14_forward/jumpingFull_A1_1ms_h20_d60_full_state.csv"};
    // string jump_trajectories[1] ={"src/LASER-Software/laikago_ros/jumping/data11_forward/jumpingFull_A1_1ms_h20_d60_full_state.csv"};
    double jump_zx_coordinates[2] = {0.2, 0.6};
    // string jump_trajectories[1] ={"src/LASER-Software/laikago_ros/jumping/data9_backflip/backflipFull_A1_1ms_h0_d-50_full_state.csv"};
    // double jump_zx_coordinates[2] = {0.0, -0.5};

    int jump_traj_idx = 0;
    
    bool spawn_cubes = true;
    bool front_cube = true;
    bool rear_cube = false;
    float rear_cube_height = 0.05;
    float front_cube_height = 0.05; // 0.05
    // float rear_cube_height = 0.08;
    // float front_cube_height = 0.01;

    if(spawn_cubes){
        ROS_INFO("Spawn cube..");

        // dimensions
        float dim_x = 0.8;
        float dim_y = 1;
        float dim_z = jump_zx_coordinates[jump_traj_idx*2];
        // location 
        float x=jump_zx_coordinates[jump_traj_idx*2+1]+dim_x/2-0.2;
        float y=0;
        float z=dim_z;
        std::string name="landing_box";
        std::string frame_id="world";

        float dim=0.05;   
        float mass=0.05;

        dim = 0.5;
        mass = 100;

        //spawner.spawnCube(name,frame_id,x,y,z,0,0,0,1,dim,dim,dim,mass);
        spawnCube(name,frame_id,x,y,z,0,0,0,1,dim_x,dim_y,dim_z,mass);
    }
    // add cube under front / rear
    if (front_cube){
        // dimensions
        float dim_x = 0.12;
        float dim_y = 1;
        float dim_z = front_cube_height; // change this 
        // location 
        float x=0.15;
        float y=0;
        float z=dim_z;
        std::string name="front_box";
        std::string frame_id="world";
        float mass=0.05;
        mass = 100;

        spawnCube(name,frame_id,x,y,z,0,0,0,1,dim_x,dim_y,dim_z,mass);
    }
    // add cube under front / rear
    if (rear_cube){
        // dimensions
        float dim_x = 0.3;
        float dim_y = 1;
        float dim_z = rear_cube_height; // change this 
        // location 
        float x=-0.15;
        float y=0;
        float z=dim_z;
        std::string name="rear_box";
        std::string frame_id="world";
        float mass=0.05;
        mass = 100;

        spawnCube(name,frame_id,x,y,z,0,0,0,1,dim_x,dim_y,dim_z,mass);
    }

    // reset quadruped 
    // start at the right position 


    // lowState must be updated with pybullet data
    CurrentState listen_publish_obj;

    usleep(300000); // must wait 300ms, to get first state

    laikago_msgs::PybulletFullState ros2pybstate;

    
    std::cout << "stand up" << std::endl;
    double dt = 0.001;
    Quadruped quad ;
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
    // cheat data (ground truth)
    stateEstimator->addEstimator<ContactEstimator>();
    stateEstimator->addEstimator<CheaterOrientationEstimator>();
    stateEstimator->addEstimator<CheaterPositionVelocityEstimator>();
    // using sensors (regarding front foot)
    // stateEstimator->addEstimator<ContactEstimator>();
    // stateEstimator->addEstimator<VectorNavOrientationEstimator>();
    // stateEstimator->addEstimator<TunedKFPositionVelocityEstimator>();


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
    //motion_init();

    // ConvexMPCLocomotion Cmpc(0.001, 30);
    
    // Start interface obj
    //PybulletStateListener pyb_listen_obj; 
    //pyb_listen_obj._controlData = _controlData;
    JumpingObj jumping_obj;
    jumping_obj._controlData = _controlData;

    // load trajectory 
    jumping_obj.readTraj(jump_trajectories[jump_traj_idx]);

    // extraneous
    legController->updateData();
    stateEstimator->run();

    int counter = 0;
    bool runQP = false;
    int flight_idx = 800;
    int posing_time = 200;

    /*

    CHECK ACTION SPACE RANGE - changed to 0.1 offsets 

    */

    //Jump_Init();
    // jumping_obj.moveToTrajInitPos();
    // motion_reset();

    // start at the right position 
    gazebo_msgs::SetModelState model_state;
    model_state.request.model_state.model_name = "laikago_gazebo";
    model_state.request.model_state.pose.position.z = 0.5;
    std::cout << "set model" << std::endl;
    std::cout << set_model_state_serv.call(model_state) << std::endl;
    jumping_obj.moveToTrajInitPos();

    // record full state 
    // write out trajectory
    int full_state_len = 72+4+12+4+12; // 72+ footforce+action+feeforwardForce
    std::ofstream full_traj("jump_full_state.txt");
    std::vector<double> full_state(full_state_len);
    // jumping_obj.getFullState(full_state);
    // for(int j=0; j<full_state_len; j++)
    //     full_traj << full_state[j] << " ";
    // full_traj << "\n";



    usleep(1000000);
    stateEstimator->run();
    auto& seResult = stateEstimator->getResult();
    for(int i = 0; i < 3; i++) cout<<"initial position:" << seResult.position[i] << std::endl;
    cout<<"roll:"<<seResult.rpy[0] <<"pitch: " << seResult.rpy[1] <<"yaw:" << seResult.rpy[2]<< std::endl;

    usleep(4000000);
    // Get init_final for observation
    std::vector<double> init_final(14);
    init_final[0]=seResult.position[0]; init_final[1]=seResult.position[1]; init_final[2]=seResult.position[2];
    init_final[3]=seResult.rpy[0]; init_final[4]=seResult.rpy[1]; init_final[5]=seResult.rpy[2];
    init_final[6]=jump_zx_coordinates[1]; init_final[7]=0; init_final[8]=jump_zx_coordinates[0]+0.15;
    init_final[9]=0; init_final[10]=0; init_final[11]=0;
    init_final[12]=0; init_final[13]=0;
    
    if (front_cube){
        init_final[12]=front_cube_height; // front box height
    }
    if (rear_cube){
        init_final[13]=rear_cube_height; // rear box height
    }



    std::cout << "Start jumping" << std::endl;

    // cout<< "period:" <<jumping_obj.full_opt_N << std::endl;
    int full_opt_N = 1200;

    while(ros::ok()){

        // double FR=1; double FL=1; double RR=1; double RL=1; // contact state
        // bool ct = true; bool in_fl_phase = false;
        
        // for(int i = 0; i < jumping_obj.full_opt_N-100; i++)
        // {

            // update current robot state
            _controlData->_legController->updateData();
            _controlData->_stateEstimator->run();

            // Get contact state
            double FR=lowState.footForce[0];
            double FL=lowState.footForce[1];
            double RR=lowState.footForce[2];
            double RL=lowState.footForce[3];
            cout<<"time index:" << counter << std::endl;
            cout<<"FR:"<< FR <<", "<< "FL:" << FL<<", "<< "RR:"<< RR<< ", " << "RL:" << RL << std::endl;


            Vec3<double> pFeetVecCOM; // relative position to CoM
            double pFeetW[4] ={0,0,0,0};
            std::vector<double> contactState ={1, 1, 1, 1}; // obtain from calculation as follow
            // std::cout << "FR:" << contactState[0] << "FL:" << contactState[1] << "RR:" << contactState[2] << "RL:" << contactState[3]<< std::endl;

            // Get the foot locations relative to COM
            for (int leg = 0; leg < 4; leg++) {
            // computeLegJacobianAndPosition(&_data->_quadruped, _data->_legController->data[leg].q,
            //                           (Mat3<double>*)nullptr, &pFeetVec, leg);
            //pFeetVecCOM = _data->_stateEstimator->getResult().rBody.transpose() *
                        //(_data->_quadruped->getHipLocation(leg) + pFeetVec);

                pFeetVecCOM =  _controlData->_stateEstimator->getResult().rBody.transpose() *
                (_controlData->_quadruped->getHipLocation(leg) + _controlData->_legController->data[leg].p);

                pFeetW[leg] = pFeetVecCOM[2] + _controlData->_stateEstimator ->getResult().position[2];
                // std::cout << "leg" << leg << std::endl;
                // std::cout << "pFeetW" << pFeetW[leg] << std::endl;
            }

            // if (pFeetW[0] > front_cube_height) contactState[0]=0;
            // if (pFeetW[1] > front_cube_height) contactState[1]=0;
            // if (pFeetW[2] > 0.01) contactState[2]=0;
            // if (pFeetW[3] > 0.01) contactState[3]=0;

            // if (pFeetW[0] > 0.01) contactState[0]=0;
            // if (pFeetW[1] > 0.01) contactState[1]=0;
            // if (pFeetW[2] > rear_cube_height) contactState[2]=0;
            // if (pFeetW[3] > rear_cube_height) contactState[3]=0;

            if (FR<0){
                contactState[0]=0; contactState[1]=0;
            }
            if (RR<0){
                contactState[2]=0; contactState[3]=0;
            }
            // cout << "pFeetW:" << pFeetW[0] << pFeetW[1] << pFeetW[2] << pFeetW[3] << std::endl;

            // if (counter % 20 == 0 && counter <= 1000){
            if (counter % 20 == 0 && counter <= flight_idx+20){
                cout<<"time index:" << counter << std::endl;
                // cout<<"FR:"<< FR<< "FL:" << FL<< "RR:"<< RR<< "RL:" << RL << std::endl;
                jumping_obj.computeAction_2D(counter, init_final, contactState);
            }

            // // get trajectory action
            if(counter< full_opt_N-posing_time){
               jumping_obj.setLegControllerCommandsFromTrajIndex_2D(counter);

            // // actually compute full torques and send
            //    jumping_obj.computeFullTorquesAndSend_constraints();
            //    jumping_obj.computeFullTorquesAndSend_constraints_v1();
               jumping_obj.computeFullTorquesAndSend_constraints();
            }

            // switch to QP
            if(counter>= full_opt_N-posing_time && counter < full_opt_N+5000){
                jumping_obj.landing(counter);
            }
            
            // if (counter < jumping_obj.full_opt_N+1000){
                // write out full state 
                jumping_obj.getFullState(full_state);
                for(int j=0; j<full_state_len; j++)
                    full_traj << full_state[j] << " ";
                full_traj << "\n";

            // }


            loop_rate.sleep();


        // } 

        // for (int i = 0; i<1000; i++){
        //     cout<<"time index:" << i << std::endl;
        //     jumping_obj.landing();
        //     loop_rate.sleep();

        // }

            counter++;

        // // continue holding for a few extra seconds (just pass in last state)
        // for(int i = 0; i < 3000; i++)
        // {

        //     // update current robot state
        //     _controlData->_legController->updateData();
        //     _controlData->_stateEstimator->run();

        //     // get trajectory action
        //     jumping_obj.setLegControllerCommandsFromTrajIndex(jumping_obj.full_opt_N-1);

        //     // HERE adjust with RL

        //     // actually compute full torques and send
        //     jumping_obj.computeFullTorquesAndSend();

        //     loop_rate.sleep();

        
        // } 
        
    }
    full_traj.close();

    // motion_init();
    // sendServoCmd();
    //std::cout << "finished" << std::endl;
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
