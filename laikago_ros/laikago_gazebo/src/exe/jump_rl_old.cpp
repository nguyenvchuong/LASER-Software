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

    motion_reset();

    // string jump_trajectories[3] ={ "/home/guillaume/catkin_ws/src/USC-AlienGo-Software/laikago_ros/jumping/jumpingFull_A1_1ms_h40_d70_full_state.csv",
    //                                 "/home/guillaume/catkin_ws/src/USC-AlienGo-Software/laikago_ros/jumping/jumpingFull_A1_1ms_h20_d60_full_state.csv",
    //                                 "/home/guillaume/catkin_ws/src/USC-AlienGo-Software/laikago_ros/jumping/jumpingFull_A1_1ms_h30_d50_full_state.csv",
    //                             };
    string jump_trajectories[3] ={ "src/LASER-Software/laikago_ros/jumping/jumpingFull_A1_1ms_h30_d70_full_state.csv",
                                    "src/LASER-Software/laikago_ros/jumping/jumpingFull_A1_1ms_h25_d65_full_state.csv",
                                    "src/LASER-Software/laikago_ros/jumping/jumpingFull_A1_1ms_h20_d70_full_state.csv",
                                };
    int jump_traj_idx = 0;
    double jump_zx_coordinates[6] = { 0.3, 0.7, 
                                   0.25, 0.65,
                                   0.2, 0.7};

    bool spawn_cubes = true;
    bool front_cube = true;
    bool rear_cube = true;
    float rear_cube_height = 0.01;
    float front_cube_height = 0.08; // 0.05
    // float rear_cube_height = 0.08;
    // float front_cube_height = 0.01;

    if(spawn_cubes){
        ROS_INFO("Spawn cube..");

        // dimensions
        float dim_x = 1;
        float dim_y = 1;
        float dim_z = jump_zx_coordinates[jump_traj_idx*2];
        // location 
        float x=jump_zx_coordinates[jump_traj_idx*2+1]+0.2;
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
        float dim_x = 0.4;
        float dim_y = 0.5;
        float dim_z = front_cube_height; // change this 
        // location 
        float x=0.2;
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
        float dim_x = 0.4;
        float dim_y = 0.5;
        float dim_z = rear_cube_height; // change this 
        // location 
        float x=-0.2;
        float y=0;
        float z=dim_z;
        std::string name="rear_box";
        std::string frame_id="world";
        float mass=0.05;
        mass = 100;

        spawnCube(name,frame_id,x,y,z,0,0,0,1,dim_x,dim_y,dim_z,mass);
    }

    // reset quadruped 
    
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
    Quadruped quad ;
    quad.setQuadruped(2); // 1 for Aliengo, 2 for A1
    // initialize new leg controller and state estimate object
    StateEstimate stateEstimate;
    std::cout << "start Leg Controller" << std::endl;
    LegController* legController = new LegController(quad);

    // zero out
    // legController->zeroCommand();
    // sendServoCmd();

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
    bool USE_RL = true;

    /*

    CHECK ACTION SPACE RANGE - changed to 0.1 offsets 

    */

    //Jump_Init();
    jumping_obj.moveToTrajInitPos();

    // start at the right position 
    model_state.request.model_state.model_name = "laikago_gazebo";
    model_state.request.model_state.pose.position.z = 0.5;
    std::cout << "set model" << std::endl;
    std::cout << set_model_state_serv.call(model_state) << std::endl;
    jumping_obj.moveToTrajInitPos();
    


    // compute offsets here\

    if (USE_RL){
        cout << "Computing RL action.." << std::endl;
        jumping_obj.USE_RL = true;
        jumping_obj.computeAction();
        cout << "...done" << std::endl;
    }

    // record full state 
    // write out trajectory
    int full_state_len = 76;
    std::ofstream full_traj("jump_full_state.txt");
    std::vector<double> full_state(full_state_len);
    // jumping_obj.getFullState(full_state);
    // for(int j=0; j<full_state_len; j++)
    //     full_traj << full_state[j] << " ";
    // full_traj << "\n";

    usleep(10000000);

    if(ros::ok()){
        
        // Set the Kp and Kd value
        // double Kp = 200;
        // double Kd = 50;
        // double Hip_Kp = 300;
        // double Hip_Kd = 100;
        // update current robot state

        
        // _controlData->_legController->updateData();
        // _controlData->_stateEstimator->run();


        // double minForce = 5;
        // double maxForce = 500;
        // double contactStateScheduled[4] = {1, 1, 1, 1};

        // double minForces[4] = {minForce, minForce, minForce, minForce};
        // double maxForces[4] = {maxForce, maxForce, maxForce, maxForce};

        // Vec4<double> contactphase(0.5,0.5,0.5,0.5); 
        // _controlData->_stateEstimator->setContactPhase(contactphase);

        // Cmpc.setGaitNum(1); 
        // Vec3<double> v_des(0, 0, 0); // v_body
        // double yaw_rate = 0;
        // double roll = 0;
        // double pitch = 0;
        // v_des[0] = 0;
        // _controlData->_desiredStateCommand->setStateCommands(v_des, yaw_rate);
        // Cmpc.run(*_controlData);
        // _controlData->_legController->updateCommand();

        
        // loop_rate.sleep();
        // cout<<"counter: " << counter << std::endl;
        // counter++;

        std::cout << "Start jumping" << std::endl;
        for(int i = 0; i < jumping_obj.full_opt_N; i++)
        {

            // update current robot state
            _controlData->_legController->updateData();
            _controlData->_stateEstimator->run();

            // get trajectory action
            jumping_obj.setLegControllerCommandsFromTrajIndex(i);



            // HERE adjust with RL

            // write out full state 
            jumping_obj.getFullState(full_state);
            for(int j=0; j<full_state_len; j++)
                full_traj << full_state[j] << " ";
            full_traj << "\n";

            // actually compute full torques and send
            jumping_obj.computeFullTorquesAndSend();

            loop_rate.sleep();
        } 

        // continue holding for a few extra seconds (just pass in last state)
        for(int i = 0; i < 3000; i++)
        {

            // update current robot state
            _controlData->_legController->updateData();
            _controlData->_stateEstimator->run();

            // get trajectory action
            jumping_obj.setLegControllerCommandsFromTrajIndex(jumping_obj.full_opt_N-1);

            // HERE adjust with RL

            // actually compute full torques and send
            jumping_obj.computeFullTorquesAndSend();

            loop_rate.sleep();
        } 
        
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
