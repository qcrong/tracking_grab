#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <trac_ik/trac_ik.hpp>
#include <tf/transform_listener.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Transform.h>
#include <Eigen/Eigen>

#include "ur_grab/gripperControl.h"

typedef  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>  Client;
Client *left_client;
const std::string defaultname[6] = {
                                    "shoulder_pan_joint",
                                    "shoulder_lift_joint",
                                    "elbow_joint",
                                    "wrist_1_joint",
                                    "wrist_2_joint",
                                    "wrist_3_joint"
                                    };

TRAC_IK::TRAC_IK *left_track_ik_solver;
KDL::ChainFkSolverPos_recursive *left_kdl_fk_solver;
KDL::Chain left_chain;
KDL::Frame left_home_Pos, left_Pos[10]; //left_place_Pos,
trajectory_msgs::JointTrajectoryPoint left_curTra_Joint, left_targetTra_Joint;
control_msgs::FollowJointTrajectoryGoal left_goal;
geometry_msgs::Wrench left_loc_wrench, left_put_wrench;
//ros::Publisher pub;

//unsigned int left_step_num = 0;
//unsigned int left_step_state = 0;
unsigned int track_num = 1;
//unsigned int left_insert_state = 0;

//unsigned int touch_state = 0;

ros::Publisher gripperPub;
//ros::Publisher TestPub;
//ros::Publisher grasp_finish_Pub;

//手眼关系
Eigen::Matrix3d base2eye_r;
Eigen::Vector3d base2eye_t;
Eigen::Quaterniond base2eye_q;

Eigen::Vector3d hand2tool0_t;


//读取当前关节位置信息
void left_jointstates_subCB(const sensor_msgs::JointState &JS)
{
    for(int i=0;i<JS.name.size();i++)
    {
        for(int j=0;j<6;j++)
        {
            if(left_goal.trajectory.joint_names[j] == JS.name[i])
            {
                left_curTra_Joint.positions[j] = JS.position[i];
                //left_curTra_Joint.velocities[j]=JS.velocity[i];
            }
        }
    }
}

//插入轨迹点
bool left_insertTrack(KDL::Frame insertPoint, float tm)
{
    KDL::JntArray cur_JointArray(6), target_JointArray;
    if(track_num == 1)
    {
        left_curTra_Joint.time_from_start = ros::Duration(0);
        for(int i=0;i<6;i++)
            left_curTra_Joint.velocities[i] = 0.0;
        left_goal.trajectory.points[0] = left_curTra_Joint;
        for(int i=0;i<6;i++) cur_JointArray(i) = left_curTra_Joint.positions[i];
    }
    else
    {
        for(int i=0;i<6;i++) cur_JointArray(i) = left_targetTra_Joint.positions[i]; //添加上一周期计算出来的关节角
    }

    track_num++;
    left_goal.trajectory.points.resize(track_num);
    int rc = left_track_ik_solver->CartToJnt(cur_JointArray, insertPoint, target_JointArray);

    if(rc <= 0)
    {
        ROS_INFO("can't reach the goal.");
        //left_insert_state = 0;
        return false;
    }

    double max_delta_joint=0;
    for(int i=0;i<6;i++)  
    {
        left_targetTra_Joint.positions[i] = target_JointArray(i);
        left_targetTra_Joint.velocities[i] = 0.0;
        double delta=fabs(cur_JointArray(i)-target_JointArray(i));
        if(max_delta_joint<delta){
            max_delta_joint=delta;
        }
    }
    tm=1.0/max_delta_joint;
    if(tm<0.01) tm=0.01;
    std::cout<<"tm:"<<tm<<std::endl;
    left_targetTra_Joint.time_from_start = ros::Duration(tm);
    left_goal.trajectory.points[track_num-1] = left_targetTra_Joint;
    left_goal.trajectory.header.stamp = ros::Time::now();
    return true;
}

//执行轨迹点
bool left_excTrack(void)
{
//    left_client->cancelAllGoals();
//    ros::Duration(0.1).sleep();
    left_client->sendGoal(left_goal);
    //ros::Duration(0.05).sleep();
    left_client->waitForResult(ros::Duration(0));

    if(left_client->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_WARN("Robot Err, can't reach the goal.");
        return false;
    }
    else{
        std::cout<<"reach the goal."<<std::endl;
    }


    track_num = 1;
	return true;
}

//位姿点初始化
void variable_init(void)
{
	/*
	left_Pos[0].p = KDL::Vector(-0.271, 0.19, 0.94);
	left_Pos[0].M = KDL::Rotation::RPY(-1.571, 0.117, 0.006);
	//left_loc_Pos.M = KDL::Rotation::Quaternion(-0.5355, -0.3850, 0.6095, -0.44);

	left_Pos[1].p = KDL::Vector(-0.422, -0.161, 0.834);
	left_Pos[1].M = KDL::Rotation::RPY(-1.571, -0.238, 0.802);
	//left_put_Pos.M = KDL::Rotation::Quaternion(-0.5355, -0.3850, 0.6095, -0.44);
	*/
	
	left_home_Pos.p=KDL::Vector(0.02234, 0.78759, 0.25);
	left_home_Pos.M = KDL::Rotation::RPY(3.14, 0.0, 0.0);
	
    base2eye_r<<0.9978181985477985, 0.05899849817026713, -0.02963139990753466,
                0.06178207154344488, -0.6761810618174333, 0.734140413868662,
                0.02327699041173496, -0.7343693545165834, -0.67835081842972;

    base2eye_t<<0.01787618021554105,-1.471458374882839,0.735535095875542;
    base2eye_q=base2eye_r;
    hand2tool0_t<<0,0,-0.2;
}

/*
void slow_move_to(KDL::Frame &_pos, double _v)	//这边没有调用
{
    KDL::Frame _pos_(_pos);
	bool state = left_insertTrack(_pos, 4);
	if(state) left_excTrack();

    _v = _pos.p.z()-_v;
    _pos_.p.z(_v);
	state = left_insertTrack(_pos_, 4);
	if(state) left_excTrack();

	state = left_insertTrack(_pos, 4);
	if(state) left_excTrack();
}
*/

//插入目标点
//void left_goal_task(void)
//{
//	ROS_INFO("left ur start.");
//	//while(ros::ok())
//	{
//		bool state = left_insertTrack(left_home_Pos, 8);
//		if(state) left_excTrack();
//		ros::Duration(1).sleep();

//		//Finger1ToTarget(0, 500, 500, TestPub);
//		//Finger2ToTarget(0, 500, 500, TestPub);
//		//Finger3ToTarget(0, 500, 500, TestPub);
//		//PoseToTarget(0, 500, 500);
//		//ExcuteCmd(TestPub);

///*
//		bool state = left_insertTrack(left_Pos[0], 8);
//		if(state) left_excTrack();
//		ros::Duration(1).sleep();
		
//		state = left_insertTrack(left_Pos[1], 2);
//		if(state) left_excTrack();
//		ros::Duration(1).sleep();
		
//		state = left_insertTrack(left_Pos[2], 2);
//		if(state) left_excTrack();
//		ros::Duration(1).sleep();
//*/
//	}
//}

void robot_target_subCB(const geometry_msgs::Transform & _position)
{
    static int t=1;
    std::cout<<"receive t: "<<t<<std::endl;


    //目标物在相机坐标系下的坐标转机器人坐标系下的坐标
    Eigen::Vector3d eye_center3d, base_center3d;
    eye_center3d(0)=_position.translation.x;
    eye_center3d(1)=_position.translation.y;
    eye_center3d(2)=_position.translation.z;

    base_center3d=base2eye_r*eye_center3d+base2eye_t;//目标转换到基座标系下
    Eigen::Quaterniond base_quater(_position.rotation.w,_position.rotation.x,_position.rotation.y,_position.rotation.z);//eye2obj
    base_quater=base2eye_q*base_quater;
    base_center3d=base_quater*hand2tool0_t+base_center3d;//考虑手抓偏移

    std::cout<<"base_center3d "<<base_center3d(0)<<" "<<base_center3d(1)<<" "<<base_center3d(2)<<" "<<std::endl;
    std::cout<<"base_quater "<<base_quater.x()<<" "<<base_quater.y()<<" "<<base_quater.z()<<" "<<base_quater.w()<<" "<<std::endl;
	KDL::Frame _pos_;
    _pos_.p = KDL::Vector(base_center3d(0), base_center3d(1), base_center3d(2));
    _pos_.M=KDL::Rotation::Quaternion(base_quater.x(),base_quater.y(),base_quater.z(),base_quater.w());


	//到达抓取点上方
    bool state = left_insertTrack(_pos_, 10);
    if(state && t==1){
        left_excTrack();
        //ros::Duration(1).sleep();
        //sendGripperMsg(gripperPub,150); //闭合爪子
    }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ur_grab_trac_ik");

  ros::NodeHandle n;
  ros::Subscriber left_jointstates_sub = n.subscribe("joint_states", 1, left_jointstates_subCB);
  ros::Subscriber robot_target_sub=n.subscribe("/gpf/position",1,robot_target_subCB);
  gripperPub=n.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("/Robotiq2FGripperRobotOutput", 1);
  //TestPub = n.advertise<std_msgs::Int32MultiArray>("CohandTopic", 5);
  //grasp_finish_Pub=n.advertise<std_msgs::Int8>("grasp_finish", 1);

  left_goal.trajectory.joint_names.resize(6);
  left_goal.trajectory.points.resize(6);
  left_curTra_Joint.positions.resize(6);
  left_curTra_Joint.velocities.resize(6);
  left_targetTra_Joint.velocities.resize(6);
  left_targetTra_Joint.positions.resize(6);

  for(int i=0;i<6;i++)
  {
    left_goal.trajectory.joint_names[i] = defaultname[i];
  }
  	
  left_track_ik_solver = new TRAC_IK::TRAC_IK("base", "tool0", "/robot_description");
  KDL::Chain chain1;
  left_track_ik_solver->getKDLChain(chain1);
  left_track_ik_solver->SetSolveType(TRAC_IK::Distance);
  left_kdl_fk_solver = new KDL::ChainFkSolverPos_recursive(chain1);


  left_client = new Client("follow_joint_trajectory", true);
  //arm_controller/follow_joint_trajectory
  ROS_INFO("waiting for server...");
  bool left_state = left_client->waitForServer(ros::Duration(4));

  if(left_state) ROS_INFO("connected the server.");
  else {ROS_INFO("connect the server failed."); return 0;}

  variable_init();
  initializeGripperMsg(gripperPub);//爪子初始化
  //sendGripperMsg(gripperPub,0);

  //ros::Timer timer = n.createTimer(ros::Duration(0.1), boost::bind(&left_goal_task), true);//循环执行
  ros::MultiThreadedSpinner spinner(5);
  spinner.spin();
  left_client->cancelAllGoals();
  ros::Duration(1).sleep();

  return 0;
}
