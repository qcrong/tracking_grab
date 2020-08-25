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
#include <geometry_msgs/Point.h>
#include "CohandTestNode.h"

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
KDL::Frame left_home_Pos,left_place_Pos, left_Pos[10];
trajectory_msgs::JointTrajectoryPoint left_curTra_Joint, left_targetTra_Joint;
control_msgs::FollowJointTrajectoryGoal left_goal;
geometry_msgs::Wrench left_loc_wrench, left_put_wrench;
//ros::Publisher pub;

//unsigned int left_step_num = 0;
//unsigned int left_step_state = 0;
unsigned int left_track_num = 1;
unsigned int left_insert_state = 0;

//unsigned int touch_state = 0;

ros::Publisher TestPub;
ros::Publisher grasp_finish_Pub;


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
            }
        }
    }
}

//插入轨迹点
bool left_insertTrack(KDL::Frame insertPoint, float tm)
{
    KDL::JntArray cur_JointArray(6), target_JointArray;
    if(left_track_num == 1)
    {
        left_curTra_Joint.time_from_start = ros::Duration(0);
        for(int i=0;i<6;i++)
            left_curTra_Joint.velocities[i] = 0.0;
        left_goal.trajectory.points[0] = left_curTra_Joint;
        for(int i=0;i<6;i++) cur_JointArray(i) = left_curTra_Joint.positions[i];
    }
    else
    {
        for(int i=0;i<6;i++) cur_JointArray(i) = left_targetTra_Joint.positions[i];
    }

    left_track_num++;
    left_goal.trajectory.points.resize(left_track_num);
    int rc = left_track_ik_solver->CartToJnt(cur_JointArray, insertPoint, target_JointArray);
    if(rc <= 0)
    {
        ROS_INFO("can't reach the goal.");
        left_insert_state = 0;
        return false;
    }

    for(int i=0;i<6;i++)  
    {
        left_targetTra_Joint.positions[i] = target_JointArray(i);
        left_targetTra_Joint.velocities[i] = 0.0;
    }
    left_targetTra_Joint.time_from_start = ros::Duration(tm);
    left_goal.trajectory.points[left_track_num-1] = left_targetTra_Joint;
    left_goal.trajectory.header.stamp = ros::Time::now();
    return true;
}

//执行轨迹点
bool left_excTrack(void)
{
    left_client->sendGoal(left_goal);
    left_client->waitForResult(ros::Duration(0));

    if(left_client->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
		ROS_WARN("Robot Err, can't reach the goal.");
		return false;
    }

    left_track_num = 1;
	return true;
}

//位姿点初始化
void position_init(void)
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

	left_place_Pos.p = KDL::Vector(0.29, 0.67, 0.09);
	left_place_Pos.M = KDL::Rotation::RPY(3.14, 0.0, 0.0);
/*
	left_Pos[0].p = KDL::Vector(0.215, -0.369, 0.500);
	left_Pos[0].M = KDL::Rotation::RPY(-3.076, 0.0, -2.35);

	left_Pos[1].p = KDL::Vector(0.216, -0.501, 0.202);
	left_Pos[1].M = KDL::Rotation::RPY(3.0584, 0.0, -2.53);
	
	left_Pos[2].p = KDL::Vector(0.418, -0.618, 0.226);
	left_Pos[2].M = KDL::Rotation::RPY(3.11, 0.0, -2.4);
*/
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
void left_goal_task(void)
{
	ROS_INFO("left ur start.");
	//while(ros::ok())
	{
		bool state = left_insertTrack(left_home_Pos, 8);
		if(state) left_excTrack();
		ros::Duration(1).sleep();

		Finger1ToTarget(0, 500, 500, TestPub);
		Finger2ToTarget(0, 500, 500, TestPub);
		Finger3ToTarget(0, 500, 500, TestPub);
		//PoseToTarget(0, 500, 500);
		ExcuteCmd(TestPub);

/*
		bool state = left_insertTrack(left_Pos[0], 8);
		if(state) left_excTrack();
		ros::Duration(1).sleep();
		
		state = left_insertTrack(left_Pos[1], 2);
		if(state) left_excTrack();
		ros::Duration(1).sleep();
		
		state = left_insertTrack(left_Pos[2], 2);
		if(state) left_excTrack();
		ros::Duration(1).sleep();
*/
	}
}

void robot_target_subCB(const geometry_msgs::Point &_point)
{
	geometry_msgs::Point _point_ = _point;
	KDL::Frame _pos_;
	//if(_point_.x<0.2) _point_.x = 0.2;
	//if(_point_.x>0.7) _point_.x = 0.7;
	//if(_point_.y<0.0) _point_.y = 0.0;
	//if(_point_.y>0.6) _point_.y = 0.6;
	_point_.z+=0.195;
	_pos_.p = KDL::Vector(_point_.x, _point_.y, _point_.z);
	_pos_.M = KDL::Rotation::RPY(3.14, 0.0, 0.0);
	//到达抓取点上方
	bool state = left_insertTrack(_pos_, 1);
	if(state) left_excTrack();
	//ros::Duration(1).sleep();
	//抓取
	Finger1ToTarget(600, 1000, 800, TestPub);
	Finger2ToTarget(600, 1000, 800, TestPub);
	Finger3ToTarget(600, 1000, 800, TestPub);
	//PoseToTarget(0, 500, 500);
	ExcuteCmd(TestPub);
	ros::Duration(0.5).sleep();

	//移动到放置点	
	state = left_insertTrack(left_place_Pos, 1);
	if(state) left_excTrack();
	ros::Duration(0.5).sleep();
	//放置
	Finger1ToTarget(0, 1000, 500, TestPub);
	Finger2ToTarget(0, 1000, 500, TestPub);
	Finger3ToTarget(0, 1000, 500, TestPub);
	//PoseToTarget(0, 500, 500);
	ExcuteCmd(TestPub);

	//回初始位置
	state = left_insertTrack(left_home_Pos, 1);
	if(state) left_excTrack();
	
	std_msgs::Int8 grasp_finish;
    grasp_finish.data=1;
	grasp_finish_Pub.publish(grasp_finish);
	ros::Duration(1).sleep();
	
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "single_robot_move_test");

  ros::NodeHandle n;
  ros::Subscriber left_jointstates_sub = n.subscribe("joint_states", 10, left_jointstates_subCB);
  ros::Subscriber robot_target_sub=n.subscribe("grasp_point",1,robot_target_subCB);
  TestPub = n.advertise<std_msgs::Int32MultiArray>("CohandTopic", 5);
  grasp_finish_Pub=n.advertise<std_msgs::Int8>("grasp_finish", 1);

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

  position_init();
  ros::Timer timer = n.createTimer(ros::Duration(0.1), boost::bind(&left_goal_task), true);
  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();

  left_client->cancelGoal();

  return 0;
}
