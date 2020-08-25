#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"



void Finger1ToTarget(uint32_t _Target, uint32_t _Speed, uint32_t _Cur, ros::Publisher &TestPub)
{
	std_msgs::Int32MultiArray val;
	val.data.push_back(1);	
    val.data.push_back(_Target);
    val.data.push_back(_Speed);
    val.data.push_back(_Cur);
	TestPub.publish(val);
	ros::spinOnce();
}

void Finger2ToTarget(uint32_t _Target, uint32_t _Speed, uint32_t _Cur, ros::Publisher &TestPub)
{
	std_msgs::Int32MultiArray val;
	val.data.push_back(2);	
    val.data.push_back(_Target);
    val.data.push_back(_Speed);
    val.data.push_back(_Cur);
	TestPub.publish(val);
	ros::spinOnce();
}

void Finger3ToTarget(uint32_t _Target, uint32_t _Speed, uint32_t _Cur, ros::Publisher &TestPub)
{
	std_msgs::Int32MultiArray val;
	val.data.push_back(3);	
    val.data.push_back(_Target);
    val.data.push_back(_Speed);
    val.data.push_back(_Cur);
	TestPub.publish(val);
	ros::spinOnce();
}

void PoseToTarget(uint32_t _Target, uint32_t _Speed, uint32_t _Cur, ros::Publisher &TestPub)
{
	std_msgs::Int32MultiArray val;
	val.data.push_back(4);	
    val.data.push_back(_Target);
    val.data.push_back(_Speed);
    val.data.push_back(_Cur);
	TestPub.publish(val);
	ros::spinOnce();
}

void ExcuteCmd(ros::Publisher &TestPub)
{
	std_msgs::Int32MultiArray val;
	val.data.push_back(0);	
    val.data.push_back(0);
    val.data.push_back(0);
    val.data.push_back(0);
	TestPub.publish(val);
	ros::spinOnce();
}
