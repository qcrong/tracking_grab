#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <math.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int8.h>
#include <Eigen/Eigen>
#include "ur_arm/Joints.h"
#include "calibration/toolposeChange.h"

using namespace std;
using namespace Eigen;

const float deg2rad = M_PI / 180.0;//用于将角度转化为弧度。rad = deg*deg2rad
const float rad2deg = 180.0 / M_PI;//用于将弧度转化为角度。deg = rad*rad2deg

//沿末端坐标系XYZ轴进行平移和旋转运动,time=0一直运动下去，time不为0时运动time时间停止,单位为s
int tool_vel(VectorXd Velocity_, const tf2_ros::Buffer &tfBuffer_, const ros::Publisher &tool_vel_pub_, double time=0)
{
  	geometry_msgs::TransformStamped transformStamped;
  	try
  	{
    	transformStamped = tfBuffer_.lookupTransform( "base", "tool0", ros::Time(0),ros::Duration(0.5));
  	}
  	catch (tf2::TransformException &ex) 
  	{
    	ROS_WARN("%s",ex.what());
    	return -1;
  	}

  	Quaterniond rotQ(transformStamped.transform.rotation.w,
                   	transformStamped.transform.rotation.x,
                   	transformStamped.transform.rotation.y,
                   	transformStamped.transform.rotation.z);
  	Matrix3d rotM;
  	rotM=rotQ.matrix();
  	Vector3d v_t, v_r;
  	v_t<<Velocity_(0),Velocity_(1),Velocity_(2);
  	v_r<<Velocity_(3),Velocity_(4),Velocity_(5);
  	v_t=rotM*v_t;
  	v_r=rotM*v_r;
  
  	geometry_msgs::Twist toolVel;

  	toolVel.linear.x=v_t(0);
  	toolVel.linear.y=v_t(1);
  	toolVel.linear.z=v_t(2);
  	toolVel.angular.x=v_r(0);
  	toolVel.angular.y=v_r(1);
  	toolVel.angular.z=v_r(2);
  	tool_vel_pub_.publish(toolVel);
  
  	if(time!=0)
  	{
    	ros::Duration(time).sleep();
    	toolVel.linear.x=0;
    	toolVel.linear.y=0;
    	toolVel.linear.z=0;
    	toolVel.angular.x=0;
    	toolVel.angular.y=0;
    	toolVel.angular.z=0;
    	tool_vel_pub_.publish(toolVel);
  	}
  	//ros::Duration(0.1).sleep();
  	return 0;
}

//旋转矩阵和平移向量转齐次矩阵
MatrixXd rt2m(const VectorXd pose_t,const Matrix3d pose_r)
{
	MatrixXd pose_m = MatrixXd::Zero(4, 4);
	for(int j=0; j<3; j++)
  	{
    	for(int k=0; k<3; k++)	
		{
	 	 	pose_m(j,k)=pose_r(j,k);
		}		
  	}
	for(int j=0; j<3; j++)
	{
	  	pose_m(j,3)=pose_t(j);
	}
	pose_m(3,3)=1.0;

	return pose_m;
}

//获取基座标系base下末端坐标系tool0的位置pose_t和姿态pose_r（旋转矩阵）
int getPose(const tf2_ros::Buffer &tfBuffer_,VectorXd &pose_t,Matrix3d &pose_r)
{
   	geometry_msgs::TransformStamped transformStamped;
   	try
   	{
     	transformStamped = tfBuffer_.lookupTransform( "base", "tool0", ros::Time(0),ros::Duration(0.5));
   	}
   	catch (tf2::TransformException &ex) 
   	{
     	ROS_WARN("%s",ex.what());
     	return -1;
   	}
   
   	Quaterniond rotQ(transformStamped.transform.rotation.w,
                   	transformStamped.transform.rotation.x,
                   	transformStamped.transform.rotation.y,
                   	transformStamped.transform.rotation.z);

   	pose_r=rotQ.matrix();
 
   	pose_t<<transformStamped.transform.translation.x,
            	transformStamped.transform.translation.y,
            	transformStamped.transform.translation.z;
    //添加验证部分
  	Vector3d rotRPY=pose_r.eulerAngles(2,1,0); //ZYX欧拉角 
   
	//RPYY与示教器上的RPY一致
  	cout<<"translation && RPY"<<endl
			<<transformStamped.transform.translation.x<<endl
            <<transformStamped.transform.translation.y<<endl
            <<transformStamped.transform.translation.z<<endl
            <<rotRPY.z()<<endl
            <<rotRPY.y()<<endl
            <<rotRPY.x()<<endl;

   	return 0;   
}

//获取old位姿坐标系下，从old到new的位置和姿态增量
MatrixXd getPoseIncreasement(VectorXd poseNew_t,Matrix3d &poseNew_r,VectorXd poseOld_t,Matrix3d &poseOld_r)
{
  
  Matrix3d R_old_new;
  VectorXd t_old_new(3);
  R_old_new = poseOld_r.transpose()*poseNew_r;
  t_old_new = poseOld_r.transpose()*poseNew_t-poseOld_r.transpose()*poseOld_t;

  MatrixXd old_new = MatrixXd::Zero(4, 4);
  for(int j=0; j<3; j++)
  {
    for(int k=0; k<3; k++)	
	{
	  old_new(j,k)=R_old_new(j,k);
	}		
  }
	for(int j=0; j<3; j++)
	{
	  old_new(j,3)=t_old_new(j);
	}
	old_new(3,3)=1.0;

 // cout<<"d_pose"<<endl<<d_pose_<<endl;

  return old_new;
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "camera_inter_calibration");   //定义节点名称
	ros::NodeHandle handle;   //为这个进程的节点创建一个句柄
	
	tf2_ros::Buffer tfBuffer;
  	tf2_ros::TransformListener tfListener(tfBuffer);  //获取机械臂末端在基坐标系下的位姿

	//发布的消息
	ros::Publisher joint_pos_pub = handle.advertise<ur_arm::Joints>("/ur_arm_controller/cmd_joint_pos", 1);      //关节角度控制
	ros::Publisher tool_vel_pub = handle.advertise<geometry_msgs::Twist>("/ur_arm_controller/cmd_tool_vel", 1);  //末端速度控制
	ros::Publisher signal_pub = handle.advertise<std_msgs::Int8>("hand_eye_arm_signal", 1);  //到达位姿后，发送标志位
	ros::Publisher pose_change_pub = handle.advertise<calibration::toolposeChange>("hand_eye_arm_pose_change", 10);//发送末端位姿齐次变换矩阵
	ros::Publisher pose_pub = handle.advertise<calibration::toolposeChange>("hand_eye_arm_pose", 10);//发送末端位姿齐次变换矩阵

	Matrix3d poseNew_r = MatrixXd::Zero(3, 3);						//本周期获取的姿态
	VectorXd poseNew_t = MatrixXd::Zero(3, 1);						//理想位置
	Matrix3d poseOld_r = MatrixXd::Zero(3, 3);						//本周期获取的姿态
	VectorXd poseOld_t = MatrixXd::Zero(3, 1);						//理想位置
	MatrixXd d_pose = MatrixXd::Zero(4, 4);						//用于某一次存放位姿变化增量的向量

	std_msgs::Int8 flag;	//到达位姿后，发送标志位
	calibration::toolposeChange poseChange;   //末端位姿齐次变换矩阵
	MatrixXd pose_m=MatrixXd::Zero(4, 4);   //基坐标系下末端位姿
	calibration::toolposeChange pose;   //基坐标系下末端位姿
	flag.data=1;

	/**************************************************************************************************************************/
	/****************************************************1. 初始位姿***************************************************/
	/**************************************************************************************************************************/
	cout<<"位姿0 拍摄第1张照片"<<endl;
	ur_arm::Joints jointPos;   //关节角度值
  	//double p[6]={105,-145,-95,85,80,-10};  //关节角度值
	double p[6]={-90,-150,-63,-15,90,-90};  //关节角度值

  	for(int i=0;i<6;i++)
  	{
    	p[i]=deg2rad*p[i];          //关节角度值转弧度值
  	}
  	jointPos.base=p[0];
  	jointPos.shoulder=p[1];
  	jointPos.elbow=p[2];
  	jointPos.wrist1=p[3];
  	jointPos.wrist2=p[4];
  	jointPos.wrist3=p[5];
  	ros::Duration(0.3).sleep();
  	joint_pos_pub.publish(jointPos);
   	ros::Duration(12).sleep();

	getPose(tfBuffer,poseNew_t,poseNew_r);
	cout<<"poseNew_r"<<endl<<poseNew_r<<endl;
	cout<<"poseNew_t"<<endl<<poseNew_t<<endl;
	poseOld_t=poseNew_t;
	poseOld_r=poseNew_r;

	//发送标志位
	signal_pub.publish(flag);
	ros::Duration(2).sleep();
	flag.data=2;

	/**************************************************************************************************************************/
	/****************************************************2. 位姿1***************************************************/
	/**************************************************************************************************************************/
	cout<<"位姿1 拍摄第2张照片"<<endl;
    VectorXd vel(6);
  	vel<<0.03, -0.02, 0.0, 5.0*deg2rad, 3.0*deg2rad, 0.0 *deg2rad;
  	tool_vel(vel,tfBuffer,tool_vel_pub,2);
    ros::Duration(5).sleep();
	
	getPose(tfBuffer,poseNew_t,poseNew_r);
	d_pose=getPoseIncreasement(poseNew_t,poseNew_r,poseOld_t,poseOld_r);
	cout<<"poseNew_r"<<endl<<poseNew_r<<endl;
	cout<<"poseNew_t"<<endl<<poseNew_t<<endl;
	cout<<"d_pose"<<endl<<d_pose<<endl;
	poseOld_t=poseNew_t;
	poseOld_r=poseNew_r;

	//当前末端位姿
	pose_m=rt2m(poseNew_t,poseNew_r);	
	//发送运动前到运动后末端位姿的变换矩阵
	for(int i=0; i<4; i++)
    {
      for(int j=0; j<4; j++)	
	  {
	    poseChange.pose_change[4*i+j]=d_pose(i,j);
		pose.pose_change[4*i+j]=pose_m(i,j);
	  }		
    }
    pose_change_pub.publish(poseChange);
	pose_pub.publish(pose);
	//发送标志位
	signal_pub.publish(flag);
	ros::Duration(2).sleep();
	flag.data=3;

	/**************************************************************************************************************************/
	/****************************************************3. 位姿2***************************************************/
	/**************************************************************************************************************************/
	cout<<"位姿2 拍摄第3张照片"<<endl;
	//VectorXd vel(6);
  	vel<<-0.07, 0.0, 0.02, 0.0, -5*deg2rad, -5*deg2rad;
  	tool_vel(vel,tfBuffer,tool_vel_pub,2);
  	ros::Duration(3).sleep();//稍作延时

	getPose(tfBuffer,poseNew_t,poseNew_r);
	d_pose=getPoseIncreasement(poseNew_t,poseNew_r,poseOld_t,poseOld_r);
	cout<<"poseNew_r"<<endl<<poseNew_r<<endl;
	cout<<"poseNew_t"<<endl<<poseNew_t<<endl;
	cout<<"d_pose"<<endl<<d_pose<<endl;
	poseOld_t=poseNew_t;
	poseOld_r=poseNew_r;

	//当前末端位姿
	pose_m=rt2m(poseNew_t,poseNew_r);	
	//发送运动前到运动后末端位姿的变换矩阵
	for(int i=0; i<4; i++)
    {
      for(int j=0; j<4; j++)	
	  {
	    poseChange.pose_change[4*i+j]=d_pose(i,j);
		pose.pose_change[4*i+j]=pose_m(i,j);
	  }		
    }
    pose_change_pub.publish(poseChange);
	pose_pub.publish(pose);
	//发送标志位
	signal_pub.publish(flag);
	ros::Duration(2).sleep();
	flag.data=4;
	
	/**************************************************************************************************************************/
	/****************************************************4. 位姿3***************************************************/
	/**************************************************************************************************************************/
	cout<<"位姿3 拍摄第4张照片"<<endl;
  	vel<<-0.05, 0.0, 0.0, -10*deg2rad, 10*deg2rad, 0.0;
  	tool_vel(vel,tfBuffer,tool_vel_pub,2);
  	ros::Duration(3).sleep();//稍作延时

	getPose(tfBuffer,poseNew_t,poseNew_r);
	d_pose=getPoseIncreasement(poseNew_t,poseNew_r,poseOld_t,poseOld_r);
	cout<<"poseNew_r"<<endl<<poseNew_r<<endl;
	cout<<"poseNew_t"<<endl<<poseNew_t<<endl;
	cout<<"d_pose"<<endl<<d_pose<<endl;
	poseOld_t=poseNew_t;
	poseOld_r=poseNew_r;

	//当前末端位姿
	pose_m=rt2m(poseNew_t,poseNew_r);	
	//发送运动前到运动后末端位姿的变换矩阵
	for(int i=0; i<4; i++)
    {
      for(int j=0; j<4; j++)	
	  {
	    poseChange.pose_change[4*i+j]=d_pose(i,j);
		pose.pose_change[4*i+j]=pose_m(i,j);
	  }		
    }
    pose_change_pub.publish(poseChange);
	pose_pub.publish(pose);
	//发送标志位
	signal_pub.publish(flag);
	ros::Duration(2).sleep();
	flag.data=5;

	/**************************************************************************************************************************/
	/****************************************************5. 位姿4***************************************************/
	/**************************************************************************************************************************/
	cout<<"位姿4 拍摄第5张照片"<<endl;
  	vel<< -0.05,0.04,0.01,10*deg2rad,-5*deg2rad,5*deg2rad;
  	tool_vel(vel,tfBuffer,tool_vel_pub,2);
  	ros::Duration(3).sleep();//稍作延时

	getPose(tfBuffer,poseNew_t,poseNew_r);
	d_pose=getPoseIncreasement(poseNew_t,poseNew_r,poseOld_t,poseOld_r);
	cout<<"poseNew_r"<<endl<<poseNew_r<<endl;
	cout<<"poseNew_t"<<endl<<poseNew_t<<endl;
	cout<<"d_pose"<<endl<<d_pose<<endl;
	poseOld_t=poseNew_t;
	poseOld_r=poseNew_r;

	//当前末端位姿
	pose_m=rt2m(poseNew_t,poseNew_r);	
	//发送运动前到运动后末端位姿的变换矩阵
	for(int i=0; i<4; i++)
    {
      for(int j=0; j<4; j++)	
	  {
	    poseChange.pose_change[4*i+j]=d_pose(i,j);
		pose.pose_change[4*i+j]=pose_m(i,j);
	  }		
    }
    pose_change_pub.publish(poseChange);
	pose_pub.publish(pose);
	//发送标志位
	signal_pub.publish(flag);
	ros::Duration(2).sleep();//稍作延时
        flag.data=6;


	return 0;
}
