#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Eigen>

#include "ur_grab_pbvs/gripperControl.h"
#include <gpf/obj_tool_transform.h>
#include "opencv2/opencv.hpp"

ros::Publisher gripperPub;
ros::Publisher tool_vel_pub;

//手眼关系
Eigen::Matrix3d base2eye_r;
Eigen::Vector3d base2eye_t;
Eigen::Quaterniond base2eye_q;

Eigen::Vector3d hand2tool0_t;

//读取当前关节位置信息
void variable_init(void)
{
    base2eye_r<<0.9978181985477985, 0.05899849817026713, -0.02963139990753466,
                0.06178207154344488, -0.6761810618174333, 0.734140413868662,
                0.02327699041173496, -0.7343693545165834, -0.67835081842972;

    base2eye_t<<0.01787618021554105,-1.471458374882839,0.735535095875542;
    base2eye_q=base2eye_r;
    hand2tool0_t<<0,0,-0.2;
}

//插入轨迹点
void cmd_tool_vel_pub(geometry_msgs::Twist &toolVel_, double time=0){
    tool_vel_pub.publish(toolVel_);

    if(time!=0)
    {
        geometry_msgs::Twist vel;
        ros::Duration(time).sleep();
        vel.linear.x=0;
        vel.linear.y=0;
        vel.linear.z=0;
        vel.angular.x=0;
        vel.angular.y=0;
        vel.angular.z=0;
        tool_vel_pub.publish(vel);
    }
    //ros::Duration(0.1).sleep();
}


void robot_target_subCB(const gpf::obj_tool_transform &transform_)
{
    //目标物在相机坐标系下的坐标转机器人坐标系下的坐标
    Eigen::Vector3d eye_center3d, bTtd;
    eye_center3d(0)=transform_.cam2obj.translation.x;
    eye_center3d(1)=transform_.cam2obj.translation.y;
    eye_center3d(2)=transform_.cam2obj.translation.z;

    bTtd=base2eye_r*eye_center3d+base2eye_t;//目标转换到基座标系下
    Eigen::Quaterniond bRtd_q(transform_.cam2obj.rotation.w, transform_.cam2obj.rotation.x, transform_.cam2obj.rotation.y, transform_.cam2obj.rotation.z);//eye2obj
    bRtd_q=base2eye_q*bRtd_q;
    bTtd=bRtd_q*hand2tool0_t+bTtd;//考虑手抓偏移

    std::cout<<"bTtd "<<bTtd(0)<<" "<<bTtd(1)<<" "<<bTtd(2)<<" "<<std::endl;
    std::cout<<"bRtd_q "<<bRtd_q.x()<<" "<<bRtd_q.y()<<" "<<bRtd_q.z()<<" "<<bRtd_q.w()<<" "<<std::endl;

    //机械臂末端坐标系tool0在基座标系base下的位姿
    Eigen::Vector3d bTt;
    bTt(0)=transform_.base2tool0.translation.x;
    bTt(1)=transform_.base2tool0.translation.y;
    bTt(2)=transform_.base2tool0.translation.z;
    Eigen::Quaterniond bRt_q(transform_.base2tool0.rotation.w, transform_.base2tool0.rotation.x, transform_.base2tool0.rotation.y, transform_.base2tool0.rotation.z);
    //转换到末端理想位姿坐标系下
    Eigen::Quaterniond tdRb=bRtd_q.inverse();
    Eigen::Quaterniond tRtd_q=bRt_q.inverse()*bRtd_q;
    Eigen::Vector3d tdTt=tdRb*bTt-tdRb*bTtd;

    //轴角
    Eigen::AngleAxisd tdRt_tu(tRtd_q.inverse());
    std::cout<<"tdRt_tu angle"<<tdRt_tu.angle()<<std::endl;
    std::cout<<"tdRt_tu axis"<<tdRt_tu.axis()<<std::endl;
    //opencv
    Eigen::Matrix3d tdRt_m(tRtd_q.inverse());
    cv::Mat tdRt_mat(3,3,CV_32F);
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++){
            tdRt_mat.at<float>(i,j)=tdRt_m(i,j);
        }
    cv::Mat tdRt_tu_m;
    cv::Rodrigues(tdRt_mat,tdRt_tu_m);
    std::cout<<"tdRt_tu_m"<<tdRt_tu_m<<std::endl;
	
    //速度转换到基座标系下
    
    //速度发布
//    geometry_msgs::Twist toolVel;
//    toolVel.linear.x=;
//    toolVel.linear.y=;
//    toolVel.linear.z=;
//    toolVel.angular.x=;
//    toolVel.angular.y=;
//    toolVel.angular.z=;
//    tool_vel_pub(toolVel);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ur_grab_pbvs");

  ros::NodeHandle n;
  ros::Subscriber robot_target_sub=n.subscribe("/gpf/position",1,robot_target_subCB);
  gripperPub=n.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("/Robotiq2FGripperRobotOutput", 1);
  tool_vel_pub = n.advertise<geometry_msgs::Twist>("/ur_arm_controller/cmd_tool_vel", 1);  //ur_arm速度控制

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);  //获取base坐标系下tool0的位姿

  variable_init();
  initializeGripperMsg(gripperPub);//手抓初始化
  //sendGripperMsg(gripperPub,0);

  //ros::Timer timer = n.createTimer(ros::Duration(0.1), boost::bind(&left_goal_task), true);
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  return 0;
}
