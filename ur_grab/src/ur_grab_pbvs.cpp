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
ros::Publisher tool_pos_pub;

//手眼关系
Eigen::Matrix3d base2eye_r;
Eigen::Vector3d base2eye_t;
Eigen::Quaterniond base2eye_q;

Eigen::Vector3d hand2tool0_t;//跟踪时手抓偏移
Eigen::Vector3d obj2hand_t;//抓取时手抓偏移

//pbvs速度系数
float kp=3.5;
//float ti=0.0;
//float td=0.0;
//计时
//double time_old_sec=0.0;


//读取当前关节位置信息
void variable_init(void)
{
    base2eye_r<<0.9995315118578345, -0.009752539532206575, -0.02901111469529991,
    0.01948326657704545, -0.5282719873530139, 0.8488516417499494,
    -0.02360421840855667, -0.8490191961026374, -0.5278344868635918;

    base2eye_t<<0.06197284625670423,-1.134581509723617,0.3029004003996792;
    base2eye_q=base2eye_r;
    hand2tool0_t<<0,0,-0.25;
    obj2hand_t<<0,0,-0.32;
}

//发布末端速度
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
void cmd_tool_stoop_pub(){
    geometry_msgs::Twist vel;
    vel.linear.x=0;
    vel.linear.y=0;
    vel.linear.z=0;
    vel.angular.x=0;
    vel.angular.y=0;
    vel.angular.z=0;
    tool_vel_pub.publish(vel);
}


void robot_target_subCB(const gpf::obj_tool_transform &transform_)
{
    //目标物在相机坐标系下的坐标转机器人坐标系下的坐标
    //计时
    //timeval time_rec,time_end;
    //gettimeofday(&time_rec,NULL);
    //double time_rec_sec=time_rec.tv_sec+time_rec.tv_usec/1000000.0;
    ////clock_t time_rec=clock();
    ////double time_rec_sec=double(time_rec)/ CLOCKS_PER_SEC;
    //std::cout<<"time receive msg: "<<(time_rec.tv_usec-transform_.time_pub_sec_msg)/1000000.0<<std::endl;
    //std::cout<<"time of sleep: "<<time_rec_sec - time_old_sec<<std::endl;

    Eigen::Vector3d eye_center3d, bTtd;//相机坐标系下目标物的位置，基座标系下目标物的位置
    eye_center3d(0)=transform_.cam2obj.translation.x;
    eye_center3d(1)=transform_.cam2obj.translation.y;
    eye_center3d(2)=transform_.cam2obj.translation.z;

    bTtd=base2eye_r*eye_center3d+base2eye_t;//目标转换到基座标系下
    Eigen::Quaterniond bRtd_q(transform_.cam2obj.rotation.w, transform_.cam2obj.rotation.x, transform_.cam2obj.rotation.y, transform_.cam2obj.rotation.z);//eye2obj
    bRtd_q=base2eye_q*bRtd_q;//基座标系下目标的姿态
    Eigen::Vector3d bTtd_track;
    bTtd_track=bRtd_q*hand2tool0_t+bTtd;//考虑手抓偏移
    //bTtd=bRtd_q*hand2tool0_t+bTtd;//考虑手抓偏移

    //std::cout<<"bTtd_track "<<bTtd_track(0)<<" "<<bTtd_track(1)<<" "<<bTtd_track(2)<<" "<<std::endl;
    //std::cout<<"bRtd_q "<<bRtd_q.x()<<" "<<bRtd_q.y()<<" "<<bRtd_q.z()<<" "<<bRtd_q.w()<<" "<<std::endl;

    //机械臂末端坐标系tool0在基座标系base下的位姿
    Eigen::Vector3d bTt;
    bTt(0)=transform_.base2tool0.translation.x;
    bTt(1)=transform_.base2tool0.translation.y;
    bTt(2)=transform_.base2tool0.translation.z;
    Eigen::Quaterniond bRt_q(transform_.base2tool0.rotation.w, transform_.base2tool0.rotation.x, transform_.base2tool0.rotation.y, transform_.base2tool0.rotation.z);
    //std::cout<<"bTt "<<bTt(0)<<" "<<bTt(1)<<" "<<bTt(2)<<" "<<std::endl;
    //std::cout<<"bRt_q "<<bRt_q.x()<<" "<<bRt_q.y()<<" "<<bRt_q.z()<<" "<<bRt_q.w()<<" "<<std::endl;
    //转换到末端理想位姿坐标系下
    Eigen::Quaterniond tdRb=bRtd_q.inverse();
    Eigen::Quaterniond tRtd_q=bRt_q.inverse()*bRtd_q;
    Eigen::Vector3d tdTt=tdRb*bTt-tdRb*bTtd_track;
    //std::cout<<"tdTt"<<tdTt<<std::endl;
    //轴角
    Eigen::AngleAxisd tdRt_tu(tRtd_q.inverse());
    //std::cout<<"tdRt_tu angle"<<tdRt_tu.angle()<<std::endl;
    //std::cout<<"tdRt_tu axis"<<tdRt_tu.axis()<<std::endl;
    //opencv
    Eigen::Matrix3d tdRt_m(tRtd_q.inverse());
    cv::Mat tdRt_mat(3,3,CV_32F);
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++){
            tdRt_mat.at<float>(i,j)=tdRt_m(i,j);
        }
    cv::Mat tdRt_tu_m;
    cv::Rodrigues(tdRt_mat,tdRt_tu_m);
    //std::cout<<"tdRt_tu_m"<<tdRt_tu_m<<std::endl;
	

    //速度计算,换到基座标系下
    //轴角
    //Eigen::Vector3d delta_xyz=tRtd_q*tdTt;
    //static Eigen::Vector3d delta_xyz_old=delta_xyz;//用于微分
    //static Eigen::Vector3d delta_xyz_i(0,0,0); //积分项
    //delta_xyz_i+=delta_xyz;



    //opencv
    Eigen::Vector3d tdRt_tu_opencv(0,0,0);
    tdRt_tu_opencv(0)=tdRt_tu_m.at<float>(0,0);
    tdRt_tu_opencv(1)=tdRt_tu_m.at<float>(0,1);
    tdRt_tu_opencv(2)=tdRt_tu_m.at<float>(0,2);
    //static Eigen::Vector3d tdRt_tu_opencv_old=tdRt_tu_opencv;//用于微分
    //static Eigen::Vector3d tdRt_tu_opencv_i;//积分项
    //tdRt_tu_opencv_i+=tdRt_tu_opencv;
    //std::cout<<"tdRt_tu_opencv: "<<tdRt_tu_opencv<<std::endl;

    /*****误差计算*****/
    double err=sqrt(tdTt(0)*tdTt(0)+tdTt(1)*tdTt(1)+tdTt(2)*tdTt(2)+
                    tdRt_tu_opencv(0)*tdRt_tu_opencv(0)+tdRt_tu_opencv(1)*tdRt_tu_opencv(1)+tdRt_tu_opencv(2)*tdRt_tu_opencv(2));
    if(err<0.04){
        std::cout<<"error: "<<err<<std::endl;
        std::cout<<"tdTt: "<<tdTt<<std::endl;
        std::cout<<"tdRt_tu_opencv: "<<tdRt_tu_opencv<<std::endl;
        cmd_tool_stoop_pub();
        Eigen::Vector3d bTtd_grab;
        bTtd_grab=bRtd_q*obj2hand_t+bTtd;//考虑手抓偏移
        geometry_msgs::Pose grabPose;
        grabPose.position.x=bTtd_grab(0);
        grabPose.position.y=bTtd_grab(1);
        grabPose.position.z=bTtd_grab(2);
        grabPose.orientation.x=bRtd_q.x();
        grabPose.orientation.y=bRtd_q.y();
        grabPose.orientation.z=bRtd_q.z();
        grabPose.orientation.w=bRtd_q.w();
        //tool_pos_pub.publish(grabPose);
        ros::Duration(0.1).sleep();
        exit(0);
    }


    /*****速度计算*****/
    Eigen::Vector3d v=-kp*(tRtd_q*tdTt);//-kp*ti*delta_xyz_i-kp*td*(delta_xyz-delta_xyz_old);
    //delta_xyz_old=delta_xyz;

    Eigen::Vector3d w_opencv=-kp*tdRt_tu_opencv;//-kp*ti*tdRt_tu_opencv_i-kp*td*(tdRt_tu_opencv-tdRt_tu_opencv_old);
    //tdRt_tu_opencv_old=tdRt_tu_opencv;


    //std::cout<<"w_opencv: "<<w_opencv<<std::endl;
    v=bRt_q*v;
    w_opencv=bRt_q*w_opencv;
    //std::cout<<"base_w_opencv: "<<w_opencv<<std::endl<<std::endl<<std::endl;

    //速度发布
    geometry_msgs::Twist toolVel;
    toolVel.linear.x=v(0);
    toolVel.linear.y=v(1);
    toolVel.linear.z=v(2);
    toolVel.angular.x=w_opencv(0);
    toolVel.angular.y=w_opencv(1);
    toolVel.angular.z=w_opencv(2);
    cmd_tool_vel_pub(toolVel);

    std::cout<<"error: "<<err<<std::endl;
    //计时
    //gettimeofday(&time_end,NULL);
    //double time_end_sec=time_end.tv_sec+time_end.tv_usec/1000000.0;
    ////clock_t time_end=clock();
    ////double time_end_sec=double(time_end)/ CLOCKS_PER_SEC;
    ////std::cout<<"time fun once: "<<time_end_sec-time_rec_sec<<std::endl;
    //std::cout<<"time fun once: "<<time_end_sec - time_rec_sec/1000000.0<<std::endl;
    //std::cout<<"time call back fun: "<<time_rec_sec-time_old_sec<<std::endl;
    //std::cout<<"time from start to end: "<<time_end_sec-double(transform_.time_cur_sec_msg)<<std::endl<<std::endl;
    //time_old_sec=time_rec_sec;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ur_grab_pbvs");

  ros::NodeHandle n;
  ros::Subscriber robot_target_sub=n.subscribe("/gpf/position",1,robot_target_subCB);
  gripperPub=n.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("/Robotiq2FGripperRobotOutput", 20);
  tool_vel_pub = n.advertise<geometry_msgs::Twist>("/ur_arm_controller/cmd_tool_vel", 1);  //ur_arm速度控制
  tool_pos_pub=n.advertise<geometry_msgs::Pose>("/ur_arm_controller/cmd_tool_pos", 1);  //ur_arm位置控制

  //tf2_ros::Buffer tfBuffer;
  //tf2_ros::TransformListener tfListener(tfBuffer);  //获取base坐标系下tool0的位姿
  ros::Duration(0.1).sleep();
  variable_init();
  initializeGripperMsg(gripperPub);//手抓初始化
  //sendGripperMsg(gripperPub,0);
  //ros::Duration(3).sleep();
  //sendGripperMsg(gripperPub,250);

  std::cout<<"grapper init"<<std::endl;

  //ros::Timer timer = n.createTimer(ros::Duration(0.1), boost::bind(&left_goal_task), true);
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  cmd_tool_stoop_pub();
  ros::Duration(0.3).sleep();
  return 0;
}
