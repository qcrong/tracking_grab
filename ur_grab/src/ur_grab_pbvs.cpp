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
#include "ur_grab_pbvs/ur5_base_tool.hpp"

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
float kp_track=3;
float kp_grab=3.6;
Eigen::Vector3d vT;
//float ti=0.0;
//float td=0.0;
//计时
//double time_old_sec=0.0;
tf2_ros::Buffer tfBuffer;
geometry_msgs::Transform base2tool0;


//读取当前关节位置信息
void variable_init(void)
{
    base2eye_r<<0.9993968249877675, 0.01712990233798108, -0.0302084863947243,
    0.03463776186797156, -0.5541297160089593, 0.8317093742940544,
    -0.00249231963172894, -0.8322540623401736, -0.5543888202887322;


    base2eye_t<<0.03969047837898311, -1.204680720943689,0.3279361299516342;
    base2eye_q=base2eye_r;
    hand2tool0_t<<0,0,-0.25;
    obj2hand_t<<0,0,-0.125;
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

void pbvsGetVel(const gpf::obj_tool_transform &transform_,double& err_xyz_2,double& err,geometry_msgs::Twist& toolVel,bool trackFlag=1)
{
    Eigen::Vector3d eye_center3d, bTtd;//相机坐标系下目标物的位置，基座标系下目标物的位置
    eye_center3d(0)=transform_.cam2obj.translation.x;
    eye_center3d(1)=transform_.cam2obj.translation.y;
    eye_center3d(2)=transform_.cam2obj.translation.z;

    bTtd=base2eye_r*eye_center3d+base2eye_t;//目标转换到基座标系下
    /*****偏差预测*****/
    double deltaTime=(ros::Time::now()-transform_.data).toSec();
    std::cout<<"deltaTime: "<<deltaTime<<std::endl;
    static Eigen::Vector3d bTtdOld=bTtd;
    Eigen::Vector3d bTtd_pred=bTtd+(bTtd-bTtdOld);//下一周期位置预测


    Eigen::Quaterniond bRtd_q(transform_.cam2obj.rotation.w, transform_.cam2obj.rotation.x, transform_.cam2obj.rotation.y, transform_.cam2obj.rotation.z);//eye2obj
    bRtd_q=base2eye_q*bRtd_q;//基座标系下目标的姿态
    Eigen::Vector3d bTtd_track;
    if(trackFlag){
        vT=(bTtd-bTtdOld)/deltaTime;
        std::cout<<"vT: "<<vT<<std::endl;   //速度
        bTtdOld=bTtd;
        bTtd_track=bRtd_q*hand2tool0_t+bTtd_pred;//考虑手抓偏移
    }
    else
    {
        bTtd_pred+=vT*1.5;
        bTtd_track=bRtd_q*obj2hand_t+bTtd_pred;//考虑手抓偏移
    }


    //机械臂末端坐标系tool0在基座标系base下的位姿
    getPose(tfBuffer,base2tool0);

    Eigen::Vector3d bTt;
    bTt(0)=base2tool0.translation.x;
    bTt(1)=base2tool0.translation.y;
    bTt(2)=base2tool0.translation.z;

    Eigen::Quaterniond bRt_q(base2tool0.rotation.w, base2tool0.rotation.x, base2tool0.rotation.y, base2tool0.rotation.z);
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
    std::cout<<"tdRt_tu_m"<<tdRt_tu_m<<std::endl;

    //opencv
    Eigen::Vector3d tdRt_tu_opencv(0,0,0);
    tdRt_tu_opencv(0)=tdRt_tu_m.at<float>(0,0);
    tdRt_tu_opencv(1)=tdRt_tu_m.at<float>(0,1);
    tdRt_tu_opencv(2)=tdRt_tu_m.at<float>(0,2);

    //位置控制，保证Z轴高度，不打物品放置平面
    if(bTt(2)<0.012)
    {
        cmd_tool_stoop_pub();
        std::cout<<"bTt Z: "<<bTt(2)<<std::endl;
        ros::Duration(0.1).sleep();
        exit(0);
    }

    /*****误差计算*****/
    err_xyz_2=tdTt(0)*tdTt(0)+tdTt(1)*tdTt(1)+tdTt(2)*tdTt(2);
    err=sqrt(err_xyz_2+
                    tdRt_tu_opencv(0)*tdRt_tu_opencv(0)+tdRt_tu_opencv(1)*tdRt_tu_opencv(1)+tdRt_tu_opencv(2)*tdRt_tu_opencv(2));
    if(sqrt(err_xyz_2)>0.8){
        cmd_tool_stoop_pub();
        std::cout<<"error: "<<err<<std::endl;
        ros::Duration(0.1).sleep();
        exit(0);
    }
    /*****速度计算*****/
    Eigen::Vector3d v;
    Eigen::Vector3d w_opencv;
    if(trackFlag){
        v=-kp_track*(tRtd_q*tdTt);
        w_opencv=-kp_track*tdRt_tu_opencv;
    }
    else{
        v=-kp_grab*(tRtd_q*tdTt);
        w_opencv=-kp_grab*tdRt_tu_opencv;
    }

    //std::cout<<"w_opencv: "<<w_opencv<<std::endl;
    v=bRt_q*v;
    w_opencv=bRt_q*w_opencv;
    //std::cout<<"base_w_opencv: "<<w_opencv<<std::endl<<std::endl<<std::endl;
    //速度赋值
    toolVel.linear.x=v(0);
    toolVel.linear.y=v(1);
    toolVel.linear.z=v(2);
    toolVel.angular.x=w_opencv(0);
    toolVel.angular.y=w_opencv(1);
    toolVel.angular.z=w_opencv(2);
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

//    Eigen::Vector3d eye_center3d, bTtd;//相机坐标系下目标物的位置，基座标系下目标物的位置
//    eye_center3d(0)=transform_.cam2obj.translation.x;
//    eye_center3d(1)=transform_.cam2obj.translation.y;
//    eye_center3d(2)=transform_.cam2obj.translation.z;

//    bTtd=base2eye_r*eye_center3d+base2eye_t;//目标转换到基座标系下
//    /*****偏差预测*****/
//    double deltaTime=(ros::Time::now()-transform_.data).toSec();
//    std::cout<<"deltaTime: "<<deltaTime<<std::endl;
//    static Eigen::Vector3d bTtdOld=bTtd;
//    Eigen::Vector3d bTtd_pred=bTtd;//+(bTtd-bTtdOld);//下一周期位置预测
//    Eigen::Vector3d vT=(bTtd-bTtdOld)/deltaTime;
//    std::cout<<"vT: "<<vT<<std::endl;   //速度
//    bTtdOld=bTtd;

//    Eigen::Quaterniond bRtd_q(transform_.cam2obj.rotation.w, transform_.cam2obj.rotation.x, transform_.cam2obj.rotation.y, transform_.cam2obj.rotation.z);//eye2obj
//    bRtd_q=base2eye_q*bRtd_q;//基座标系下目标的姿态
//    Eigen::Vector3d bTtd_track;
//    bTtd_track=bRtd_q*hand2tool0_t+bTtd_pred;//考虑手抓偏移
//    //bTtd=bRtd_q*hand2tool0_t+bTtd;//考虑手抓偏移

//    //std::cout<<"bTtd_track "<<bTtd_track(0)<<" "<<bTtd_track(1)<<" "<<bTtd_track(2)<<" "<<std::endl;
//    //std::cout<<"bRtd_q "<<bRtd_q.x()<<" "<<bRtd_q.y()<<" "<<bRtd_q.z()<<" "<<bRtd_q.w()<<" "<<std::endl;

//    //机械臂末端坐标系tool0在基座标系base下的位姿
//    getPose(tfBuffer,base2tool0);

//    Eigen::Vector3d bTt;
//    bTt(0)=base2tool0.translation.x;
//    bTt(1)=base2tool0.translation.y;
//    bTt(2)=base2tool0.translation.z;

//    Eigen::Quaterniond bRt_q(base2tool0.rotation.w, base2tool0.rotation.x, base2tool0.rotation.y, base2tool0.rotation.z);
//    //std::cout<<"bTt "<<bTt(0)<<" "<<bTt(1)<<" "<<bTt(2)<<" "<<std::endl;
//    //std::cout<<"bRt_q "<<bRt_q.x()<<" "<<bRt_q.y()<<" "<<bRt_q.z()<<" "<<bRt_q.w()<<" "<<std::endl;
//    //转换到末端理想位姿坐标系下
//    Eigen::Quaterniond tdRb=bRtd_q.inverse();
//    Eigen::Quaterniond tRtd_q=bRt_q.inverse()*bRtd_q;
//    Eigen::Vector3d tdTt=tdRb*bTt-tdRb*bTtd_track;
//    //std::cout<<"tdTt"<<tdTt<<std::endl;
//    //轴角
//    Eigen::AngleAxisd tdRt_tu(tRtd_q.inverse());
//    //std::cout<<"tdRt_tu angle"<<tdRt_tu.angle()<<std::endl;
//    //std::cout<<"tdRt_tu axis"<<tdRt_tu.axis()<<std::endl;
//    //opencv
//    Eigen::Matrix3d tdRt_m(tRtd_q.inverse());
//    cv::Mat tdRt_mat(3,3,CV_32F);
//    for(int i=0;i<3;i++)
//        for(int j=0;j<3;j++){
//            tdRt_mat.at<float>(i,j)=tdRt_m(i,j);
//        }
//    cv::Mat tdRt_tu_m;
//    cv::Rodrigues(tdRt_mat,tdRt_tu_m);
//    std::cout<<"tdRt_tu_m"<<tdRt_tu_m<<std::endl;
	

//    //速度计算,换到基座标系下
//    //轴角
//    //Eigen::Vector3d delta_xyz=tRtd_q*tdTt;
//    //static Eigen::Vector3d delta_xyz_old=delta_xyz;//用于微分
//    //static Eigen::Vector3d delta_xyz_i(0,0,0); //积分项
//    //delta_xyz_i+=delta_xyz;



//    //opencv
//    Eigen::Vector3d tdRt_tu_opencv(0,0,0);
//    tdRt_tu_opencv(0)=tdRt_tu_m.at<float>(0,0);
//    tdRt_tu_opencv(1)=tdRt_tu_m.at<float>(0,1);
//    tdRt_tu_opencv(2)=tdRt_tu_m.at<float>(0,2);
//    //static Eigen::Vector3d tdRt_tu_opencv_old=tdRt_tu_opencv;//用于微分
//    //static Eigen::Vector3d tdRt_tu_opencv_i;//积分项
//    //tdRt_tu_opencv_i+=tdRt_tu_opencv;
//    //std::cout<<"tdRt_tu_opencv: "<<tdRt_tu_opencv<<std::endl;

//    //位置控制，保证Z轴高度，不打物品放置平面
//    if(bTt(2)<0.012)
//    {
//        cmd_tool_stoop_pub();
//        std::cout<<"bTt Z: "<<bTt(2)<<std::endl;
//        ros::Duration(0.1).sleep();
//        exit(0);
//    }

    /*****误差计算*****/
    double err_xyz_2;
    double err;
    geometry_msgs::Twist toolVel;
    pbvsGetVel(transform_, err_xyz_2, err,toolVel);

    if(err<0.05){
        pbvsGetVel(transform_, err_xyz_2, err,toolVel,0);
        cmd_tool_vel_pub(toolVel);
        ros::Duration(0.05).sleep();
        while(err>0.005 && ros::ok()){
            pbvsGetVel(transform_, err_xyz_2, err,toolVel,0);
            cmd_tool_vel_pub(toolVel);
            ros::Duration(0.05).sleep();
        }
        sendGripperMsg(gripperPub,65);
        cmd_tool_stoop_pub();
        std::cout<<"success"<<std::endl;

//        Eigen::Vector3d bTtd_grab;
//        //bTtd_pred+=vT*0.7;
//        bTtd_grab=bRtd_q*obj2hand_t+bTtd_pred;//考虑手抓偏移
//        if(bTtd_grab(2)<0.015){
//            std::cout<<"bTtd_grab z: "<<bTtd_grab(2)<<"<0.015"<<std::endl;
//            cmd_tool_stoop_pub();
//            exit(0);
//        }
//        geometry_msgs::Pose grabPose;
//        grabPose.position.x=bTtd_grab(0);
//        grabPose.position.y=bTtd_grab(1);
//        grabPose.position.z=bTtd_grab(2);

////        grabPose.orientation.x=base2tool0.rotation.x;
////        grabPose.orientation.y=base2tool0.rotation.y;
////        grabPose.orientation.z=base2tool0.rotation.z;
////        grabPose.orientation.w=base2tool0.rotation.w;
//        grabPose.orientation.x=bRt_q.x();
//        grabPose.orientation.y=bRt_q.y();
//        grabPose.orientation.z=bRt_q.z();
//        grabPose.orientation.w=bRt_q.w();


//        tool_pos_pub.publish(grabPose);
//        std::cout<<"grabPose: "<<grabPose<<std::endl;
//        std::cout<<"error: "<<err<<std::endl;
//        std::cout<<"tdTt: "<<tdTt<<std::endl;
//        std::cout<<"tdRt_tu_opencv: "<<tdRt_tu_opencv<<std::endl;
//        std::cout<<"bTtd_grab: "<<bTtd_grab<<std::endl;
//        //夹爪夹紧位置
//        while(ros::ok()){
//            getPose(tfBuffer,base2tool0);
//            double delatXYZ=sqrt((bTtd_grab(0)-base2tool0.translation.x)*(bTtd_grab(0)-base2tool0.translation.x)+
//                                 (bTtd_grab(1)-base2tool0.translation.y)*(bTtd_grab(1)-base2tool0.translation.y)+
//                                 (bTtd_grab(2)-base2tool0.translation.z)*(bTtd_grab(2)-base2tool0.translation.z)
//                                 );
//            //std::cout<<"delatXYZ: "<<delatXYZ<<std::endl;
//            if(delatXYZ<0.02){
//                break;
//            }
//        }
//        sendGripperMsg(gripperPub,65);

//        grabPose.position.z+=0.3;
//        tool_pos_pub.publish(grabPose);

        ros::Duration(0.1).sleep();
        geometry_msgs::Twist toolVel;
        toolVel.linear.x=0;
        toolVel.linear.y=0;
        toolVel.linear.z=0.1;
        toolVel.angular.x=0;
        toolVel.angular.y=0;
        toolVel.angular.z=0;
        cmd_tool_vel_pub(toolVel,2);

        exit(0);
    }
//    else if(sqrt(err_xyz_2)>0.8){
//        cmd_tool_stoop_pub();
//        std::cout<<"error: "<<err<<std::endl;
//        ros::Duration(0.1).sleep();
//        exit(0);
//    }

//    /*****速度计算*****/
//    Eigen::Vector3d v=-kp*(tRtd_q*tdTt);//-kp*ti*delta_xyz_i-kp*td*(delta_xyz-delta_xyz_old);
//    //delta_xyz_old=delta_xyz;

//    Eigen::Vector3d w_opencv=-kp*tdRt_tu_opencv;//-kp*ti*tdRt_tu_opencv_i-kp*td*(tdRt_tu_opencv-tdRt_tu_opencv_old);
//    //tdRt_tu_opencv_old=tdRt_tu_opencv;


//    //std::cout<<"w_opencv: "<<w_opencv<<std::endl;
//    v=bRt_q*v;
//    w_opencv=bRt_q*w_opencv;
//    //std::cout<<"base_w_opencv: "<<w_opencv<<std::endl<<std::endl<<std::endl;

//    //速度发布
//    geometry_msgs::Twist toolVel;
//    toolVel.linear.x=v(0);
//    toolVel.linear.y=v(1);
//    toolVel.linear.z=v(2);
//    toolVel.angular.x=w_opencv(0);
//    toolVel.angular.y=w_opencv(1);
//    toolVel.angular.z=w_opencv(2);

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

  tf2_ros::TransformListener tfListener(tfBuffer);  //获取base坐标系下tool0的位姿
  ros::Duration(0.2).sleep();
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
