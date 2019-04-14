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
#include "ur_arm/Joints.h"
#include <fstream>

const float deg2rad = M_PI / 180.0;//用于将角度转化为弧度。rad = deg*deg2rad

ros::Publisher gripperPub;
ros::Publisher tool_vel_pub;
ros::Publisher tool_pos_pub;
ros::Publisher joint_pos_pub;

//手眼关系
Eigen::Matrix3d base2eye_r;
Eigen::Vector3d base2eye_t;
Eigen::Quaterniond base2eye_q;

Eigen::Vector3d hand2tool0_t;//跟踪时手抓偏移
Eigen::Vector3d tool02hand_t;
Eigen::Vector3d obj2hand_t;//抓取时手抓偏移

//pbvs速度系数
float kp_track=3.0;
float kp_grab=1.5;
int times=0;
double deltaTime=0.0;
//float ti=0.0;
//float td=0.0;
//计时
//double time_old_sec=0.0;
tf2_ros::Buffer tfBuffer;
geometry_msgs::Transform base2tool0;

std::ofstream targetPosOut;//目标位姿
std::ofstream handPosOut;//手抓位姿
std::ofstream errOut;//总体误差
std::ofstream errxyzqOut;//各项误差
std::ofstream targetVelOut;//目标速度
std::ofstream toolVelOut;//手抓速度
//保存数据变量
Eigen::Vector3d bTtd;//基座标系下目标物的位置
Eigen::Quaterniond bRtd_q;//基座标系下目标物的姿态
Eigen::Vector3d bTt;//基座标系下末端的位置
Eigen::Quaterniond bRt_q;//基座标系下末端的姿态
Eigen::Vector3d tdTt;//目标位置到当前位置的误差
Eigen::Vector3d tdRt_tu_opencv;//目标姿态到当前姿态的误差
Eigen::Vector3d targetVelt;//基座标系下目标物线速度
Eigen::Vector3d targetVelr;//基座标系下目标物角速度

float errVal=0.05;//抓取动作阈值


//读取当前关节位置信息
void variable_init(void)
{
    base2eye_r<<0.9991987969968049, 0.01166580553949524, -0.03828410977483826,
    0.03837739908251811, -0.5506842145529742, 0.833830960735948,
    -0.01135514508046016, -0.8346321374253974, -0.5506907079812708;

    base2eye_t<<0.03046466580335716, -1.329345536341161, 0.3244909190467186;
    base2eye_q=base2eye_r;
    hand2tool0_t<<0,0,-0.25;
    tool02hand_t<<0,0,0.25;
    obj2hand_t<<0,0,-0.13;
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
    Eigen::Vector3d eye_center3d;//相机坐标系下目标物的位置，基座标系下目标物的位置
    eye_center3d(0)=transform_.cam2obj.translation.x;
    eye_center3d(1)=transform_.cam2obj.translation.y;
    eye_center3d(2)=transform_.cam2obj.translation.z;

    bTtd=base2eye_r*eye_center3d+base2eye_t;//目标转换到基座标系下
    static Eigen::Vector3d bTtdOld=bTtd;
    /*****偏差预测*****/
    //deltaTime=(ros::Time::now()-transform_.data).toSec();
    //std::cout<<"deltaTime: "<<deltaTime<<std::endl;
    Eigen::Vector3d bTtd_pred=bTtd;//+(bTtd-bTtdOld);//下一周期位置预测

    bRtd_q.x()=transform_.cam2obj.rotation.x;
    bRtd_q.y()=transform_.cam2obj.rotation.y;
    bRtd_q.z()=transform_.cam2obj.rotation.z;
    bRtd_q.w()=transform_.cam2obj.rotation.w;
    bRtd_q=base2eye_q*bRtd_q;//基座标系下目标的姿态

    Eigen::Vector3d bTtd_track;
    if(trackFlag){
        bTtd_track=bRtd_q*hand2tool0_t+bTtd_pred;//考虑手抓偏移
    }
    else
    {
        bTtd_pred+=targetVelt*1.5;
        bTtd_track=bRtd_q*obj2hand_t+bTtd_pred;//考虑手抓偏移
    }


    //机械臂末端坐标系tool0在基座标系base下的位姿
    getPose(tfBuffer,base2tool0);
    bTt(0)=base2tool0.translation.x;
    bTt(1)=base2tool0.translation.y;
    bTt(2)=base2tool0.translation.z;

    bRt_q.x()=base2tool0.rotation.x;
    bRt_q.y()=base2tool0.rotation.y;
    bRt_q.z()=base2tool0.rotation.z;
    bRt_q.w()=base2tool0.rotation.w;
    //std::cout<<"bTt "<<bTt(0)<<" "<<bTt(1)<<" "<<bTt(2)<<" "<<std::endl;
    //std::cout<<"bRt_q "<<bRt_q.x()<<" "<<bRt_q.y()<<" "<<bRt_q.z()<<" "<<bRt_q.w()<<" "<<std::endl;
    //转换到末端理想位姿坐标系下
    Eigen::Quaterniond tdRb=bRtd_q.inverse();
    Eigen::Quaterniond tRtd_q=bRt_q.inverse()*bRtd_q;
    tdTt=tdRb*bTt-tdRb*bTtd_track;
    //std::cout<<"tdTt"<<tdTt<<std::endl;
    //轴角
    //Eigen::AngleAxisd tdRt_tu(tRtd_q.inverse());
    //std::cout<<"tdRt_tu angle"<<tdRt_tu.angle()<<std::endl;
    //std::cout<<"tdRt_tu axis"<<tdRt_tu.axis()<<std::endl;
    //opencv
    Eigen::Matrix3d tdRt_m(tRtd_q.inverse());
    Eigen::Matrix3d bRtd_m(bRtd_q);
    cv::Mat tdRt_mat(3,3,CV_32F);
    cv::Mat bRtd_mat(3,3,CV_32F);
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++){
            tdRt_mat.at<float>(i,j)=tdRt_m(i,j);
            bRtd_mat.at<float>(i,j)=bRtd_m(i,j);
        }
    cv::Mat tdRt_tu_m;
    cv::Mat bRtd_tu_m;
    cv::Rodrigues(tdRt_mat,tdRt_tu_m);
    cv::Rodrigues(bRtd_mat,bRtd_tu_m);
    //std::cout<<"tdRt_tu_m"<<tdRt_tu_m<<std::endl;

    //opencv
    tdRt_tu_opencv(0)=tdRt_tu_m.at<float>(0,0);
    tdRt_tu_opencv(1)=tdRt_tu_m.at<float>(0,1);
    tdRt_tu_opencv(2)=tdRt_tu_m.at<float>(0,2);
    Eigen::Vector3d bRtd_tu_opencv;
    bRtd_tu_opencv(0)=bRtd_tu_m.at<float>(0,0);
    bRtd_tu_opencv(1)=bRtd_tu_m.at<float>(0,1);
    bRtd_tu_opencv(2)=bRtd_tu_m.at<float>(0,2);
    static Eigen::Vector3d bRtd_tu_opencvOld=bRtd_tu_opencv;
    std::cout<<"bRtd_tu_opencv: "<<bRtd_tu_opencv<<std::endl;


    static double timeOld=transform_.data.toSec();

    if(trackFlag){
        times++;
        if(times==5){
            deltaTime=transform_.data.toSec()-timeOld;
            targetVelt=(bTtd-bTtdOld)/deltaTime;
            std::cout<<"targetVelt: "<<targetVelt<<std::endl;   //线速度
            targetVelr=(bRtd_tu_opencv-bRtd_tu_opencvOld)/deltaTime;
            bTtdOld=bTtd;
            bRtd_tu_opencvOld=bRtd_tu_opencv;
            timeOld=transform_.data.toSec();
        }
    }
    //位置控制，保证Z轴高度，不打物品放置平面
    if(bTt(2)<0)
    {
        cmd_tool_stoop_pub();
        std::cout<<"exit bTt Z: "<<bTt(2)<<std::endl;
        ros::Duration(0.1).sleep();
        exit(0);
    }

    /*****误差计算*****/
    err_xyz_2=tdTt(0)*tdTt(0)+tdTt(1)*tdTt(1)+tdTt(2)*tdTt(2);
    err=sqrt(err_xyz_2+
                    tdRt_tu_opencv(0)*tdRt_tu_opencv(0)+tdRt_tu_opencv(1)*tdRt_tu_opencv(1)+tdRt_tu_opencv(2)*tdRt_tu_opencv(2));
    if(sqrt(err_xyz_2)>0.8){
        cmd_tool_stoop_pub();
        std::cout<<"exit error: "<<err<<std::endl;
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

    if(err>=errVal){
        cmd_tool_vel_pub(toolVel);
    }
    std::cout<<"err: "<<err<<std::endl;

    if(trackFlag){
        //保存数据
        double timeNow=ros::Time::now().toSec();
        static double timeStart=timeNow;
        double time=(timeNow-timeStart)*1000;
        //保存目标物在基座标下的姿态
        Eigen::Matrix3d bRtd_mat=bRtd_q.matrix();
        Eigen::Matrix4d targetPos=Eigen::MatrixXd::Zero(4, 4);
        for(int i=0; i<3;i++)
            for(int j=0;j<3;j++)
            {
                targetPos(i,j)=bRtd_mat(i,j);
            }
        for(int i=0;i<3;i++)
        {
            targetPos(i,3)=bTtd(i);
        }
        targetPos(3,3)=1.0;
        targetPosOut<<targetPos<<std::endl;
        //保存机械臂末端姿态
        Eigen::Matrix3d bRt_mat=bRt_q.matrix();
        Eigen::Vector3d bThand=bRt_mat*tool02hand_t+bTt;
        Eigen::Matrix4d handPos=Eigen::MatrixXd::Zero(4, 4);
        for(int i=0; i<3;i++)
            for(int j=0;j<3;j++)
            {
                handPos(i,j)=bRt_mat(i,j);
            }
        for(int i=0;i<3;i++)
        {
            handPos(i,3)=bThand(i);
        }
        handPos(3,3)=1.0;
        handPosOut<<handPos<<std::endl;
        //保存总体误差
        errOut<<time<<" "<<err<<std::endl;
        //保存6个自由度误差
        errxyzqOut<<time<<" "<<tdTt(0)<<" "<<tdTt(1)<<" "<<tdTt(2)<<" ";
        errxyzqOut<<tdRt_tu_opencv(0)<<" "<<tdRt_tu_opencv(1)<<" "<<tdRt_tu_opencv(2)<<std::endl;
        //目标物速度
        if(times==5){
            times=1;
            targetVelOut<<time<<" "<<targetVelt(0)<<" "<<targetVelt(1)<<" "<<targetVelt(2)<<" ";
            targetVelOut<<targetVelr(0)<<" "<<targetVelr(1)<<" "<<targetVelr(2)<<std::endl;
        }
        //手抓速度
        toolVelOut<<time<<" "<<toolVel.linear.x<<" "<<toolVel.linear.y<<" "<<toolVel.linear.z<<" ";
        toolVelOut<<toolVel.angular.x<<" "<<toolVel.angular.y<<" "<<toolVel.angular.z<<std::endl;
    }

}



void robot_target_subCB(const gpf::obj_tool_transform &transform_)
{
    /*****误差计算*****/
    double err_xyz_2;
    double err;
    geometry_msgs::Twist toolVel;
    pbvsGetVel(transform_, err_xyz_2, err,toolVel);

    if(err<errVal){
        pbvsGetVel(transform_, err_xyz_2, err,toolVel,0);
        cmd_tool_vel_pub(toolVel);
        ros::Duration(0.05).sleep();
        int grapFlag=0;
        while(err>0.015 && ros::ok()){
            if(grapFlag==0 && err_xyz_2<0.02){
                sendGripperMsg(gripperPub,65);
                grapFlag=1;
            }
            pbvsGetVel(transform_, err_xyz_2, err,toolVel,0);
            cmd_tool_vel_pub(toolVel);
            ros::Duration(0.05).sleep();
        }

        //cmd_tool_stoop_pub();
        std::cout<<"success"<<std::endl;


        //ros::Duration(0.1).sleep();
        geometry_msgs::Twist toolVel;
        toolVel.linear.x=0;
        toolVel.linear.y=0;
        toolVel.linear.z=0.3;
        toolVel.angular.x=0;
        toolVel.angular.y=0;
        toolVel.angular.z=0;
        cmd_tool_vel_pub(toolVel,1);

        ur_arm::Joints jointPos;   //关节角度值
        double p[6]={-92,-121,-54,-96,90,-180};  //关节角度值
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


        exit(0);
    }



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
  joint_pos_pub = n.advertise<ur_arm::Joints>("/ur_arm_controller/cmd_joint_pos", 1);      //关节角度控制

  tf2_ros::TransformListener tfListener(tfBuffer);  //获取base坐标系下tool0的位姿
  ros::Duration(0.2).sleep();
  variable_init();

  ur_arm::Joints jointPos;   //关节角度值
  double p[6]={-92,-121,-54,-96,90,-180};  //关节角度值-92,-120,-73,-76,90,-180  //-92,-121,-54,-96,90,-180
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

  initializeGripperMsg(gripperPub);//手抓初始化

  std::cout<<"grapper init"<<std::endl;
  //保存数据文件

  targetPosOut=std::ofstream("/home/qcrong/Documents/thesis/trackGrapOut/tragetPos.txt");
  if(!targetPosOut){
      std::cout<<"open tragetPos faild"<<std::endl;
      exit(-1);
  }
  handPosOut=std::ofstream("/home/qcrong/Documents/thesis/trackGrapOut/handPos.txt");
  if(!handPosOut){
      std::cout<<"open handPosOut faild"<<std::endl;
      exit(-1);
  }
  errOut=std::ofstream("/home/qcrong/Documents/thesis/trackGrapOut/err.txt");
  if(!errOut){
      std::cout<<"open errOut faild"<<std::endl;
      exit(-1);
  }
  errxyzqOut=std::ofstream("/home/qcrong/Documents/thesis/trackGrapOut/errxyzq.txt");
  if(!errxyzqOut){
      std::cout<<"open errxyzq faild"<<std::endl;
      exit(-1);
  }
  targetVelOut=std::ofstream("/home/qcrong/Documents/thesis/trackGrapOut/targetVel.txt");
  if(!targetVelOut){
      std::cout<<"open targetVel faild"<<std::endl;
      exit(-1);
  }
  toolVelOut=std::ofstream("/home/qcrong/Documents/thesis/trackGrapOut/toolVel.txt");
  if(!toolVelOut){
      std::cout<<"open toolVel faild"<<std::endl;
      exit(-1);
  }


  //ros::Timer timer = n.createTimer(ros::Duration(0.1), boost::bind(&left_goal_task), true);
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  cmd_tool_stoop_pub();
  targetPosOut.close();

  handPosOut.close();
  errOut.close();
  errxyzqOut.close();
  targetVelOut.close();
  toolVelOut.close();

  ros::Duration(0.3).sleep();
  return 0;
}
