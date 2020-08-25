#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Transform.h>
#include <Eigen/Eigen>

#define has_ur5 //是否链接ur5


#ifdef has_ur5
tf2_ros::Buffer tfBuffer;
#endif

//获取基座标系base下末端坐标系tool0的位置pose_t和姿态pose_r（旋转矩阵）
bool getPose(tf2_ros::Buffer &tfBuffer_,Eigen::Vector3d &pose_t, Eigen::Matrix3d &pose_r)
{
   geometry_msgs::TransformStamped transformStamped;
   try
   {
     transformStamped = tfBuffer_.lookupTransform( "base", "tool0", ros::Time(0),ros::Duration(0.1));
   }
   catch (tf2::TransformException &ex)
   {
     ROS_WARN("%s",ex.what());
     return false;
   }

   Eigen::Quaterniond rotQ(transformStamped.transform.rotation.w,
                   transformStamped.transform.rotation.x,
                   transformStamped.transform.rotation.y,
                   transformStamped.transform.rotation.z);

   pose_r=rotQ.matrix();

   pose_t<<transformStamped.transform.translation.x,
            transformStamped.transform.translation.y,
            transformStamped.transform.translation.z;

   return true;
}
bool getPose(tf2_ros::Buffer &tfBuffer_,geometry_msgs::Transform &t_)
{
   geometry_msgs::TransformStamped transformStamped;
   try
   {
     transformStamped = tfBuffer_.lookupTransform( "base", "tool0", ros::Time(0),ros::Duration(0.1));
   }
   catch (tf2::TransformException &ex)
   {
     ROS_WARN("%s",ex.what());
     return false;
   }

   t_=transformStamped.transform;
   return true;

}
