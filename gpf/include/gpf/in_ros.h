#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/subscriber_filter.h>
#include <kinect2_bridge/kinect2_definitions.h>

#include "utils.h"
#include "ur5_base_tool.hpp"
#include <math.h>
#include <geometry_msgs/Transform.h>
#include "gpf/obj_tool_transform.h"

cv_bridge::CvImagePtr cv_ptr;
cv::Mat I_ORI;  //RGB
cv::Mat I_ORI_DEPTH; //深度图
bool get_new_I=false; //获得新图片
ros::Publisher position_publisher;
#ifdef has_ur5
    geometry_msgs::Transform base2tool0;
#endif

struct img_point{
	cv::Mat image;
	std::vector<cv::Point2d> xys;
    std::string win_name;
};


void rosinit(int argc, char** argv);
void rosimage2opencv(const sensor_msgs::ImageConstPtr& msg);//璁㈤槄鐩告満鍥惧儚锛屽苟杞崲涓簅pencv鏍煎紡
void RecognitionCallback(const sensor_msgs::ImageConstPtr image_rgb, const sensor_msgs::ImageConstPtr image_depth);
bool get_template_poly_pnts(const cv::Mat &frame, std::vector<float> &template_xs, std::vector<float> &template_ys);
void mouse(int event, int x, int y, int flags, void* param);
void pub_position(cv::Mat &_T,cv::Mat &_R);


void rosinit(int argc, char **argv){
    ros::init(argc, argv, "gpf");
    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::Image> image_rgb_sub(nh, "/kinect2/qhd/image_color_rect", 1);
    message_filters::Subscriber<sensor_msgs::Image>image_depth_sub(nh, "/kinect2/qhd/image_depth_rect", 1);
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(image_rgb_sub, image_depth_sub, 10);
    sync.registerCallback(boost::bind(&RecognitionCallback, _1, _2));

}
/*
	璁㈤槄鐩告満鍥惧儚锛屽苟杞崲涓簅pencv鏍煎紡
*/
void rosimage2opencv(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::MONO8);
	}
	catch(cv_bridge::Exception &e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	//cvtColor(cv_ptr->image,I_ORI,CV_BGR2GRAY);
	I_ORI=cv_ptr->image;
}

void RecognitionCallback(const sensor_msgs::ImageConstPtr image_rgb, const sensor_msgs::ImageConstPtr image_depth){
  	//转换ROS图像消息到opencv图像
	try
	{
		cv_ptr=cv_bridge::toCvCopy(image_rgb,sensor_msgs::image_encodings::MONO8);
   		I_ORI_DEPTH = cv_bridge::toCvShare(image_depth)->image;
	}
	catch(cv_bridge::Exception &e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
    I_ORI=cv_ptr->image;
#ifdef has_ur5
    get_new_I=getPose(tfBuffer,base2tool0);
#else
    get_new_I=true;
#endif

  	//I_ORI = cv_bridge::toCvShare(image_rgb,"mono8")->image;
  	//I_ORI_DEPTH = cv_bridge::toCvShare(image_depth)->image;
}

bool get_template_poly_pnts(const cv::Mat &frame, std::vector<float> &template_xs, std::vector<float> &template_ys, int &far_point_, int &near_point_){
    std::string win_name = "get_roi";
	cv::namedWindow(win_name, 1);
	cv::imshow(win_name, I_ORI);
	img_point image_xys;
	image_xys.image=frame.clone();
    image_xys.win_name=win_name;	
	cv::setMouseCallback(win_name, &mouse, &image_xys);
	cv::waitKey(0);
	if(image_xys.xys.size()!=4){
		std::cout<<"selected points not 4"<<std::endl;
		return false;
	}
	for(int i=0;i<image_xys.xys.size();i++){
		template_xs.push_back(image_xys.xys[i].x);
		template_ys.push_back(image_xys.xys[i].y);
	}

	far_point_=1;	//假设第一个点为长轴u
	near_point_=3;

	int u_x=template_xs[far_point_]-template_xs[0];
    int u_y=template_ys[far_point_]-template_ys[0];
	int v_x=template_xs[near_point_]-template_xs[0];
	int v_y=template_ys[near_point_]-template_ys[0];

	float L1=sqrt(pow(u_x,2)+pow(u_y,2));
	float L2=sqrt(pow(v_x,2)+pow(v_y,2));
	int angle_uv;	//用于判断长轴到短轴的夹角正负
	if(L1>L2){		//假设正确，u为长轴
		angle_uv=u_x*v_y-u_y*v_x;	//向量叉乘，求判断长轴到短轴的夹角正负
	}
	else{
		far_point_=3;
		near_point_=1;
		angle_uv=v_x*u_y-v_y*u_x;
	}

	if(angle_uv<0){	//夹角为负
		near_point_=-near_point_;
	}
	
    //std::cout<<"far_point_: "<<far_point_<<std::endl;
    //std::cout<<"near_point_: "<<near_point_<<std::endl;
	

    cv::destroyWindow(win_name);
	return true;
}

void mouse(int event, int x, int y, int flags, void* param){
	img_point* image_xys_ = (img_point*)param;
    if (event == CV_EVENT_LBUTTONDOWN)
	{
		cv::Point2d p(x,y);
		image_xys_->xys.push_back(p);
		cv::circle(image_xys_->image,p,3,cv::Scalar(255,0,0),-1);
		cv::imshow(image_xys_->win_name,image_xys_->image);
		cv::waitKey(1);
	}
	
}

//发布相机坐标系到目标无坐标系的变换关系
void pub_position(cv::Mat &_T,cv::Mat &_R){
    geometry_msgs::Transform position;
    position.translation.x=_T.at<float>(0,0);
    position.translation.y=_T.at<float>(0,1);
    position.translation.z=_T.at<float>(0,2);

    Eigen::Matrix3d r;
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            r(i,j)=_R.at<float>(i,j);
        }
    }
    Eigen::Quaterniond quate_r(r);
    position.rotation.x=quate_r.x();
    position.rotation.y=quate_r.y();
    position.rotation.z=quate_r.z();
    position.rotation.w=quate_r.w();
    position_publisher.publish(position);
}
//发布相机坐标系到目标无坐标系的变换关系和机器人基座标系到末端坐标系的变换关系
void pub_position(cv::Mat &_T,cv::Mat &_R,geometry_msgs::Transform &base2tool0_){
    gpf::obj_tool_transform pub_msg;
    geometry_msgs::Transform position;
    pub_msg.cam2obj.translation.x=_T.at<float>(0,0);
    pub_msg.cam2obj.translation.y=_T.at<float>(0,1);
    pub_msg.cam2obj.translation.z=_T.at<float>(0,2);

    Eigen::Matrix3d r;
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            r(i,j)=_R.at<float>(i,j);
        }
    }
    Eigen::Quaterniond quate_r(r);
    pub_msg.cam2obj.rotation.x=quate_r.x();
    pub_msg.cam2obj.rotation.y=quate_r.y();
    pub_msg.cam2obj.rotation.z=quate_r.z();
    pub_msg.cam2obj.rotation.w=quate_r.w();

    pub_msg.base2tool0=base2tool0_;
    position_publisher.publish(pub_msg);
}

