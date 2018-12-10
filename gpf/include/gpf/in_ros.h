#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "utils.h"

cv_bridge::CvImagePtr cv_ptr;
cv::Mat I_ORI;  //输入原始灰度图像

struct img_point{
	cv::Mat image;
	std::vector<cv::Point2d> xys;
    char* win_name;
};

void rosimage2opencv(const sensor_msgs::ImageConstPtr& msg);//订阅相机图像，并转换为opencv格式
void get_template_poly_pnts(const cv::Mat &frame, std::vector<float> &template_xs, std::vector<float> &template_ys);
void mouse(int event, int x, int y, int flags, void* param);

/*
	订阅相机图像，并转换为opencv格式
*/
void rosimage2opencv(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
	}
	catch(cv_bridge::Exception &e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	cvtColor(cv_ptr->image,I_ORI,CV_BGR2GRAY);
	//I_ORI=cv_ptr->image;
}

void get_template_poly_pnts(const cv::Mat &frame, std::vector<float> &template_xs, std::vector<float> &template_ys){
	char* win_name = "get_roi";
	cv::namedWindow(win_name, 1);
	cv::imshow(win_name, I_ORI);
	img_point image_xys;
	image_xys.image=frame.clone();
    image_xys.win_name=win_name;	
	cv::setMouseCallback(win_name, &mouse, &image_xys);
	cv::waitKey(0);
	for(int i=0;i<image_xys.xys.size();i++){
		template_xs.push_back(image_xys.xys[i].x);
		template_ys.push_back(image_xys.xys[i].y);
	}
	cvDestroyWindow(win_name);
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
