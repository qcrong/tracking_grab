#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <stdio.h>
#include <sys/stat.h>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

  public:
  ImageConverter()
  : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/kinect2/hd/image_color_rect", 1, &ImageConverter::imageCb, this);

    cv::namedWindow(OPENCV_WINDOW);
    image_count = 0;
    image_path=std::string("/home/qcrong/Pictures/images_save_2");
    path_name=image_path+std::string("/test_%d.png");
	if(mkdir(image_path.c_str(),S_IRWXU)!=0){ //创建记录文件夹，可读可写可执行权限
      std::cout<<"图像保存文件夹创建失败"<<std::endl;
	  exit(-1);
	}
    max_count=200;
    cv::waitKey(2000);
   }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    std::cout<<++image_count<<std::endl;
    char fn_out[128];
    snprintf(fn_out,128,path_name.c_str(),image_count);
    cv::imwrite(fn_out,cv_ptr->image);
    if(image_count>=max_count)
	{
	  std::cout<<"图片保存完成"<<std::endl;
	  exit(1);
    }
  }

  private:
  int image_count;	//图像计数
  std::string image_path;		//保存图像的路径
  std::string path_name;
  int max_count;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
