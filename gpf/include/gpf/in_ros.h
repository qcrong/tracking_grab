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
cv::Mat I_ORI;  //gray
cv::Mat I_ORI_DEPTH; //���ͼ
cv::Mat I_template; //ģ��ͼ
cv::Ptr<cv::xfeatures2d::SiftFeatureDetector> feature;
std::vector<cv::KeyPoint> template_keypoints;
cv::Mat template_description;//ģ����������
cv::Mat Htc;   //�任����,ģ��ͼ�񵽵�ǰͼ��
cv::Ptr<cv::xfeatures2d::SiftDescriptorExtractor> descript;
bool get_new_I=false; //�����ͼƬ

cv::Mat I_ORI_RGB;  //RGB
ros::Publisher position_publisher;
ros::Subscriber colorimg_sub;
bool get_new_RGBI=false; //�����ͼƬ

#ifdef has_ur5
    geometry_msgs::Transform base2tool0;
#endif

struct img_point{
	cv::Mat image;
	std::vector<cv::Point2d> xys;
    std::string win_name;
};


void rosinit(int argc, char** argv);
void rosimage2opencv(const sensor_msgs::ImageConstPtr& msg);//订阅相机图像，并转换为opencv格式
void RecognitionCallback(const sensor_msgs::ImageConstPtr image_rgb, const sensor_msgs::ImageConstPtr image_depth);
bool get_template_poly_pnts(std::vector<float> &template_xs, std::vector<float> &template_ys, int &far_point_, int &near_point_);//�ֶ���ȡĿ���ĸ�����
bool autoget_template_poly_pnts(const std::vector<cv::Point2f> &I_template_conners_,std::vector<float> &template_xs, std::vector<float> &template_ys, int &far_point_, int &near_point_, bool showimage);//�Զ���ȡĿ���ĸ�����
void mouse(int event, int x, int y, int flags, void* param);
void pub_position(cv::Mat &_T,cv::Mat &_R);
//�����������ϵ��Ŀ��������ϵ�ı任��ϵ�ͻ����˻�����ϵ��ĩ������ϵ�ı任��ϵ
void pub_position(cv::Mat &_T,cv::Mat &_R,geometry_msgs::Transform &base2tool0_,double t_cur,double t_pub);
bool load_template_params(const std::string &template_img_dir_,std::vector<cv::Point2f> &I_template_conners_,std::vector<cv::Point2f> &I_template_features_);//ģ���������
void colorimgSubCallback(const sensor_msgs::ImageConstPtr msg);


void rosinit(int argc, char **argv){
    ros::init(argc, argv, "gpf");
    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::Image> image_rgb_sub(nh, "/kinect2/qhd/image_color_rect", 1);
    message_filters::Subscriber<sensor_msgs::Image>image_depth_sub(nh, "/kinect2/qhd/image_depth_rect", 1);
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(image_rgb_sub, image_depth_sub, 10);
    sync.registerCallback(boost::bind(&RecognitionCallback, _1, _2));

}
/*
	订阅相机图像，并转换为opencv格式
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
  	//ת��ROSͼ����Ϣ��opencvͼ��
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

bool get_template_poly_pnts(std::vector<float> &template_xs, std::vector<float> &template_ys, int &far_point_, int &near_point_){
    std::string win_name = "get_roi";
	cv::namedWindow(win_name, 1);
	cv::imshow(win_name, I_ORI);
	img_point image_xys;
    image_xys.image=I_ORI.clone();
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

	far_point_=1;	//�����һ����Ϊ����u
	near_point_=3;

	int u_x=template_xs[far_point_]-template_xs[0];
    int u_y=template_ys[far_point_]-template_ys[0];
	int v_x=template_xs[near_point_]-template_xs[0];
	int v_y=template_ys[near_point_]-template_ys[0];

	float L1=sqrt(pow(u_x,2)+pow(u_y,2));
	float L2=sqrt(pow(v_x,2)+pow(v_y,2));
	int angle_uv;	//�����жϳ��ᵽ����ļн�����
	if(L1>L2){		//������ȷ��uΪ����
		angle_uv=u_x*v_y-u_y*v_x;	//������ˣ����жϳ��ᵽ����ļн�����
	}
	else{
		far_point_=3;
		near_point_=1;
		angle_uv=v_x*u_y-v_y*u_x;
	}

	if(angle_uv<0){	//�н�Ϊ��
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

//�����������ϵ��Ŀ��������ϵ�ı任��ϵ
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
//�����������ϵ��Ŀ��������ϵ�ı任��ϵ�ͻ����˻�����ϵ��ĩ������ϵ�ı任��ϵ,��ʱ��
void pub_position(cv::Mat &_T,cv::Mat &_R,geometry_msgs::Transform &base2tool0_,double t_cur,double t_pub){
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
    pub_msg.time_cur_sec_msg=t_cur;
    pub_msg.time_pub_sec_msg=t_pub;
    position_publisher.publish(pub_msg);
}

//�����������ϵ��Ŀ��������ϵ�ı任��ϵ�ͻ����˻�����ϵ��ĩ������ϵ�ı任��ϵ
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

//ģ���������
bool load_template_params(const std::string &template_img_dir_,std::vector<cv::Point2f> &I_template_conners_,std::vector<cv::Point2f> &I_template_features_){
    I_template=cv::imread(template_img_dir_);
    if(I_template.empty()){
        std::cout<<"template image load error"<<std::endl;
        return false;
    }
    feature = cv::xfeatures2d::SiftFeatureDetector::create();
    feature->detect(I_template,template_keypoints);
    descript = cv::xfeatures2d::SiftDescriptorExtractor::create();
    descript->compute(I_template,template_keypoints,template_description);
    /*��ʼ���ĸ��ǵ�*/
    I_template_conners_.resize(4);
    I_template_conners_[0]=cv::Point2f(0,0);
    I_template_conners_[1]=cv::Point2f(I_template.cols,0);
    I_template_conners_[2]=cv::Point2f(I_template.cols,I_template.rows);
    I_template_conners_[3]=cv::Point2f(0,I_template.rows);
    /*ģ����ȡ��Ե�����������ͣ���õ���������*/
    cv::Mat edge,gray;
    cv::cvtColor(I_template,gray,cv::COLOR_BGR2GRAY);//ģ��ͼ��ת��Ϊ�Ҷ�ͼ
    cv::blur(gray,edge,cv::Size(3,3));//����
    cv::Canny(edge,edge,20,60,3);
    //cv::imshow("edge canny",edge);
    cv::Mat element=cv::getStructuringElement(cv::MORPH_RECT,cv::Size(5,5));//��ȡ�Զ����
    cv::dilate(edge,edge,element);//���ͣ�����������
    //cv::imshow("edge dilate",edge);
    //cv::imshow("template",I_template);

    int allPoints=0;
    int featurePoints=0;

    for(int u=0;u<edge.rows;u++)
        for(int v=0;v<edge.cols;v++){
            allPoints++;
            if(edge.at<unsigned char>(u,v)==255){
                I_template_features_.push_back(cv::Point2f(v,u));
                featurePoints++;
            }
        }

    std::cout<<"allPoints: "<<allPoints<<std::endl;
    std::cout<<"featurePoints: "<<featurePoints<<std::endl;

    //cv::waitKey(0);



    //exit(-1);

    return true;
}

//�Զ���ȡĿ���ĸ�����
bool autoget_template_poly_pnts(const std::vector<cv::Point2f> &I_template_conners_,std::vector<float> &template_xs, std::vector<float> &template_ys, int &far_point_, int &near_point_, bool showimage=0){
    while(get_new_RGBI==false && ros::ok()){
        ros::spinOnce();
    }
    get_new_RGBI=false;
    //��ȡ������
    std::vector<cv::KeyPoint> curimg_keypoints;
    feature->detect(I_ORI_RGB,curimg_keypoints);
    //��ȡ��������
    cv::Mat curimg_description;
    descript->compute(I_ORI_RGB,curimg_keypoints,curimg_description);
    //�������������ƥ��
    std::vector<cv::DMatch> matches;
    cv::FlannBasedMatcher matcher;
    matcher.match(template_description, curimg_description, matches);    
    /************************************************************************/
    /* RANSAC��ȡ��Ӧ�Ծ���ȥ����ƥ�� */
    /************************************************************************/
    /*����ƥ����Ŷ�*/
    if(matches.size()>10){
        std::vector<int> queryIdxs(matches.size()), trainIdxs(matches.size());
        for( int i = 0; i < matches.size(); i++ )
        {
            queryIdxs[i] = matches[i].queryIdx;
            trainIdxs[i] = matches[i].trainIdx;
        }

        std::vector<cv::Point2f> points_t;
        cv::KeyPoint::convert(template_keypoints, points_t, queryIdxs);
        std::vector<cv::Point2f> points_c;
        cv::KeyPoint::convert(curimg_keypoints, points_c, trainIdxs);
        int ransacReprojThreshold = 2;  //�ܾ���ֵ

        Htc = cv::findHomography( cv::Mat(points_t), cv::Mat(points_c), CV_RANSAC, ransacReprojThreshold );
        std::vector<char> matchesMask( matches.size(), 0 );
        cv::Mat points_t_t;
        cv::perspectiveTransform(cv::Mat(points_t), points_t_t, Htc);
        int n_good_matchs=0;
        for(int i = 0; i < points_t.size(); i++)  //���桮�ڵ㡯
        {
            if( cv::norm(points_c[i] - points_t_t.at<cv::Point2f>(i,0)) <= ransacReprojThreshold) //���ڵ������
            {
                matchesMask[i] = 1;
                n_good_matchs++;
            }
        }
        if(n_good_matchs>=8){
            std::cout<<"numbers of good matchs: "<<n_good_matchs<<std::endl;
            colorimg_sub.shutdown();//ȡ����ͼ����
            /*ӳ���õ�ǰͼ����Ŀ������ĸ�����*/
            cv::Mat cur_conners;
            cv::perspectiveTransform(cv::Mat(I_template_conners_), cur_conners, Htc);
            template_xs.resize(4);
            template_ys.resize(4);
            for(int i=0;i<I_template_conners_.size();i++){
                template_xs[i]=cur_conners.at<float>(i,0);
                template_ys[i]=cur_conners.at<float>(i,1);
            }
            /*ȷ��������ͼн�����*/
            far_point_=1;	//�����һ����Ϊ����u
            near_point_=3;

            int u_x=template_xs[far_point_]-template_xs[0];
            int u_y=template_ys[far_point_]-template_ys[0];
            int v_x=template_xs[near_point_]-template_xs[0];
            int v_y=template_ys[near_point_]-template_ys[0];

            float L1=sqrt(pow(u_x,2)+pow(u_y,2));
            float L2=sqrt(pow(v_x,2)+pow(v_y,2));
            int angle_uv;	//�����жϳ��ᵽ����ļн�����
            if(L1>L2){		//������ȷ��uΪ����
                angle_uv=u_x*v_y-u_y*v_x;	//������ˣ����жϳ��ᵽ����ļн�����
            }
            else{
                far_point_=3;
                near_point_=1;
                angle_uv=v_x*u_y-v_y*u_x;
            }

            if(angle_uv<0){	//�н�Ϊ��
                near_point_=-near_point_;
            }
            /*��ʾ����ƥ����*/
            if(showimage){
                cv::Mat image_before_ransac;
                cv::drawMatches(I_template, template_keypoints, I_ORI_RGB, curimg_keypoints, matches, image_before_ransac);
                cv::imshow("before RANSAC", image_before_ransac);

                cv::Mat image_after_ransac;
                cv::drawMatches(I_template,template_keypoints,I_ORI_RGB,curimg_keypoints,matches,image_after_ransac,cv::Scalar::all(-1),cv::Scalar::all(-1),matchesMask);
                cv::imshow("after RANSAC", image_after_ransac);
                //cv::waitKey(0);
                cv::destroyAllWindows();
            }
        }
        else{
            std::cout<<"numbers of good matchs: "<<n_good_matchs<<std::endl;
            std::cout<<"matches point less than 8"<<std::endl;
            return false;
        }

    }
    else{
        std::cout<<"matches point less than 10"<<std::endl;
        return false;
    }

    return true;
}

//���Ĳ�ɫͼ��תΪopencv��ʽ
void colorimgSubCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_color_ptr;
    try
    {
        cv_color_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    //cvtColor(cv_ptr->image,I_ORI,CV_BGR2GRAY);
    I_ORI_RGB=cv_color_ptr->image;
    get_new_RGBI=true;
    std::cout<<"get RGB image"<<std::endl;
}

