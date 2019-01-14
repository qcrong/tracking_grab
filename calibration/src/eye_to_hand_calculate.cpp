#include <iostream>
#include <ros/ros.h>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <vector>
#include <std_msgs/Int8.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "hand_eye_calculate/handeye.h"
#include "calibration/toolposeChange.h"

using namespace std;
using namespace cv;

int positionSignal=-1;    //到达指定位姿后标志位改变
vector<Mat> Hgij;         //AX=XB中的B
vector<Mat> Hbt;         //基坐标系下机械臂末端位姿
vector<Mat> eye2hand;	//相机坐标系下机械臂末端位姿
Mat img_color;		//相机彩图
bool receive_img=false;	//是否收到图像

//接收track_points发布的标志位
void handeye_signal_subCB(const std_msgs::Int8::ConstPtr &signal_sub)
{
  
  positionSignal=(int)signal_sub->data;
  cout << "positionSignal is " << positionSignal << endl;
}

//接收机械臂位姿变化矩阵
void pose_change_subCB(const calibration::toolposeChange::ConstPtr &poseChange)
{
  Mat b1(4, 4, CV_64FC1, Scalar::all(0));
  for(int i=0; i<4; i++)
  {
    for(int j=0; j<4; j++)	
    {
	  b1.at<double>(i,j)=poseChange->pose_change[4*i+j];
	}		
  }
  Hgij.push_back(b1.clone());
  cout << "poseChange is " << b1 << endl;
}

//接收订阅机械臂末端位姿矩阵
void pose_subCB(const calibration::toolposeChange::ConstPtr &pose)
{
  Mat b1(4, 4, CV_64FC1, Scalar::all(0));
  for(int i=0; i<4; i++)
  {
    for(int j=0; j<4; j++)	
    {
	  b1.at<double>(i,j)=pose->pose_change[4*i+j];
	}		
  }
  Hbt.push_back(b1.clone());
  cout << "pose is " << b1 << endl;
}

//旋转矩阵和平移向量转齐次矩阵
Mat rt2m(const Mat &pose_t,const Mat &pose_r)
{
	Mat pose_m = Mat::eye(cv::Size(4,4),CV_64FC1); 
	for(int j=0; j<3; j++)
  	{
    	for(int k=0; k<3; k++)	
		{
	 	 	pose_m.at<double>(j,k)=pose_r.at<double>(j,k);
		}		
  	}
	for(int j=0; j<3; j++)
	{
	  	pose_m.at<double>(j,3)=pose_t.at<double>(j,0)/1000;
	}
	//pose_m(3,3)=1.0;
	cout<<"齐次矩阵: "<<endl<<pose_m<<endl;
	return pose_m;
}

void rosimage2opencv(const sensor_msgs::ImageConstPtr msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
	}
	catch(cv_bridge::Exception &e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	//cvtColor(cv_ptr->image,I_ORI,CV_BGR2GRAY);
	img_color=cv_ptr->image;
	receive_img=true;
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "eye_to_hand_calculate");   //定义节点名称
	ros::NodeHandle handle;   //为这个进程的节点创建一个句柄
	//发布话题

	//接收话题
	ros::Subscriber signal_sub=handle.subscribe("hand_eye_arm_signal", 1, handeye_signal_subCB); //订阅机械臂运动标志位
	ros::Subscriber pose_change_sub=handle.subscribe("hand_eye_arm_pose_change", 1, pose_change_subCB); //订阅机械臂位姿变化矩阵
	ros::Subscriber pose_sub=handle.subscribe("hand_eye_arm_pose", 1, pose_subCB); //订阅机械臂末端位姿矩阵
	ros::Subscriber image_sub=handle.subscribe("/kinect2/qhd/image_color_rect",1,rosimage2opencv); //订阅相机图像，并转换为Mat格式
	
	//参数变量
	const int image_count = 5;	               //需要保存的图片数量
	int count = 0;  //计算保存的标定图像数量
	int n = 0;        //保存图片的名称
	stringstream tempname;
	string filename;
	//Size image_size = image.size();     //图像的尺寸
	Size board_size = Size(11, 8);            /****    定标板上每行、列的角点数       ****/
    //Size board_size = Size(9, 6);            /****    定标板上每行、列的角点数bhs      ****/
	vector<Point2f> corners;                  /****    缓存每幅图像上检测到的角点       ****/
	string msg;
	int baseLine;
	Size textSize;
	int key;         //记录按键值

	//solvePnP参数
	vector<Point3f> objP;
	
    objP.push_back(Point3f(0,0,0));
	objP.push_back(Point3f(0,150,0));		//(0,150,0)
	objP.push_back(Point3f(0,300,0));
	objP.push_back(Point3f(210,0,0));
	objP.push_back(Point3f(210,150,0));
	objP.push_back(Point3f(210,300,0));
	
    
    Mat objPM;
	Mat(objP).convertTo(objPM,CV_32F);
	//相机参数
	double camD[9] = {534.400487700659, 0.0, 484.24372704867756,
					0.0, 533.9819562746023, 268.8884067683077,
					0.0, 0.0, 1.0};
					
	double distCoeffD[5] = {0.07060073848688614, -0.12252143853363986,
							-0.00012139960322877213, 0.00046942629502756217,
							0.05004052354669021};
							
	Mat camera_matrix = Mat(3,3,CV_64FC1,camD);
	Mat distortion_coefficients = Mat(5,1,CV_64FC1,distCoeffD);

	//计算手眼关系参数
    vector<Mat> v_rotM;
	vector<Mat> v_tvec;

	cv::namedWindow("camera exterior calibration", cv::WINDOW_AUTOSIZE);
	waitKey(3000);
	while (n < image_count && ros::ok())
	{
		ros::spinOnce();
		Mat img_gray;
        cvtColor(img_color, img_gray, CV_RGB2GRAY);	//彩色图片需要转化为灰度图
		if (img_gray.empty() || img_color.empty() || receive_img==false)
		{
			cout << "img error" << endl;
			continue;
		}
		receive_img=true;			
		//cvtColor(image, imageGray, CV_RGB2GRAY);	//彩色图片需要转化为灰度图

		// 提取角点
		bool patternfound = findChessboardCorners(img_gray, board_size, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
		
		key = 0xff & waitKey(50);
		
		if ((key & 255) == 32  || positionSignal!=-1)   //  空格键
		{
			positionSignal=-1;   //重新赋值，为下一个位姿做准备
			if (patternfound)
			{
				n++;
				tempname << "/home/qcrong/Documents/thesis/calibration_result/eye_to_hand/" << n;
				tempname >> filename;
				filename += ".jpg";
				/* 亚像素精确化 */
				cornerSubPix(img_gray, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

				count += corners.size();
				drawChessboardCorners(img_color,board_size,corners,patternfound); //绘制角点，patternfound=true表示角点检测完全
				imwrite(filename, img_color);
				tempname.clear();
				filename.clear();

				vector<Point2f> imgP;
				imgP.push_back(corners.at(0));
				imgP.push_back(corners.at(5));
				imgP.push_back(corners.at(10));
				imgP.push_back(corners.at(77));
				imgP.push_back(corners.at(82));
				imgP.push_back(corners.at(87));

				
				//solvePnP求解相机坐标系下目标物的位姿
				vector<double> rv(3), tv(3);
				Mat rvec(rv), tvec(tv);
				double rm[9];
				Mat rotM(3,3,CV_64FC1,rm);

				Rodrigues(rotM,rvec);
				solvePnP(objPM, Mat(imgP), camera_matrix, distortion_coefficients, rvec, tvec);
				Rodrigues(rvec,rotM);

				v_rotM.push_back(rotM.clone());
  				v_tvec.push_back(tvec.clone());
				eye2hand.push_back(rt2m(tvec,rotM));
				cout<<"图片数量：  "<<n<<endl;
				cout<<"rotation matrix: "<<endl<<rotM<<endl;  
    				//cout<<"translation matrix: "<<endl<<tv[0]<<" "<<tv[1]<<" "<<tv[2]<<endl;
				cout<<"tvec matrix: "<<endl<<tvec.at<double>(0,0)<<" "<<tvec.at<double>(1,0)<<" "<<tvec.at<double>(2,0)<<endl;
				//cout<<"齐次矩阵: "<<endl<<eye2hand[]<<endl;
			}
			else
			{
				std::cout << "Detect Failed.\n";
				drawChessboardCorners(img_color,board_size,corners,patternfound); //绘制角点，patternfound=false表示角点检测不完全
			}
		}
		else if ((key & 255) == 27)     //按esc键退出
		{
			break;
		}
		else{
			drawChessboardCorners(img_color,board_size,corners,patternfound); //绘制角点
		}
		

		baseLine = 0;
		textSize = getTextSize(msg, 1, 1, 1, &baseLine);
		Point textOrigin(img_color.cols - 2 * textSize.width - 10, img_color.rows - 2 * baseLine - 10);
		msg = format("Press 'esc' to quit  %d/%d", n, image_count);
		putText(img_color, msg, textOrigin, 1, 2, CV_RGB(255, 0, 0),2);
		cv::imshow("camera exterior calibration", img_color);
	}

	if(n==image_count)  //达到计数要求，开始标定手眼关系
	{
		ofstream hand_eye_fout("/home/qcrong/Documents/thesis/calibration_result/eye_to_hand/hand_eye_result.txt");  /**    保存定标结果的文件     **/
		vector<Mat> Hcij;		

		for(int i=1;i<image_count;i++)
		{
			Mat a1(4, 4, CV_64FC1, Scalar::all(0));
			Mat a_r=Mat(3,3,CV_64FC1,Scalar::all(0)); 
			Mat a_t=Mat(3,1,CV_64FC1,Scalar::all(0));
			//a_r=v_rotM.at(i-1)*v_rotM.at(i).t();
			//a_t=v_rotM.at(i-1)*(-v_rotM.at(i).t()*v_tvec.at(i))+v_tvec.at(i-1);
			a_r=v_rotM.at(i-1).t()*v_rotM.at(i);
			a_t=v_rotM.at(i-1).t()*v_tvec.at(i)-v_rotM.at(i-1).t()*v_tvec.at(i-1);
			
			for(int j=0; j<3; j++)
			{
				for(int k=0; k<3; k++)	
				{
					a1.at<double>(j,k)=a_r.at<double>(j,k);
				}		
			}
			for(int j=0; j<3; j++)
			{
				a1.at<double>(j,3)=a_t.at<double>(j,0)/1000;
			}
			a1.at<double>(3,3)=1.0;

			Hcij.push_back(a1.clone());
		}
		
		for(int i=0;i<Hcij.size();i++)
		{
			cout<<"Hcij "<< i << endl <<Hcij.at(i)<<endl;
		}
		for(int i=0;i<Hgij.size();i++)
		{
			cout<<"Hgij "<< i << endl <<Hgij.at(i)<<endl;
		}

		Mat Hcg1(4, 4, CV_64FC1);
		Tsai_HandEye(Hcg1, Hgij, Hcij);

		Mat Hcg2(4, 4, CV_64FC1);
		DualQ_HandEye(Hcg2, Hgij, Hcij);

		Mat Hcg3(4, 4, CV_64FC1);
		Inria_HandEye(Hcg3, Hgij, Hcij);

		Mat Hcg4(4, 4, CV_64FC1);
		Navy_HandEye(Hcg4, Hgij, Hcij);

		Mat Hcg5(4, 4, CV_64FC1);
		Kron_HandEye(Hcg5, Hgij, Hcij);

		hand_eye_fout << "cam to hand " << endl;
		hand_eye_fout << "Tsai_HandEye;" << endl;
		hand_eye_fout << Hcg1 << endl << endl;
		hand_eye_fout << "DualQ_HandEye;" << endl;
		hand_eye_fout << Hcg2 << endl << endl;
		hand_eye_fout << "Inria_HandEye;" << endl;
		hand_eye_fout << Hcg3 << endl << endl;
		hand_eye_fout << "Navy_HandEye;" << endl;
		hand_eye_fout << Hcg4 << endl << endl;
		hand_eye_fout << "Kron_HandEye;" << endl;
		hand_eye_fout << Hcg5 << endl << endl << endl;

		Mat Hbe;         //机械臂基座标系到相机坐标系的关系
		hand_eye_fout << "arm base to cam" << endl;
        hand_eye_fout << "Tsai_HandEye;" << endl;
		for(int i=0;i<Hbt.size();i++)
		{
			cout<<"Hbt"<<i<<Hbt[i]<<endl;
			cout<<"Hcg1.inv()"<<Hcg1.inv()<<endl;
			cout<<"eye2hand[i+1].inv()"<<eye2hand[i+1].inv()<<endl;
			hand_eye_fout << Hbt[i]*Hcg1*eye2hand[i+1].inv() << endl << endl;
		}
		hand_eye_fout << "DualQ_HandEye;" << endl;
		for(int i=0;i<Hbt.size();i++)
		{
			hand_eye_fout << Hbt[i]*Hcg2*eye2hand[i+1].inv() << endl << endl;
		}
		hand_eye_fout << "Inria_HandEye;" << endl;
		for(int i=0;i<Hbt.size();i++)
		{
			hand_eye_fout << Hbt[i]*Hcg3*eye2hand[i+1].inv() << endl << endl;
		}
		hand_eye_fout << "Navy_HandEye;" << endl;
		for(int i=0;i<Hbt.size();i++)
		{
			hand_eye_fout << Hbt[i]*Hcg4*eye2hand[i+1].inv() << endl << endl;
		}
		hand_eye_fout << "Kron_HandEye;" << endl;
		for(int i=0;i<Hbt.size();i++)
		{
			hand_eye_fout << Hbt[i]*Hcg5*eye2hand[i+1].inv() << endl << endl;
		}
		
	}


	cout << "finish" << endl;
	return 0;
}
