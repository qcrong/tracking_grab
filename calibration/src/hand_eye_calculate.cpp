#include <visp/vpImage.h> 
#include <visp_ros/vpROSGrabber.h>
#include <iostream>
#include "ros/ros.h" 
#include <visp3/core/vpConfig.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpKeyboard.h>
#include <stdio.h>
#include <visp3/core/vpColor.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <vector>
#include <std_msgs/Int8.h>
#include "hand_eye_calculate/handeye.h"
#include "calibration/toolposeChange.h"

using namespace std;
using namespace cv;

int positionSignal=-1;    //到达指定位姿后标志位改变
vector<Mat> Hgij;         //AX=XB中的B

//接收track_points发布的标志位
void handeye_signal_subCB(const std_msgs::Int8::ConstPtr &signal_sub)
{
  
  positionSignal=(int)signal_sub->data;
  cout << "positionSignal is " << positionSignal << endl;
}

//接收track_points发布的标志位
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

int main(int argc, char **argv)
{
	ros::init(argc, argv, "hand_eye_calculate");   //定义节点名称
	ros::NodeHandle handle;   //为这个进程的节点创建一个句柄
	//发布话题

	//接收话题
	ros::Subscriber signal_sub=handle.subscribe("hand_eye_arm_signal", 1, handeye_signal_subCB); //订阅机械臂运动标志位
	ros::Subscriber pose_change_sub=handle.subscribe("hand_eye_arm_pose_change", 10, pose_change_subCB); //

	vpImage<unsigned char> I;  //visp用于存储捕捉到的最新图像
	Mat image;             //OpenCV存储捕捉到的最新图像
	vpROSGrabber g;
	g.setImageTopic("/pylon_camera_node/image_raw");
	g.open(I);          //打开相机
	g.acquire(I);       //获取图像
	vpImageConvert::convert(I, image);   //图像类型转换
	
	
	//参数变量
	const int image_count = 5;	               //需要保存的图片数量
	int count = 0;  //计算保存的标定图像数量
	int n = 0;        //保存图片的名称
	stringstream tempname;
	string filename;
	Size image_size = image.size();     //图像的尺寸
	Size board_size = Size(11, 8);            /****    定标板上每行、列的角点数       ****/
	vector<Point2f> corners;                  /****    缓存每幅图像上检测到的角点       ****/
	string msg;
	int baseLine;
	Size textSize;
	int key;         //记录按键值

	//solvePnP参数
	vector<Point3f> objP;
	objP.push_back(Point3f(0,0,0));
	objP.push_back(Point3f(0,150,0));
	objP.push_back(Point3f(0,300,0));
	objP.push_back(Point3f(210,0,0));
	objP.push_back(Point3f(210,150,0));
	objP.push_back(Point3f(210,300,0));
	Mat objPM;
	Mat(objP).convertTo(objPM,CV_32F);
	//相机参数
	double camD[9] = {750.0042620573236, 0, 330.9371610210754,
  0, 749.824122908542, 240.245636050228,
  0, 0, 1};
	double distCoeffD[5] = {-0.4060816337423698, 0.3252315409025565, -0.0005665095590638105, -0.0002624987785133228, -0.3428026884282165};
	Mat camera_matrix = Mat(3,3,CV_64FC1,camD);
	Mat distortion_coefficients = Mat(5,1,CV_64FC1,distCoeffD);

	//计算手眼关系参数
    vector<Mat> v_rotM;
	vector<Mat> v_tvec;

	cv::namedWindow("camera exterior calibration", cv::WINDOW_AUTOSIZE);
	while (n < image_count && ros::ok())
	{
		g.acquire(I);       //获取图像
		vpImageConvert::convert(I, image);   //图像类型转换   
		if (image.empty())
		{
			cout << "img is empty" << endl;
			g.close();
			return -1;
		}

		//Mat imageGray = image;			
		//cvtColor(image, imageGray, CV_RGB2GRAY);	//彩色图片需要转化为灰度图

		// 提取角点
		bool patternfound = findChessboardCorners(image, board_size, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);

		key = 0xff & waitKey(50);
		ros::spinOnce();
		if ((key & 255) == 32  || positionSignal!=-1)   //  空格键
		{
			positionSignal=-1;   //重新赋值，为下一个位姿做准备
			if (patternfound)
			{
				n++;
				tempname << "/home/e305/Documents/calibration_result/hand_eye/" << n;
				tempname >> filename;
				filename += ".jpg";
				/* 亚像素精确化 */
				cornerSubPix(image, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
				count += corners.size();
				imwrite(filename, image);
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
				cout<<"图片数量：  "<<n<<endl;
				cout<<"rotation matrix: "<<endl<<rotM<<endl;  
    				//cout<<"translation matrix: "<<endl<<tv[0]<<" "<<tv[1]<<" "<<tv[2]<<endl;
				cout<<"tvec matrix: "<<endl<<tvec.at<double>(0,0)<<" "<<tvec.at<double>(1,0)<<" "<<tvec.at<double>(2,0)<<endl;
			}
			else
			{
				std::cout << "Detect Failed.\n";
			}
		}
		else if ((key & 255) == 27)     //按esc键退出
		{
			break;
		}

		for(int i=0;i<corners.size();i++)
		{
			cv::circle(image,corners.at(i),2,CV_RGB(0,0,255),1);
		}

		baseLine = 0;
		textSize = getTextSize(msg, 1, 1, 1, &baseLine);
		Point textOrigin(image.cols - 2 * textSize.width - 10, image.rows - 2 * baseLine - 10);
		msg = format("Press 'esc' to quit  %d/%d", n, image_count);
		putText(image, msg, textOrigin, 1, 1, CV_RGB(0, 0, 255));
		cv::imshow("camera exterior calibration", image);
	}
	g.close();

	if(n==image_count)  //达到计数要求，开始标定手眼关系
	{
		ofstream hand_eye_fout("/home/e305/Documents/calibration_result/hand_eye/hand_eye_result.txt");  /**    保存定标结果的文件     **/
		vector<Mat> Hcij;		

		for(int i=1;i<image_count;i++)
		{
			Mat a1(4, 4, CV_64FC1, Scalar::all(0));
			Mat a_r=Mat(3,3,CV_64FC1,Scalar::all(0)); 
			Mat a_t=Mat(3,1,CV_64FC1,Scalar::all(0));
			a_r=v_rotM.at(i-1)*v_rotM.at(i).t();
			a_t=v_rotM.at(i-1)*(-v_rotM.at(i).t()*v_tvec.at(i))+v_tvec.at(i-1);
			
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

		hand_eye_fout << "相机坐标系到手爪坐标系的转换关系" << endl;
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
		
	}

	cout << "手眼标定完成" << endl;
	return 0;
}
