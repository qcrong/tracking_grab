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

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "camera_inter_calibration");   //定义节点名称
	ros::NodeHandle handle;   //为这个进程的节点创建一个句柄
	//发布话题


	//接收话题


	vpImage<unsigned char> I;  //visp用于存储捕捉到的最新图像
	Mat image;             //OpenCV存储捕捉到的最新图像
	vpROSGrabber g;
	g.setImageTopic("/pylon_camera_node/image_raw");
	g.open(I);          //打开相机
	g.acquire(I);       //获取图像
	vpImageConvert::convert(I, image);   //图像类型转换

	//参数变量
	int image_count = 25;	               //需要保存的图片数量
	int count = 0;  //计算保存的标定图像数量
	int n = 0;        //保存图片的名称
	stringstream tempname;
	string filename;
	Size image_size = image.size();     //图像的尺寸
	Size board_size = Size(11, 8);            /****    定标板上每行、列的角点数       ****/
	vector<Point2f> corners;                  /****    缓存每幅图像上检测到的角点       ****/
	vector< vector<Point2f> >  corners_Seq;    /****  保存检测到的所有角点       ****/
	string msg;
	int baseLine;
	Size textSize;
	int key;         //记录按键值

	cv::namedWindow("camera inter calibration", cv::WINDOW_AUTOSIZE);

	while (n < image_count && ros::ok())
	{
		g.acquire(I);       //获取图像
		vpImageConvert::convert(I, image);   //图像类型转换   

		key = 0xff & waitKey(100);
		if ((key & 255) == 32)   //  空格键
		{
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
			if (patternfound)
			{
				n++;
				tempname << "/home/qcrong/result/" << n;
				tempname >> filename;
				filename += ".jpg";
				/* 亚像素精确化 */
				cornerSubPix(image, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
				count += corners.size();
				corners_Seq.push_back(corners);
				imwrite(filename, image);
				tempname.clear();
				filename.clear();
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

		//msg = "Press 'esc' to quit";
		baseLine = 0;
		textSize = getTextSize(msg, 1, 1, 1, &baseLine);
		Point textOrigin(image.cols - 2 * textSize.width - 10, image.rows - 2 * baseLine - 10);
		msg = format("Press 'esc' to quit  %d/%d", n, image_count);
		putText(image, msg, textOrigin, 1, 1, Scalar(0, 0, 255));
		cv::imshow("camera inter calibration", image);

	}


	destroyWindow("camera inter calibration");
	cout << "角点提取完成！\n";

	/************************************************************************
	  摄像机定标
	  *************************************************************************/
	cout << "开始定标………………" << endl;
	Size square_size = Size(30, 30);                                      /**** 实际测量得到的定标板上每个棋盘格的大小   ****/
	vector< vector<Point3f> >  object_Points;                                      /****  保存定标板上角点的三维坐标   ****/

	Mat image_points = Mat(1, count, CV_32FC2, Scalar::all(0));          /*****   保存提取的所有角点   *****/
	vector<int>  point_counts;                                          /*****    每幅图像中角点的数量    ****/
	Mat intrinsic_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0));                /*****    摄像机内参数矩阵    ****/
	Mat distortion_coeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));            /* 摄像机的5个畸变系数：k1,k2,p1,p2,k3 */
	vector<Mat> rotation_vectors;                                      /* 每幅图像的旋转向量 */
	vector<Mat> translation_vectors;                                  /* 每幅图像的平移向量 */

	ofstream fout("/home/e305/Documents/calibration_result/inter/calibration_result.txt");  /**    保存定标结果的文件     **/

	/* 初始化定标板上角点的三维坐标 */
	for (int t = 0; t < image_count; t++)
	{
		vector<Point3f> tempPointSet;
		for (int i = 0; i < board_size.height; i++)
		{
			for (int j = 0; j < board_size.width; j++)
			{
				/* 假设定标板放在世界坐标系中z=0的平面上 */
				Point3f tempPoint;
				tempPoint.x = (float)i*square_size.width;
				tempPoint.y = (float)j*square_size.height;		//y轴先增长
				tempPoint.z = 0.0;
				tempPointSet.push_back(tempPoint);
			}
		}
		object_Points.push_back(tempPointSet);
	}

	/* 初始化每幅图像中的角点数，这里我们假设每幅图像中都可以看到完整的定标板 */
	for (int i = 0; i < image_count; i++)
	{
		point_counts.push_back(board_size.width*board_size.height);
	}

	/* 开始定标 */
	calibrateCamera(object_Points, corners_Seq, image_size, intrinsic_matrix, distortion_coeffs, rotation_vectors, translation_vectors);
	std::cout << "定标完成！\n";

	/************************************************************************
	 对定标结果进行评价
	 *************************************************************************/
	std::cout << "开始评价定标结果………………" << endl;
	double total_err = 0.0;                   /* 所有图像的平均误差的总和 */
	double err = 0.0;                        /* 每幅图像的平均误差 */
	vector<Point2f>  image_points2;             /****   保存重新计算得到的投影点    ****/

	std::cout << "每幅图像的定标误差：" << endl;
	fout << "每幅图像的定标误差：" << endl << endl;
	for (int i = 0; i < image_count; i++)
	{
		vector<Point3f> tempPointSet = object_Points[i];
		/****    通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点     ****/
		projectPoints(tempPointSet, rotation_vectors[i], translation_vectors[i], intrinsic_matrix, distortion_coeffs, image_points2);
		/* 计算新的投影点和旧的投影点之间的误差*/
		vector<Point2f> tempImagePoint = corners_Seq[i];
		Mat tempImagePointMat = Mat(1, tempImagePoint.size(), CV_32FC2);
		Mat image_points2Mat = Mat(1, image_points2.size(), CV_32FC2);
		for (unsigned int j = 0; j < tempImagePoint.size(); j++)
		{
			image_points2Mat.at<Vec2f>(0, j) = Vec2f(image_points2[j].x, image_points2[j].y);
			tempImagePointMat.at<Vec2f>(0, j) = Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
		}
		err = norm(image_points2Mat, tempImagePointMat, NORM_L2);
		total_err += err /= point_counts[i];
		std::cout << "第" << i + 1 << "幅图像的平均误差：" << err << "像素" << endl;
		fout << "第" << i + 1 << "幅图像的平均误差：" << err << "像素" << endl;
	}
	std::cout << "总体平均误差：" << total_err / image_count << "像素" << endl;
	fout << "总体平均误差：" << total_err / image_count << "像素" << endl << endl;
	std::cout << "评价完成！" << endl;

	/************************************************************************
	保存定标结果
	*************************************************************************/
	std::cout << "开始保存定标结果………………" << endl;
	Mat rotation_matrix = Mat(3, 3, CV_64FC1, Scalar::all(0)); /* 保存每幅图像的旋转矩阵 */
	//两幅图像间相机位姿变换
	Mat old_rotation_matrix = Mat(3, 3, CV_64FC1, Scalar::all(0));
	Mat out_rotation_matrix = Mat(3, 3, CV_64FC1, Scalar::all(0));
	Mat out_trtranslation_vectors = Mat(3, 1, CV_64FC1, Scalar::all(0));

	fout << "相机内参数矩阵：" << endl;
	fout << intrinsic_matrix << endl << endl;
	fout << "畸变系数：\n";
	fout << distortion_coeffs << endl << endl << endl;
	for (int i = 0; i < image_count; i++)
	{
		//		fout << "第" << i + 1 << "幅图像的旋转向量：" << endl;
		//		fout << rotation_vectors[i] << endl;

		/* 将旋转向量转换为相对应的旋转矩阵 */
		Rodrigues(rotation_vectors[i], rotation_matrix);
		fout << "第" << i + 1 << "幅图像的旋转矩阵：" << endl;
		fout << rotation_matrix << endl;
		fout << "第" << i + 1 << "幅图像的平移向量：" << endl;
		fout << translation_vectors[i] << endl << endl;
	}

	std::cout << "完成保存" << endl;
	fout << endl;

	cv::destroyWindow("camera inter calibration");
	g.close();  //关闭相机
	return 0;
}
