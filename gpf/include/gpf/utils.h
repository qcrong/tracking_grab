#ifndef UTILS_H_
#define UTILS_H_

// basic c++ libs
#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include <random>
#include <chrono>
#include <csignal>
#include <time.h>
#include <chrono>
#include <fstream>
#include <sstream>


// opencv
#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

// eigen
#include <unsupported/Eigen/MatrixFunctions>
#include <Eigen/QR>
//#include <Eigen/Dense>
#include<Eigen/Core>

#if _MSC_VER
#include <direct.h>
#define snprintf _snprintf_s
#endif

//////////////////////////////////////////////////
// definitions
//////////////////////////////////////////////////

// N_{aff(2)} (X, S)
inline void mvnrnd_aff2( cv::Mat &X, cv::Mat &S );
// N_{SL(3)} (X, S)
void mvnrnd_sl3( cv::Mat &X, cv::Mat &S );
// N_{SE(3)} (X, S)
void mvnrnd_se3( cv::Mat &X, cv::Mat &S );
// expm(X), FIXME: 3x3
inline void expm( cv::Mat &i_X, cv::Mat &o_X);
// logm(X), FIXME: 3x3
inline void logm( cv::Mat &i_X, cv::Mat &o_X );
// qr(A)
void qr_thin( cv::Mat &i_A, cv::Mat &o_Q );
// depth2pc
void depth2pc( cv::Mat &i_d, cv::Mat &o_p );

//////////////////////////////////////////////////
// implementation
//////////////////////////////////////////////////

// N_{Aff(2)} (X, S), FIXME: S is diagonal
inline void mvnrnd_aff2( cv::Mat &X, cv::Mat &S, std::vector<cv::Mat> &E, cv::Mat &o_X ) {
	//std::cout << "S: " << S << std::endl;
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    cv::Mat eE(3, 3, CV_32F, cv::Scalar(0));
    for( int b=0; b<6; ++b ) {
        std::normal_distribution<double> normdist(0.0, std::sqrt(S.at<float>(b, b)));
        float r = normdist(generator);
        eE += r * E[b];
    }
	//std::cout << "eE" << eE << std::endl;
    cv::Mat eEexp;
    expm(eE, eEexp);
	//std::cout << "eEexp" << eEexp << std::endl;
    // return
    o_X = X * eEexp;
	//float temp = o_X.at<float>(o_X.rows - 1, o_X.rows - 1);
    o_X = o_X / o_X.at<float>(o_X.rows-1, o_X.rows-1); //FIXME: okay?
	//std::cout << "o_X" << o_X << std::endl;
}
inline void my_mvnrnd_aff2(const cv::Mat &prev_x, const cv::Mat &prev_a, const cv::Mat &S, const std::vector<cv::Mat> &E, cv::Mat &o_X) {
	//std::cout << "S: " << S << std::endl;
	//unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	//cv::waitKey(1);
	static unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	//std::cout << "seed" << seed << std::endl;
	static std::default_random_engine generator(seed);
	cv::Mat eE(3, 3, CV_32F, cv::Scalar(0));
	for (int b = 0; b < 6; ++b) {
		std::normal_distribution<double> normdist(0.0, std::sqrt(S.at<float>(b, b)));
		float r = normdist(generator);
		eE += r * E[b];
	}
	eE += prev_a;
	//std::cout << "eE" << eE << std::endl;
	cv::Mat eEexp;
	expm(eE, eEexp);
	//std::cout << "eEexp" << eEexp << std::endl;
	// return
	o_X = prev_x * eEexp;
	//float temp = o_X.at<float>(o_X.rows - 1, o_X.rows - 1);
	//std::cout << "o_X1" << o_X.at<float>(o_X.rows - 1, o_X.rows - 1) << std::endl;
	//o_X = o_X / o_X.at<float>(o_X.rows - 1, o_X.rows - 1); //FIXME: okay?
	
}
// N_{SL(3)} (X, S), FIXME: S is diagonal
void mvnrnd_sl3( cv::Mat X, cv::Mat &S, std::vector<cv::Mat> &E, cv::Mat &o_X ) {
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    cv::Mat eE(E[0].rows, E[0].cols, CV_32F, cv::Scalar(0));
    for( int b=0; b<E.size(); ++b ) {
        std::normal_distribution<double> normdist(0.0, std::sqrt(S.at<float>(b, b)));
        float r = normdist(generator);
        eE += r * E[b];
    }
    cv::Mat eEexp;
    expm(eE, eEexp);
    // return
    o_X = X * eEexp;
    o_X = o_X / o_X.at<float>(o_X.rows-1, o_X.rows-1); //FIXME: okay?
    // det(X) = 1
    o_X = o_X / std::pow( cv::determinant(o_X), 1/o_X.rows );
}

// N_{SE(3)} (X, S), FIXME: S is diagonal
void mvnrnd_se3( cv::Mat &X, cv::Mat &S, std::vector<cv::Mat> &E, cv::Mat &o_X ) {
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    cv::Mat eE(E[0].rows, E[0].cols, CV_32F, cv::Scalar(0));
    for( int b=0; b<E.size(); ++b ) {
        std::normal_distribution<double> normdist(0.0, std::sqrt(S.at<float>(b, b)));
        float r = normdist(generator);
        eE += r * E[b];
    }
    cv::Mat eEexp;
    expm(eE, eEexp);
    // return
    o_X = X * eEexp;
    o_X = o_X / o_X.at<float>(o_X.rows-1, o_X.rows-1); //FIXME: okay?
}

// expm(X)
inline void expm( cv::Mat &i_X, cv::Mat &o_X) {
    int n = i_X.rows;
    // expm
    Eigen::MatrixXd X_e(n, n);
    for( int i=0; i<n; ++i )  //深拷贝矩阵，做格式转换,方便调用Eigen库
        for( int j=0; j<n; ++j )
            X_e(i, j) = i_X.at<float>(i, j);
    Eigen::MatrixXd Xexp_e = X_e.exp();
    cv::Mat Xexp(n, n, CV_32F);	
    for( int i=0; i<n; ++i )	//转换回Mat
        for( int j=0; j<n; ++j )
            Xexp.at<float>(i, j) = Xexp_e(i, j);
    o_X = Xexp;
}


// logm(X)
inline void logm( cv::Mat &i_X, cv::Mat &o_X ) {
    int n = i_X.rows;
    // logm
    Eigen::MatrixXd X_e(n, n);
    for( int i=0; i<n; ++i )
        for( int j=0; j<n; ++j )
            X_e(i, j) = i_X.at<float>(i, j);
    Eigen::MatrixXd Xlog_e = X_e.log();
    cv::Mat Xlog(n, n, CV_32F);
    for( int i=0; i<n; ++i )
        for( int j=0; j<n; ++j )
            Xlog.at<float>(i, j) = Xlog_e(i, j);
    o_X = Xlog;
}

// resample
	

// unsigned int64 GetRDTSC( void )
// {
//     unsigned int tmp1 = 0;
//     unsigned int tmp2 = 0;
//     unsigned int64 ret = 0;
//     __asm
//     {  
//         RDTSC;  
//         mov tmp1, eax;
//         mov tmp2, edx;
//     }
//     ret = tmp2;
//     return ((ret << 32) | tmp1);
// }

// Particle resampling according to the weights 感觉有问题
void resample(std::vector<float> &w, int N, std::vector<int> &outindex)
{
    //outindex.resize(N);
	CvRNG rng_state = cvRNG(cvGetTickCount ());	//基本随机数，返回64位长整数的时间数据,在OpenCV是为CvRNG设置的专用种子。
	CvMat* U = cvCreateMat(1, 1, CV_64F);
	cvRandArr(&rng_state, U, CV_RAND_UNI, cvScalar(0), cvScalar(1)); //均匀分布的随机数填充数组，范围[cvScalar(0), cvScalar(1))
	double UU = CV_MAT_ELEM(*U, double, 0, 0)/((double)N);

	int k=0;
    //std::vector<float> n_R(N);
	float n_R;
	//double* n_R;
	//n_R = new double[N];
	for(int i=0; i < N; i++)
	{
		n_R = floor((w[i] - UU)*N) + 1; //向下取整
		UU = UU + n_R/N - w[i];	

		if(n_R != 0) {	//w[i]>uu才会被保留下来，这里如何保证粒子数不变，也就是outindex下标不越界呢？
			for(int j=0; j < n_R; j++) {
				outindex[k] = i;
				k++;
			}
		}
	}
	cvReleaseMat(&U);
}

// qr factorization: FIXME: float
void qr_thin( cv::Mat &i_A, cv::Mat &o_Q ) {
    // copy
    Eigen::MatrixXf A_E(i_A.rows, i_A.cols);
    for( int i=0; i<i_A.rows; ++i )
        for( int j=0; j<i_A.cols; ++j )
            A_E(i, j) = i_A.at<float>(i, j);
    // qr
    Eigen::HouseholderQR< Eigen::MatrixXf > qr_E(A_E);
    Eigen::MatrixXf I(Eigen::MatrixXf::Identity(A_E.rows(), A_E.cols()));
    Eigen::MatrixXf Q = qr_E.householderQ() * I;
    // return
    o_Q = cv::Mat(Q.rows(), Q.cols(), CV_32F);
    for( int i=0; i<Q.rows(); ++i )
        for( int j=0; j<Q.cols(); ++j )
            o_Q.at<float>(i, j) = Q(i, j);
}

// depth2pc
void depth2pc( const cv::Mat &i_d, const cv::Mat &i_K, cv::Mat &o_p ) {
    float mm_per_m = 1000;
    int dims[3];
    dims[0] = i_d.rows;
    dims[1] = i_d.cols;
    dims[2] = 3;
    cv::Mat pc(3, dims, CV_32F);
    for( int i=0; i<i_d.rows; ++i ) {
        for( int j=0; j<i_d.cols; ++j ) {
            float x = float(j) - i_K.at<float>(0, 2);
            float y = float(i) - i_K.at<float>(1, 2);
            // x
            pc.at<float>(i, j, 0) = x * i_d.at<float>(i, j) / i_K.at<float>(0, 0) / mm_per_m;
            pc.at<float>(i, j, 1) = y * i_d.at<float>(i, j) / i_K.at<float>(1, 1) / mm_per_m;
            pc.at<float>(i, j, 2) = i_d.at<float>(i, j) / mm_per_m;
        }
    }
    // return
    o_p = pc;
}


//////////////////////////////////////////////////
// unit test
//////////////////////////////////////////////////

void test_utils() {
}


#endif
