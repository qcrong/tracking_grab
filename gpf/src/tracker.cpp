#include "in_ros.h"
#include <sys/stat.h>

class Tracker {
public:
	//状态变量，Aff2处理彩图，后面两者为深度图和点云图
	enum STATE_DOMAIN {
		Aff2, SL3, SE3
	};
	struct Inputs {
		cv::Mat I; // image
		cv::Mat D; // depth
		cv::Mat C; // point cloud
	};
	struct SSDParams {
		cv::Mat R;  //1X1
	};
	struct PCAParams {
		cv::Mat R;      //R_PCA 2X2
		int n_basis;	//???
		int update_interval;//更新间隔
		float forget_factor;

	};
	//配置文件参数
	struct Params {
		std::string id;	//proj name

		//video setting
		std::string frame_dir;//视频帧路径
		std::string file_fmt; //file format  文件名
		int init_frame;
		int start_frame;
		int end_frame;
		float fps;
		cv::Mat K;	//row major indexing  3X3 ????

		//optimization settings
		int n_particles;

        std::vector<float> template_xs;  //模板对应当前图像的4个角点x坐标
        std::vector<float> template_ys;	 //模板对应当前图像的4个角点y坐标
        std::vector<cv::Point2f> I_template_conners;//模板图像四个顶点
        std::vector<cv::Point2f> I_template_features;//模板图像中用于匹配的特征点
		STATE_DOMAIN state_domain;		//状态变量
		std::vector<cv::Mat> E;			//Ei 6X3X3
		cv::Mat P;	//以state_sigs的平方为对角线的矩阵
		cv::Mat Q;	// P/fps
		float a;		// AR process parameter ???

		std::string obs_mdl_type;	//obs_mdl选用类型：SSD或者onlinePCA
		SSDParams ssd_params; //R_SSSD
		PCAParams pca_params;

        std::string template_img_dir; //path of template image
	};
	struct State {
		cv::Mat X;
	};
	struct AR {
		cv::Mat A;
	};
	struct Particle {
		struct State state;
		struct AR    ar;
	};
private:
	Tracker::Params         params_;
	Tracker::State          state_;//初值为对角线为1，最后一列除对角线外为多边形顶点均值的矩阵
	std::vector<Particle>   particles_;
	std::vector<float>      w_;	//权重
	std::vector<cv::Mat>    tracks_;
	cv::Mat                 template_pnts_; //模板内图像像素坐标与顶点坐标均值之差，彩图为3Xn，每行分别代表x,y，1
	cv::Mat                 template_poly_pnts_;//模板多边形顶点像素坐标与顶点坐标均值之差，彩图为3Xn，每行分别代表x,y，1

	std::string             obs_mdl_type_;//obs_mdl选用类型：SSD或者onlinePCA
	cv::Mat                 pca_basis_;
	cv::Mat                 pca_mean_;
	cv::Mat                 pca_svals_;
	int                     pca_n_;

	double                  time_prev_sec_;
    //double                  time_pub_sec;

	// buffers
	std::vector<Tracker::Particle> particles;
	std::vector<Tracker::Particle> particles_res;
	std::vector<float> w;
	std::vector<float> w_res;
	//相机内参,先进行标定，然后订阅相机话题查看
//	double camera_factor;
//	double camera_cx;
//	double camera_cy;
//	double camera_fx;
//	double camera_fy;
	//长边编号和短边编号
    int far_point, near_point;

private:
	void warp_template(const cv::Mat &i_I, const cv::Mat &i_X, const cv::Mat &i_template_pnts, cv::Mat &o_warped_I) {
		// warp（弯曲，使变形） points
		cv::Mat warped_pnts;
		if (params_.state_domain == STATE_DOMAIN::SE3) {
			cv::Mat X_rot = i_X * i_template_pnts;
			warped_pnts = params_.K * X_rot.rowRange(0, 3);
		}
		else {
			warped_pnts = i_X * i_template_pnts;
		}
		// convert to int
		//std::vector<int> c_ind(warped_pnts.cols), r_ind(warped_pnts.cols);//valid的值并没有传到子函数外起什么作用, valid
		int c_ind, r_ind;
		//cv::Mat warped_I(warped_pnts.cols, 1, CV_32F);
		for (int i = 0; i < warped_pnts.cols; ++i) {
			//int c = std::round(warped_pnts.at<float>(0, i) / warped_pnts.at<float>(2, i));//初始化时warped_pnts.at<float>(2, i)为1
			//int r = std::round(warped_pnts.at<float>(1, i) / warped_pnts.at<float>(2, i));//这里为什么要进行归一化？？？
			c_ind = std::round(warped_pnts.at<float>(0, i));
			r_ind = std::round(warped_pnts.at<float>(1, i));

			int c = std::min(std::max(0, c_ind), i_I.cols - 1);
			int r = std::min(std::max(0, r_ind), i_I.rows - 1);
			o_warped_I.at<float>(i, 0) = (i_I.at<float>(r, c));
			//if (c < 0 || i_I.cols - 1 < c || r < 0 || i_I.rows - 1 < r) { //如果超出图像边界
				//valid.push_back(0);
				//std::runtime_error("assumes all points are valid.");
			//}
			//else
				//valid.push_back(1);
		}
		// copy values
		//if (i_I.channels() != 1)
			//std::runtime_error("assume a gray image");
		//cv::Mat warped_I(c_ind.size(), 1, CV_32F);
		//for (int i = 0; i < c_ind.size(); ++i) { //超出边界的值按边界值进行处理
		//	int c = std::min(std::max(0, c_ind[i]), i_I.cols - 1);
		//	int r = std::min(std::max(0, r_ind[i]), i_I.rows - 1);
		//	warped_I.at<float>(i, 0) = (i_I.at<float>(r, c));
		//}
		//std::cout << warped_I.size() << std::endl;
		// return
		//o_warped_I = warped_I;
	}
	void learn_obs_mdl(const cv::Mat &i_I, const cv::Mat &i_X) {//将当前跟踪的区域添加到tracks_
		int init_size = 5;
		// obtain/save the corresponding template
		cv::Mat warped_I(template_pnts_.cols, 1, CV_32F);//记录的是像素值
		//template_pnts_按照i_X进行变换，将变换后坐标对应i_I图中的像素值拷贝到warped_I中，
		//这里一直按着模板中的进行变换，是否可以在上一次的基础上进行变换
		warp_template(i_I, i_X, template_pnts_, warped_I);
		tracks_.push_back(warped_I);//tracks_一直在增大，内存不够？
		// choose the obs mdl
		obs_mdl_type_ = params_.obs_mdl_type;
		if (obs_mdl_type_.compare("onlinePCA") == 0 && tracks_.size() < init_size)
			obs_mdl_type_ = "SSD";
		// update pca
		if (obs_mdl_type_.compare("onlinePCA") == 0) {
			if (pca_basis_.cols == 0) {
				// initiate
				// X
				cv::Mat X(tracks_[0].rows, tracks_.size(), CV_32F);
				for (int i = 0; i < tracks_.size(); ++i)
					tracks_[i].col(0).copyTo(X.col(i));
				std::cerr << "- size(X), should be n x 2: " << X.rows << ", " << X.cols << std::endl;
				// Xbar
				cv::Mat Xbar;
				cv::reduce(X, Xbar, 1, CV_REDUCE_AVG);
				// A = X - Xbar
				cv::Mat A(X.rows, X.cols, CV_32F);
				for (int i = 0; i < X.cols; ++i)
					A.col(i) = X.col(i) - Xbar;
				// svd(A)
				cv::Mat U, Vt, sv;
				cv::SVD::compute(A, sv, U, Vt);
				std::cerr << "- size(U): " << U.rows << ", " << U.cols << std::endl;
				std::cerr << "- sv: " << sv << std::endl;
				// keep valid basis
				int n_val_basis = std::min(params_.pca_params.n_basis, U.cols);
				int n_eff_basis = -1;
				float eff_thres = 0.95;
				cv::Mat sv_sum;
				cv::Mat sv_cumsum(1, 1, CV_32F, cv::Scalar(0));
				cv::reduce(sv, sv_sum, 0, CV_REDUCE_SUM);
				for (int i = 0; i<sv.rows; ++i) {
					sv_cumsum += sv.at<float>(i);
					float thes = sv_cumsum.at<float>(0) / sv_sum.at<float>(0);
					if (thes > eff_thres) {
						n_eff_basis = i + 1;
						break;
					}
				}
				n_val_basis = std::min(n_val_basis, n_eff_basis);
				std::cerr << "- n_val_basis: " << n_val_basis << std::endl;
				// save
				if (n_val_basis > 0) {
					U.colRange(0, n_val_basis).copyTo(pca_basis_);
					sv.rowRange(0, n_val_basis).copyTo(pca_svals_);
					pca_mean_ = Xbar;
					pca_n_ = X.cols;

					std::cerr << "- initial pca_svals_: " << pca_svals_ << std::endl;
					std::cerr << "- initial size(pca_basis_): " << pca_basis_.rows << ", " << pca_basis_.cols << std::endl;
					std::cerr << "- initial pca_basis_[0]: " << pca_basis_.at<float>(0) << std::endl;
					std::cerr << "- initial pca_n_: " << pca_n_ << std::endl;
					std::cerr << "- initial size(pca_mean_): " << pca_mean_.rows << ", " << pca_mean_.cols << std::endl;
					std::cerr << "- initial pca_mean_[0]: " << pca_mean_.at<float>(0) << std::endl;
				}
			}
			else {	//SSD
				// update
				if ((tracks_.size() - init_size) % params_.pca_params.update_interval == 0) {	//每5帧更新一次，- init_size是否有必要？
					int m = params_.pca_params.update_interval;
					float ff = params_.pca_params.forget_factor;
					const cv::Mat M = pca_mean_;		//貌似没有初值，第一次执行完本函数后才会更新
					const cv::Mat U = pca_basis_;		//貌似没有初值，第一次执行完本函数后才会更新
					const cv::Mat sv = pca_svals_;		//貌似没有初值，第一次执行完本函数后才会更新
					const int n = pca_n_; //貌似没有初值，第一次执行完本函数后才会更新
					// B
					cv::Mat B(tracks_[0].rows, m, CV_32F); //tracks_[0].rows最初要追踪的模板像素个数
					for (int i = tracks_.size() - m; i < tracks_.size(); ++i)
						tracks_[i].col(0).copyTo(B.col(i - (tracks_.size() - m)));//把最近的5列数据拷贝到B中
					std::cerr << "- size(X), should be n x 2: " << B.rows << ", " << B.cols << std::endl; //这个输出无意义
					// M_B
					cv::Mat M_B;
					cv::reduce(B, M_B, 1, CV_REDUCE_AVG);  //列向量求均值
					// M_C
					cv::Mat M_C = ff*float(n) / (ff*float(n) + float(m))*M + float(m) / (ff*float(n) + float(m))*M_B;
					// Bn
					cv::Mat Bn(B.rows, B.cols, CV_32F);
					for (int i = 0; i < B.cols; ++i)
						Bn.col(i) = B.col(i) - M_B;
					// Badd
					cv::Mat Badd = std::sqrt(float(n)*float(m) / (float(n) + float(m))) * (M_B - M);
					// Bh
					cv::Mat Bh;
					cv::hconcat(Bn, Badd, Bh);
					// Bt
					cv::Mat Bt_i = Bh - U * U.t() * Bh;
					cv::Mat Bt;
					qr_thin(Bt_i, Bt);
					std::cerr << "- Bt.size(): " << Bt.rows << ", " << Bt.cols << std::endl;
					// R
					cv::Mat R, UB1, UB2, UB, LB1, LB2, LB;
					UB1 = ff * cv::Mat::diag(sv);
					UB2 = U.t() * Bh;
					cv::hconcat(UB1, UB2, UB);
					LB1 = cv::Mat::zeros(Bt.cols, sv.rows, CV_32F);
					LB2 = Bt.t() * Bt_i;
					cv::hconcat(LB1, LB2, LB);
					R.push_back(UB);
					R.push_back(LB);
					std::cerr << "- R.size(): " << R.rows << ", " << R.cols << std::endl;
					// svd(R)
					cv::Mat svt, Ut, Vtt;
					cv::SVD::compute(R, svt, Ut, Vtt);
					std::cerr << "- S of svd(R): " << svt << std::endl;
					// keep valid basis
					int n_val_basis = std::min(params_.pca_params.n_basis, Ut.cols);
					int n_eff_basis = -1;
					float eff_thres = 0.95;
					cv::Mat sv_sum;
					cv::Mat sv_cumsum(1, 1, CV_32F, cv::Scalar(0));
					cv::reduce(svt, sv_sum, 0, CV_REDUCE_SUM);
					for (int i = 0; i<svt.rows; ++i) {
						sv_cumsum += svt.at<float>(i);
						float thes = sv_cumsum.at<float>(0) / sv_sum.at<float>(0);
						if (thes > eff_thres) {
							n_eff_basis = i + 1;
							break;
						}
					}
					n_val_basis = std::min(n_val_basis, n_eff_basis);
					std::cerr << "- n_val_basis: " << n_val_basis << std::endl;
					// save
					cv::Mat UBt, U_C, sv_C;
					cv::hconcat(U, Bt, UBt);
					U_C = UBt * Ut.colRange(0, n_val_basis);
					sv_C = svt.rowRange(0, n_val_basis);

					U_C.copyTo(pca_basis_);
					sv_C.copyTo(pca_svals_);
					pca_mean_ = M_C;
					pca_n_ = m + n;

					std::cerr << "- updated pca_svals_: " << pca_svals_ << std::endl;
					std::cerr << "- updated size(pca_basis_): " << pca_basis_.rows << ", " << pca_basis_.cols << std::endl;
					std::cerr << "- updated pca_basis_[0]: " << pca_basis_.at<float>(0) << std::endl;
					std::cerr << "- updated pca_n_: " << pca_n_ << std::endl;
					std::cerr << "- updated size(pca_mean_): " << pca_mean_.rows << ", " << pca_mean_.cols << std::endl;
					std::cerr << "- updated pca_mean_[0]: " << pca_mean_.at<float>(0) << std::endl;
				}
			}

		}


	}
	void eval_obs_mdl(const cv::Mat &i_I, const cv::Mat &i_X, cv::Mat &o_dist, cv::Mat &o_R) {

		cv::Mat warped_I(template_pnts_.cols, 1, CV_32F);
		//std::cout << template_pnts_.cols << std::endl;
		warp_template(i_I, i_X, template_pnts_, warped_I);
		if (obs_mdl_type_.compare("SSD") == 0) {
			// SSD
			cv::Mat diff = warped_I - tracks_[0];
			// return
			//if (diff.rows < diff.cols)
				//std::runtime_error("- invalid shape of diff");
            double len = double(diff.rows);
            double dist = diff.dot(diff) / len; //点乘求平方
            o_dist = cv::Mat(1, 1, CV_64F, cv::Scalar(dist));
			o_R = params_.ssd_params.R;
		}
		else if (obs_mdl_type_.compare("onlinePCA") == 0) {
			const cv::Mat M = pca_mean_;
			const cv::Mat U = pca_basis_;
			const cv::Mat sv = pca_svals_;
			// Xmn
			cv::Mat Xmn = warped_I - M;
			cv::Mat Xmnproj = U.t() * Xmn;
			// y1
			double e1 = Xmn.dot(Xmn);
			double e2 = Xmnproj.dot(Xmnproj);
			cv::Mat y1(1, 1, CV_32F, e1 - e2);
			//y1 = y1 / M.rows;
			// y2
			cv::Mat ev, evinv, Xmnproj2;
			cv::pow(sv, 2, ev);
			cv::divide(1, ev, evinv);
			cv::pow(Xmnproj, 2, Xmnproj2);
			cv::Mat y2 = evinv.t() * Xmnproj2;
			// e
			o_dist.push_back(y1);
			o_dist.push_back(y2);
			// R
			o_R = params_.pca_params.R;

		}
		// need more accuracy
        //o_dist.convertTo(o_dist, CV_64F);
		o_R.convertTo(o_R, CV_64F);
	}

	void propose_particles(std::vector<Tracker::Particle> &i_particles, std::vector<Tracker::Particle> &o_particles) {  //担心矩阵归一化的问题？？？
		// importance density重要性概率密度函数: P(X_t | X_{t-1}) = N_{aff}( f(X_{t-1}), Q), Q = P * dt
		int n_particles = i_particles.size();
		o_particles.resize(n_particles);
		for (int i = 0; i < n_particles; ++i) {
			// f(X_{t-1})
			//cv::Mat X_prev = i_particles[i].state.X;
			//cv::Mat A_prev = i_particles[i].ar.A;
			//cv::Mat f_X_prev;
			//transit_state(X_prev, A_prev, f_X_prev);	//f_X_prev=X_prev*e^A_prev，并进行了归一化？？？
			//std::cout << "f_X_prev:" << f_X_prev << std::endl;
			// X_t
			//cv::Mat X1;
			cv::Mat X;
			switch (params_.state_domain) {
			case STATE_DOMAIN::Aff2:
			{
				my_mvnrnd_aff2(i_particles[i].state.X, i_particles[i].ar.A, params_.Q, params_.E, X);
				//mvnrnd_aff2(f_X_prev, params_.Q, params_.E, X);//输出X，方程（20）但是少了根号下的deltat
			}
				break;
			case STATE_DOMAIN::SL3:
			{
				cv::Mat X_prev = i_particles[i].state.X;
				cv::Mat A_prev = i_particles[i].ar.A;
				cv::Mat f_X_prev;
				transit_state(X_prev, A_prev, f_X_prev);	//f_X_prev=X_prev*e^A_prev，并进行了归一化？？？
				mvnrnd_sl3(f_X_prev, params_.Q, params_.E, X);
			}
				
				break;
			case STATE_DOMAIN::SE3:
			{
				cv::Mat X_prev = i_particles[i].state.X;
				cv::Mat A_prev = i_particles[i].ar.A;
				cv::Mat f_X_prev;
				transit_state(X_prev, A_prev, f_X_prev);	//f_X_prev=X_prev*e^A_prev，并进行了归一化？？？
				mvnrnd_se3(f_X_prev, params_.Q, params_.E, X);
			}
				break;
			}
			// A_t
			//std::cout << "X:" << X << std::endl;

			cv::Mat XX = i_particles[i].state.X.inv() * X;
			cv::Mat A;
			logm(XX, A);
			A *= params_.a;  //方程（9）
			// save the candidate particle 更新状态
			o_particles[i].state.X = X;
			o_particles[i].ar.A = A;
		}
	}

	void transit_state(cv::Mat &i_X, cv::Mat &i_A, cv::Mat &o_X) {
		// expm
		cv::Mat Aexp;
		expm(i_A, Aexp);	//Aexp=e^i_A
		//std::cout << "Aexp:" << Aexp << std::endl;
		// X * expm(A)
		o_X = i_X * Aexp;
		//float temp = o_X.at<float>(o_X.rows - 1, o_X.cols - 1);
		o_X /= o_X.at<float>(o_X.rows - 1, o_X.cols - 1); //FIXME: okay?   为什么要除以最后一个元素
		if (params_.state_domain == STATE_DOMAIN::SL3) {
			// det(X) = 1
			o_X = o_X / std::pow(cv::determinant(o_X), 1 / o_X.rows);
		}

	}

	void update_particle_weights(std::vector<Tracker::Particle> &i_particles, const cv::Mat &i_I, std::vector<float> &i_w, std::vector<float> &o_w) {
        // P(Y_t | X_t)`
		int n_particles = i_particles.size();
		o_w.resize(n_particles);
		float w_sum = 0, prob;
		//cv::Mat prob_n, prob_dn;  //这两个值后面都没有用到？
		for (int i = 0; i < n_particles; ++i) {
			cv::Mat prob_n, prob_dn;  //这两个值后面都没有用到？
			cv::Mat dist, R;
			eval_obs_mdl(i_I, i_particles[i].state.X, dist, R);	//更新dist R
			cv::exp(-dist.t() * R.inv() * dist / 2, prob_n);
			cv::sqrt(2 * 3.14 * cv::determinant(R), prob_dn);
            prob = prob_n.at<double>(0) / prob_dn.at<double>(0);  //??不应该是用相乘吗？
            //prob = prob_n.at<double>(0) * prob_dn.at<double>(0);
			//prob = cv::exp(-dist.at<double>(0) / R.at<double>(0, 0) * dist.at<double>(0) / 2);

			//std::cout << "dist(1) = " << dist.at<double>(0) << std::endl;
			//std::cout << "dist(2) = " << dist.at<double>(1) << std::endl;
			//std::cout << "prob(1) = " << cv::exp( - dist.at<double>(0) / R.at<double>(0, 0) * dist.at<double>(0) / 2) << std::endl;
			//std::cout << "prob(2) = " << cv::exp( - dist.at<double>(1) / R.at<double>(1, 1) * dist.at<double>(1) / 2) << std::endl;
			//std::cout << "prob = " << prob << std::endl << std::endl;
			//std::cout << "prob_n = " << prob_n << std::endl;

			o_w[i] = i_w[i] * prob;
			w_sum += o_w[i];
		}
		/*if (w_sum == 0) {
			std::cout << "w_sum == 0" << std::endl;
			cv::waitKey(0);
			std::runtime_error("the sum of particle weights are zero!");
		}*/
		// normalize
		for (int i = 0; i < n_particles; ++i) {
			o_w[i] /= w_sum;
		}
	}

	void resample_particles(std::vector<Tracker::Particle> &i_particles, std::vector<float> &i_w,std::vector<Tracker::Particle> &o_particles, std::vector<float> &o_w) {
		int n_particles = i_particles.size();
		o_particles.resize(n_particles);
		o_w.resize(n_particles);
		// resample
		std::vector<int> index(n_particles);
		resample(i_w, n_particles, index);
		// return
		float w_sum = 0;
		for (int i = 0; i < n_particles; ++i) {
			o_particles[i] = i_particles[index[i]];
			o_w[i] = i_w[index[i]];
			w_sum += o_w[i];
		}
		// normalize 权值并非1/n_particles
		for (int i = 0; i < n_particles; ++i) {
			o_w[i] /= w_sum;
		}
	}

	void find_X_opt(std::vector<Tracker::Particle> &i_particles, std::vector<float> &i_w, cv::Mat &o_X_opt) {
		switch (params_.state_domain) {
		case STATE_DOMAIN::Aff2:
		{
			int n_particles = i_particles.size();
			// find max ind
			float val_max = i_w[0];
			int ind_max = 0;
			for (int i = 1; i < n_particles; ++i) {
				if (val_max < i_w[i]) {
					ind_max = i;
				}
			}
			val_max = i_w[ind_max];
			cv::Mat G_max(2, 2, CV_32F, cv::Scalar(0));
			G_max.at<float>(0, 0) = i_particles[ind_max].state.X.at<float>(0, 0);
			G_max.at<float>(0, 1) = i_particles[ind_max].state.X.at<float>(0, 1);
			G_max.at<float>(1, 0) = i_particles[ind_max].state.X.at<float>(1, 0);
			G_max.at<float>(1, 1) = i_particles[ind_max].state.X.at<float>(1, 1);
			cv::Mat G_max_inv = G_max.inv();
			// U
			cv::Mat U(2, 2, CV_32F, cv::Scalar(0)), U_tmp;
			cv::Mat G(2, 2, CV_32F, cv::Scalar(0)); //FIXME: inefficient
			cv::Mat G_tmp = cv::Mat::eye(2, 2, CV_32F);
			for (int i = 0; i < n_particles; ++i) {
				cv::Mat X = i_particles[i].state.X;
				G.at<float>(0, 0) = X.at<float>(0, 0);
				G.at<float>(0, 1) = X.at<float>(0, 1);
				G.at<float>(1, 0) = X.at<float>(1, 0);
				G.at<float>(1, 1) = X.at<float>(1, 1);
				G_tmp = G_tmp*(G_max_inv * G);
			}
			logm(G_tmp, U_tmp);
			U /= float(n_particles);//除法外移
			// G_bar
			expm(U, U_tmp);
			cv::Mat G_bar = G_max * U_tmp;
			//cv::Mat G_bar = G_max;
			// t_bar
			float t1 = 0, t2 = 0;
			for (int i = 0; i < n_particles; ++i) {
				t1 += i_particles[i].state.X.at<float>(0, 2);//除法外移
				t2 += i_particles[i].state.X.at<float>(1, 2);//除法外移
			}
			t1 /=float(n_particles);//除法外移
			t2 /=float(n_particles);//除法外移
			// return
			o_X_opt = cv::Mat(3, 3, CV_32F, cv::Scalar(0));
			o_X_opt.at<float>(0, 0) = G_bar.at<float>(0, 0);
			o_X_opt.at<float>(0, 1) = G_bar.at<float>(0, 1);
			o_X_opt.at<float>(1, 0) = G_bar.at<float>(1, 0);
			o_X_opt.at<float>(1, 1) = G_bar.at<float>(1, 1);
			o_X_opt.at<float>(0, 2) = t1;
			o_X_opt.at<float>(1, 2) = t2;
			o_X_opt.at<float>(2, 2) = 1;
		}
			break;

		case STATE_DOMAIN::SL3:
		{
			double thres = 1e-4;
			int n_particles = i_particles.size();
			cv::Mat mu;
			i_particles[0].state.X.copyTo(mu);
			do {
				cv::Mat mu_inv = mu.inv();
				cv::Mat log_sum(mu.rows, mu.cols, CV_32F, cv::Scalar(0));
				for (int i = 0; i < n_particles; ++i) {
					cv::Mat dX = mu_inv * i_particles[i].state.X;
					cv::Mat log_dX;
					logm(dX, log_dX);
					log_sum += log_dX;
				}
				cv::Mat dmu;
				log_sum /= float(n_particles);
				expm(log_sum, dmu);
				mu *= dmu;
				cv::Mat log_dmu;
				logm(dmu, log_dmu);
				if (cv::norm(log_dmu.reshape(log_dmu.rows*log_dmu.cols)) < thres)
					break;
			} while (true);

			// return
			o_X_opt = mu;
			o_X_opt /= o_X_opt.at<float>(o_X_opt.rows - 1, o_X_opt.cols - 1); //FIXME: okay?
			// det(X) = 1
			o_X_opt = o_X_opt / std::pow(cv::determinant(o_X_opt), 1 / o_X_opt.rows);
		}
			break;

		case STATE_DOMAIN::SE3:
		{
			int n_particles = i_particles.size();
			// arithmetic mean of R, R_am
			cv::Mat R_am(3, 3, CV_32F, cv::Scalar(0));
			for (int r = 0; r < R_am.rows; ++r)
			for (int c = 0; c < R_am.cols; ++c)
			for (int i = 0; i<n_particles; ++i)
				R_am.at<float>(r, c) += i_particles[i].state.X.at<float>(r, c) / float(n_particles);
			// R_m
			cv::Mat R_amt = R_am.t();
			float d_a[9] = { 1, 0, 0, 0, 1, 0, 0, 0, -1 };
			cv::Mat d(3, 3, CV_32F, d_a);
			cv::Mat U, Vt, sv, R_m;
			cv::SVD::compute(R_amt, sv, U, Vt);
			if (cv::determinant(R_amt) > 0)
				R_m = Vt.t() * U.t();
			else
				//Rm = V * diag([1 1 -1]) * U';
				R_m = Vt.t() * d * U.t();

			// arithmetic mean of t, t_am
			cv::Mat t(3, 1, CV_32F, cv::Scalar(0));
			for (int i = 0; i < n_particles; ++i) {
				t.at<float>(0, 0) += i_particles[i].state.X.at<float>(0, 3) / float(n_particles);
				t.at<float>(1, 0) += i_particles[i].state.X.at<float>(1, 3) / float(n_particles);
				t.at<float>(2, 0) += i_particles[i].state.X.at<float>(2, 3) / float(n_particles);
			}
			// return
			cv::Mat last_row(1, 4, CV_32F, cv::Scalar(0));
			last_row.at<float>(0, 3) = 1;
			cv::Mat X_opt;
			cv::hconcat(R_m, t, X_opt);
			X_opt.push_back(last_row);
			o_X_opt = X_opt;
		}
			break;
		}
	}

	void load_params(const std::string &i_conf_fn) {
		std::ifstream conf_file(i_conf_fn); //以输入方式打开文件
		std::string line;
		// proj name 输出文件夹
		while (getline(conf_file, line)) {
			if (line[0] == '#' || line.size() == 0) continue;
			else {
				params_.id = line;
				//std::cout<<"length: "<<line.length()<<std::endl;
				break;
			}
		}

		// frame_dir
		while (std::getline(conf_file, line)) {
			if (line[0] == '#' || line.size() == 0) continue;
			else {
				params_.frame_dir = line;
				//std::cout<<line<<std::endl;
				break;
			}
		}
		// file_fmt
		while (std::getline(conf_file, line)) {
			if (line[0] == '#' || line.size() == 0) continue;
			else {
				params_.file_fmt = line;
				//std::cout<<line<<std::endl;
				break;
			}
		}

		// init_frame
		while (std::getline(conf_file, line)) {
			if (line[0] == '#' || line.size() == 0) continue;
			else {
				std::istringstream iss(line);  //字符串到整形变量
				iss >> params_.init_frame;
				//std::cout<<params_.init_frame<<std::endl;
				break;
			}
		}
		// start_frame
		while (std::getline(conf_file, line)) {
			if (line[0] == '#' || line.size() == 0) continue;
			else {
				std::istringstream iss(line);
				iss >> params_.start_frame;
				break;
			}
		}
		// end_frame
		while (std::getline(conf_file, line)) {
			if (line[0] == '#' || line.size() == 0) continue;
			else {
				std::istringstream iss(line);
				iss >> params_.end_frame;
				break;
			}
		}
		// fps
		while (std::getline(conf_file, line)) {
			if (line[0] == '#' || line.size() == 0) continue;
			else {
				std::istringstream iss(line);
				iss >> params_.fps;
				//std::cout<<params_.fps<<std::endl;
				break;
			}
		}
		// K
		while (std::getline(conf_file, line)) {
			if (line[0] == '#' || line.size() == 0) continue;
			else {
				std::istringstream iss(line);
				std::vector<float> K_vec(9);
				for (int i = 0; i < 9; ++i)
					iss >> K_vec[i];
				cv::Mat K(3, 3, CV_32F);
				int i_vec = 0;
				for (int i = 0; i < 3; ++i)
				for (int j = 0; j < 3; ++j){
					K.at<float>(i, j) = K_vec[i_vec++];
				//std::cout<<K_vec[i_vec-1]<<std::endl;	
				}
				params_.K = K;
				break;
			}
		}
		// n_particles
		while (std::getline(conf_file, line)) {
			if (line[0] == '#' || line.size() == 0) continue;
			else {
				std::istringstream iss(line);
				iss >> params_.n_particles;
				//std::cout<<params_.n_particles<<std::endl;
				break;
			}
		}
		// state_domain
		while (std::getline(conf_file, line)) {
			if (line[0] == '#' || line.size() == 0) continue;
			else {
				if (line.compare("Aff2") == 0)
					params_.state_domain = STATE_DOMAIN::Aff2;
				else if (line.compare("SL3") == 0)
					params_.state_domain = STATE_DOMAIN::SL3;
				else if (line.compare("SE3") == 0)
					params_.state_domain = STATE_DOMAIN::SE3;
				//else
					//std::cout<<"state_domain error"<<std::endl;
				break;
			}
		}
		// read basis
		int n_basis=0, n_dim=0;	//Ei的个数 Ei的维数
		switch (params_.state_domain) {
		case STATE_DOMAIN::Aff2:
			n_basis = 6, n_dim = 3;
			break;
		case STATE_DOMAIN::SL3:
			n_basis = 8, n_dim = 3;
			break;
		case STATE_DOMAIN::SE3:
			n_basis = 6, n_dim = 4;
			break;
		}
		for (int E_ind = 0; E_ind < n_basis; ++E_ind)
		{
			while (std::getline(conf_file, line)) {
				if (line[0] == '#' || line.size() == 0) continue;
				else {
					std::istringstream iss(line);
					std::vector<float> E_vec(n_dim * n_dim);
					for (int i = 0; i < n_dim*n_dim; ++i)
						iss >> E_vec[i];
					cv::Mat E(n_dim, n_dim, CV_32F);
					int i_vec = 0;
					for (int i = 0; i < n_dim; ++i)
					for (int j = 0; j < n_dim; ++j)
						E.at<float>(i, j) = E_vec[i_vec++];
					params_.E.push_back(E);
					break;
				}
			}
		}
		// a
		while (std::getline(conf_file, line)) {
			if (line[0] == '#' || line.size() == 0) continue;
			else {
				std::istringstream iss(line);
				iss >> params_.a;
           		//std::cout<<"a:"<<params_.a<<std::endl;
				break;
			}
		}
		// state_sigs
		while (std::getline(conf_file, line)) {
			if (line[0] == '#' || line.size() == 0) continue;
			else {
				std::istringstream iss(line);
				std::vector<float> sigs(n_basis);
				for (int i = 0; i < n_basis; ++i)
					iss >> sigs[i];
				cv::Mat P_sigs(n_basis, 1, CV_32F);
				for (int i = 0; i < n_basis; ++i)
					P_sigs.at<float>(i) = sigs[i];
				cv::Mat P_sigssq;
				cv::pow(P_sigs, 2, P_sigssq);	//幂运算 平方
				params_.P = cv::Mat::diag(P_sigssq);  //形成以P_sigssq为对角线的矩阵
				params_.Q = params_.P / params_.fps;
				break;
			}
		}
		// obs_mdl_type
		while (std::getline(conf_file, line)) {
			if (line[0] == '#' || line.size() == 0) continue;
			else {
				params_.obs_mdl_type = line;
				break;
			}
		}
		// R_SSD
		while (std::getline(conf_file, line)) {
			if (line[0] == '#' || line.size() == 0) continue;
			else {
				std::istringstream iss(line);
				float R_SSD;
				iss >> R_SSD;
				params_.ssd_params.R = cv::Mat(1, 1, CV_32F, R_SSD);
				break;
			}
		}
		// R_PCA
		while (std::getline(conf_file, line)) {
			if (line[0] == '#' || line.size() == 0) continue;
			else {
				std::istringstream iss(line);
				std::vector<float> R_PCA_vec(4);
				for (int i = 0; i < 4; ++i)
					iss >> R_PCA_vec[i];
				cv::Mat R_PCA(2, 2, CV_32F);
				int v_ind = 0;
				for (int i = 0; i < 2; ++i)
				for (int j = 0; j < 2; ++j)
					R_PCA.at<float>(i, j) = R_PCA_vec[v_ind++];
				params_.pca_params.R = R_PCA;
				
				break;
			}
		}
		// n_basis
		while (std::getline(conf_file, line)) {
			if (line[0] == '#' || line.size() == 0) continue;
			else {
				std::istringstream iss(line);
				iss >> params_.pca_params.n_basis;
				break;
			}
		}
		// update_interval
		while (std::getline(conf_file, line)) {
			if (line[0] == '#' || line.size() == 0) continue;
			else {
				std::istringstream iss(line);
				iss >> params_.pca_params.update_interval;
				break;
			}
		}
		// forget_factor
		while (std::getline(conf_file, line)) {
			if (line[0] == '#' || line.size() == 0) continue;
			else {
				std::istringstream iss(line);
				iss >> params_.pca_params.forget_factor;
				//std::cout<<"forget_factor:"<<params_.pca_params.forget_factor<<std::endl;
				break;
			}
		}
        //path of template image
//        while (getline(conf_file, line)) {
//            if (line[0] == '#' || line.size() == 0) continue;
//            else {
//                params_.template_img_dir= line;
//                break;
//            }
//        }
        while(std::getline(conf_file, line)) {
           if(line[0] == '#' || line.size() == 0) continue;
           else {
               // xs
               {
                   std::istringstream iss(line);
                   std::vector<float> template_xs;
                   float tmp;
                   while( iss >> tmp )
                       template_xs.push_back(tmp);
                   params_.template_xs = template_xs;
               }
               // ys
               {
                   std::getline(conf_file, line);
                   std::istringstream iss(line);
                   std::vector<float> template_ys;
                   float tmp;
                   while( iss >> tmp )
                       template_ys.push_back(tmp);
                   params_.template_ys = template_ys;
               }
               break;
           }
       }

	}

public:
	Tracker(const std::string &i_conf_fn) {
		load_params(i_conf_fn);
		//std::cout<<"load successed"<<std::endl;
//        if (load_template_params(params_.template_img_dir,params_.I_template_conners,params_.I_template_features)==false){
//            exit(-1);
//        }

        //hd
//        camera_factor = 1000;
//        camera_cx = 968.48745;
//        camera_cy = 537.77681;
//        camera_fx = 1068.8010;
//        camera_fy = 1067.9639;
        //qhd
//        camera_factor = 1000;
//        camera_cx = 482.45643;
//        camera_cy = 275.98007;
//        camera_fx = 533.30794;
//        camera_fy = 533.26216;
        //305 qhd
//        camera_factor = 1000;
//        camera_cx = 484.24373;
//        camera_cy = 268.88841;
//        camera_fx = 534.40049;
//        camera_fy = 533.98196;
	}
	Tracker(const Tracker::Params &i_params) {
		params_ = i_params;
        //hd
//        camera_factor = 1000;
//        camera_cx = 968.48745;
//        camera_cy = 537.77681;
//        camera_fx = 1068.8010;
//        camera_fy = 1067.9639;

        //qhd
//        camera_factor = 1000;
//        camera_cx = 482.45643;
//        camera_cy = 275.98007;
//        camera_fx = 533.30794;
//        camera_fy = 533.26216;
        //305qhd
//        camera_factor = 1000;
//        camera_cx = 484.24373;
//        camera_cy = 268.88841;
//        camera_fx = 534.40049;
//        camera_fy = 533.98196;
	}

	void read_inputs(int i_t, Tracker::Inputs &i_inputs, cv::Mat &I_ori) {
		char input_fn[1024];
		// rgb
		snprintf(input_fn, 1024, params_.file_fmt.c_str(), i_t); //将图片命名添加编号补全
        std::string rgb_fn = params_.frame_dir + std::string(input_fn) + std::string(".jpg"); //添加图片完整路径和后缀名称
        //std::cout<<"rgb_fn: "<<rgb_fn<<std::endl;
        I_ori = cv::imread(rgb_fn.c_str(), CV_LOAD_IMAGE_GRAYSCALE); //以灰度图格式载入原始图片
		I_ori.convertTo(i_inputs.I, CV_32F, 1.0 / 255.0);//把一个矩阵从一种数据类型转换到另一种数据类型，这里是对图像像素值归一化

		if (params_.state_domain == STATE_DOMAIN::SE3) {
			// depth
			std::string d_fn = params_.frame_dir + std::string(input_fn) + std::string("_depth.png");
			cv::Mat I_ori = cv::imread(rgb_fn.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
			I_ori.convertTo(i_inputs.I, CV_32F, 1.0 / 255.0);
			cv::Mat d_ori = cv::imread(d_fn.c_str(), CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
			d_ori.convertTo(i_inputs.D, CV_32F);
			// point clouds
			depth2pc(i_inputs.D, params_.K, i_inputs.C);
		}
	}
	void my_read_inputs(Tracker::Inputs &i_inputs){
		while(get_new_I==false && ros::ok()){
			ros::spinOnce();
		}	
		get_new_I=false;
		I_ORI.convertTo(i_inputs.I, CV_32F, 1.0 / 255.0);//把一个矩阵从一种数据类型转换到另一种数据类型，这里是对图像像素值归一化
	}	
	


	void Init(const Tracker::Inputs &i_inputs, cv::Mat &o_X_opt) {
		cv::Mat i_I = i_inputs.I; //i_I的图像为灰度图，且像素经过了归一化
		// init buffers
		particles.resize(params_.n_particles);  //粒子个数准备存储空间大小
		particles_res.resize(params_.n_particles);
		w.resize(params_.n_particles);
		w_res.resize(params_.n_particles);
		// create results dir
		//mkdir(params_.id.c_str(), S_IRWXU); //Linux 新建文件夹，可读可写可执行权限
#ifdef _MSC_VER
		_mkdir(params_.id.c_str());
#else
		mkdir(params_.id.c_str(), S_IRWXU);
#endif
        // check time
        timeval time_prev_;
        gettimeofday(&time_prev_,NULL);
        //clock_t time_prev_ = clock();
        time_prev_sec_=time_prev_.tv_sec+time_prev_.tv_usec/1000000.0;
        //time_prev_sec_ = double(time_prev_) / CLOCKS_PER_SEC;
		// get inital template
		int n_pnts = params_.template_xs.size(); //目标顶点个数
        //std::cerr << "- n_pnts = " << n_pnts << std::endl;//不经过缓冲而直接输出，一般用于迅速输出出错信息，是标准错误
		// ginput
		cv::Mat means;
		{
			cv::Mat xs(1, n_pnts, CV_32F, params_.template_xs.data()); //xs列向量
			cv::Mat ys(1, n_pnts, CV_32F, params_.template_ys.data()); //ys列向量

			cv::Scalar x_mean = cv::mean(xs);
			cv::Scalar y_mean = cv::mean(ys);

			if (params_.state_domain == STATE_DOMAIN::SE3) {
				int x_c = std::round(x_mean[0]);
				int y_c = std::round(y_mean[0]);
				means = cv::Mat(3, 1, CV_32F);
				means.at<float>(0) = i_inputs.C.at<float>(y_c, x_c, 0);
				means.at<float>(1) = i_inputs.C.at<float>(y_c, x_c, 1);
				means.at<float>(2) = i_inputs.C.at<float>(y_c, x_c, 2);

				cv::Mat template_poly_pnts(4, n_pnts, CV_32F);
				for (int i = 0; i < n_pnts; ++i) {
					int x = std::round(xs.at<float>(i));
					int y = std::round(ys.at<float>(i));

					template_poly_pnts.at<float>(0, i) = i_inputs.C.at<float>(y, x, 0) - means.at<float>(0);
					template_poly_pnts.at<float>(1, i) = i_inputs.C.at<float>(y, x, 1) - means.at<float>(1);
					template_poly_pnts.at<float>(2, i) = i_inputs.C.at<float>(y, x, 2) - means.at<float>(2);
					template_poly_pnts.at<float>(3, i) = 1;
				}
				template_poly_pnts_ = template_poly_pnts;

			}
			else {
				means.push_back(float(x_mean[0]));
				means.push_back(float(y_mean[0]));
				means.push_back(float(1.0));

				cv::Mat xs_c = xs.clone();
				cv::Mat ys_c = ys.clone();
				xs_c -= x_mean[0];
				ys_c -= y_mean[0];
				cv::Mat os = cv::Mat::ones(1, xs_c.cols, CV_32F);//全为1
				template_poly_pnts_.push_back(xs_c);
				template_poly_pnts_.push_back(ys_c);
				template_poly_pnts_.push_back(os);
			}
		}
		/*std::cerr << "- template means = " << means << std::endl;
		std::cerr << "- template_poly_pnts_.rows = " << template_poly_pnts_.rows;
		std::cerr << ", template_poly_pnts_.cols = " << template_poly_pnts_.cols << std::endl;
		std::cerr << "- template_poly_pnts_ = " << template_poly_pnts_ << std::endl;*/
		// template points
		{


			if (params_.state_domain == STATE_DOMAIN::SE3) {
                //原来在if外面
                std::vector<cv::Point> xys(n_pnts);//模板角点按点对存储
                for (int i = 0; i < n_pnts; ++i) {
                    cv::Point p((int)params_.template_xs[i], (int)params_.template_ys[i]);
                    xys[i] = p;
                }
                cv::Mat mask(i_I.rows, i_I.cols, CV_8U, cv::Scalar(0));
                const cv::Point* pnts[1] = { xys.data() };//xys存储数据的起始地址
                const int cnts[1] = { n_pnts };
                cv::fillPoly(mask, pnts, cnts, 1, cv::Scalar(255)); //模板区域多边形填充，填充为白色
                cv::imshow("mask",mask);

				// 3d points
				cv::Mat template_pnts;
				for (int c = 0; c < mask.cols; ++c) {
					for (int r = 0; r < mask.rows; ++r) {
						if (mask.at<unsigned char>(r, c) == 255) {
							float x = i_inputs.C.at<float>(r, c, 0);
							float y = i_inputs.C.at<float>(r, c, 1);
							float z = i_inputs.C.at<float>(r, c, 2);
							cv::Mat p(1, 4, CV_32F);
							p.at<float>(0, 0) = x - means.at<float>(0);
							p.at<float>(0, 1) = y - means.at<float>(1);
							p.at<float>(0, 2) = z - means.at<float>(2);
							p.at<float>(0, 3) = 1;
							template_pnts.push_back(p);
						}
					}
				}
				template_pnts_ = template_pnts.t();
			}
			else {
				// 2d points
                load_template_params_2D(I_ORI,params_.template_xs,params_.template_ys,params_.I_template_features);

                //cv::Mat init_frame_features;//模板图像中用于匹配的特征点映射到当前图像
                //cv::perspectiveTransform(cv::Mat(params_.I_template_features), init_frame_features, Htc);


                int n_features=params_.I_template_features.size();
                cv::Mat r_ind(1,n_features,CV_32F), c_ind(1,n_features,CV_32F);

                cv::Scalar x_mean(means.at<float>(0));
                cv::Scalar y_mean(means.at<float>(1));

                cv::Mat I_ORI_init=I_ORI.clone();
                for(int i=0;i<n_features;i++){
                    r_ind.at<float>(0,i)=params_.I_template_features[i].y-y_mean[0];
                    c_ind.at<float>(0,i)=params_.I_template_features[i].x-x_mean[0];
                    //cv::circle(I_ORI_init,cv::Point(params_.I_template_features[i].x,params_.I_template_features[i].y),1,cv::Scalar(255));
                    I_ORI_init.at<unsigned char>(params_.I_template_features[i].y,params_.I_template_features[i].x)=255;
                    //std::cout<<params_.I_template_features[i].x<<", "<<params_.I_template_features[i].y<<std::endl;
                }
                cv::circle(I_ORI_init,cv::Point(x_mean[0],y_mean[0]),3,cv::Scalar(0),2);
                cv::imshow("I_ORI_init",I_ORI_init);
                //cv::waitKey(0);
//				for (int c = 0; c < mask.cols; ++c)	//整幅图像的列数
//                    for (int r = 0; r < mask.rows; ++r)	{//整幅图像的行数
//                        if (mask.at<unsigned char>(r, c) == 255) {	//多边形模板区域内
//                            r_ind.push_back((float)r - (float)y_mean[0]);	//所在行-模板角点行均值
//                            c_ind.push_back((float)c - (float)x_mean[0]);	//所在列-模板角点行均值
//                        }
//                }
//				r_ind = r_ind.t();	//转置由nX1转为1Xn
//				c_ind = c_ind.t();

				cv::Mat os = cv::Mat::ones(1, c_ind.cols, CV_32F);
				template_pnts_.push_back(c_ind);
				template_pnts_.push_back(r_ind);
				template_pnts_.push_back(os);
			}
			// WHICH ONE IS CORRECT? MATLB? C?
            std::cerr << "- template_pnts_rows = " << template_pnts_.rows << std::endl;
			std::cerr << "- template_pnts_cols = " << template_pnts_.cols << std::endl;
			//std::cerr << "- template_pnts_(0, 0) = " << template_pnts_.at<float>(0, 0) << std::endl;	//这两行用来看什么？
			//std::cerr << "- template_pnts_(0, 5) = " << template_pnts_.at<float>(0, 5) << std::endl;
		}
		// init X
		int mat_dim = params_.E[0].rows;//E[0] 3X3
		cv::Mat X0(mat_dim, mat_dim, CV_32F, cv::Scalar(0));
		{
			for (int i = 0; i < mat_dim; ++i) {
				X0.at<float>(i, i) = 1.0f;
				X0.at<float>(i, mat_dim - 1) = means.at<float>(i);//每行最后一列为均值
			}
			X0.at<float>(mat_dim - 1, mat_dim - 1) = 1;//保证对角线为1
			state_.X = X0;
		}
		//std::cerr << "- X0 = " << X0 << std::endl;
		// init particles 一开始粒子都在模板的几何中心
		{
			particles_.resize(params_.n_particles);
			cv::Mat A0 = cv::Mat::zeros(mat_dim, mat_dim, CV_32F);
			for (int i = 0; i < particles_.size(); ++i) {
				particles_[i].state.X = X0;
				particles_[i].ar.A = A0;
			}
		}
		// init w
		w_ = std::vector<float>(params_.n_particles, 1 / float(params_.n_particles));
		// init obs model
		learn_obs_mdl(i_I, state_.X);
		// return
		o_X_opt = state_.X;
	}

	void Track(const Tracker::Inputs &i_inputs, const int i_t, cv::Mat &o_X_opt) {
		cv::Mat i_I = i_inputs.I;
		// propose new Xt 重要性采样，更新particles
		propose_particles(particles_, particles);
		//std::cerr << "-propose_particles: " << std::endl;
		//绘制所有跟踪框
		//myShow(I_ORI, particles, i_t, 1);
		// update weights 更新w
		update_particle_weights(particles, i_I, w_, w);
		//std::cerr << "- sample w after update_particle_weights(): " << w[0] << std::endl;
		// resampling 更新particles_res w_res
		resample_particles(particles, w, particles_res, w_res);
		//std::cerr << "- sample w after resample_particles(): " << w_res[0] << std::endl;
		// find the optimum state 最优化估计
		cv::Mat X_opt;
		find_X_opt(particles_res, w_res, X_opt);
		// learn obs mdl
		//learn_obs_mdl(i_I, X_opt);
		// update X, particles, w
		state_.X = X_opt;
		particles_ = particles_res;
		//myShow(I_ORI, particles_, i_t, 0);
		w_ = w_res;
		// return
		o_X_opt = X_opt;
		//std::cerr << "- X_opt: " << X_opt << std::endl;
	}

	void Show(cv::Mat &I, const cv::Mat &i_X, const int i_t, const std::string i_fn_out) {
        timeval time_cur;
        gettimeofday(&time_cur,NULL);
        double time_cur_sec=time_cur.tv_sec+time_cur.tv_usec/1000000.0;
        //clock_t time_cur = clock();
        //double time_cur_sec = double(time_cur) / CLOCKS_PER_SEC;
		double fps = 1.0 / (time_cur_sec - time_prev_sec_);
        //time_prev_sec_ = time_cur_sec;
		// draw X
		cv::Mat poly;
		if (params_.state_domain == STATE_DOMAIN::SE3) {
			cv::Mat X_rot = i_X * template_poly_pnts_;
			poly = params_.K * X_rot.rowRange(0, 3);
		}
		else {
			poly = i_X * template_poly_pnts_; //还原顶点
		}
		//std::cout << "i_x:" << i_X << std::endl;
		std::vector<cv::Point> pnts;
		for (int i = 0; i < poly.cols; ++i) {
			cv::Point p((int)std::round(poly.at<float>(0, i)),(int)std::round(poly.at<float>(1, i)));
			pnts.push_back(p);
		}
		const cv::Point *pts[1] = { pnts.data() };
		const int npts[1] = { (int)pnts.size() };
		polylines(I, pts, npts, 1, true, cv::Scalar(255, 0, 0), 2); //绘制包围多边形
		
		// draw t ::FIXME: access to private vars
		//int n_particles = particles_.size();
		//float val_max = w_[0];
		//int ind_max = 0;
		//for (int i = 1; i < n_particles; ++i) {	//寻找权重最大的值
		//	if (val_max < w_[i]) {
		//		val_max = w_[i];
		//		ind_max = i;
		//	}
		//}
		//for (int i = 0; i < n_particles; ++i) {
		//	float tx, ty;
		//	if (params_.state_domain == STATE_DOMAIN::SE3) {
		//		cv::Mat t = particles_[i].state.X.col(3);
		//		cv::Mat t_proj = params_.K * t.rowRange(0, 3);
		//		tx = t_proj.at<float>(0) / t_proj.at<float>(2);
		//		ty = t_proj.at<float>(1) / t_proj.at<float>(2);
		//	}
		//	else {
		//		tx = particles_[i].state.X.at<float>(0, 2) / particles_[i].state.X.at<float>(2, 2);//（2,2）不是为1吗？为什么还要除？是不是有哪里不为1的
		//		ty = particles_[i].state.X.at<float>(1, 2) / particles_[i].state.X.at<float>(2, 2);
		//	}
		//	cv::Point p(tx, ty);
		//	int r = std::max(1.0, w_[i] / val_max * 3.0);
		//	circle(I, p, r, cv::Scalar(255));//画一堆多边形中心
		//}

		cv::Point2i p((int)i_X.at<float>(0,2), (int)i_X.at<float>(1,2));
		circle(I, p, 2, cv::Scalar(255),-1);//画多边形中心


        //防止边缘点超出物品边界，收缩为原来的1/5
//		std::vector<cv::Point2i> small_P2d(4);
//		small_P2d[0]=p;
//		for(int i=1;i<4;i++){
//			small_P2d[i].x=p.x+(int)(pnts[i].x-pnts[0].x)/5;
//			small_P2d[i].y=p.y+(int)(pnts[i].y-pnts[0].y)/5;
//		}

//		std::vector<cv::Mat> small_P3f(4);
//		bool depth_avail=true; //深度图读取是否有值
//		for(int i=0;i<4;i++){
//			// 获取深度图点处的值
//            //std::cout<<"small_P2d[i].y "<< small_P2d[i].y <<std::endl;
//            //std::cout<<"small_P2d[i].x "<< small_P2d[i].x <<std::endl;
//        	ushort d = I_ORI_DEPTH.ptr<ushort>(small_P2d[i].y)[small_P2d[i].x];
//			if(d<=0 || d>3000){
//				depth_avail=false;
//				std::cout<<"depth error"<< i <<": "<<d<<std::endl;
//				break;
//			}
//			//计算空间坐标
//			cv::Mat temp_point=cv::Mat(1,3,CV_32FC1);
//			temp_point.at<float>(0,2)=float(d) / camera_factor;
//			temp_point.at<float>(0,0) = (small_P2d[i].x- camera_cx) * temp_point.at<float>(0,2) / camera_fx;
//        	temp_point.at<float>(0,1) = (small_P2d[i].y - camera_cy) * temp_point.at<float>(0,2) / camera_fy;
//			small_P3f[i]=temp_point.clone();
//		}
//        //输出缩小后的四个点在相机坐标系下的坐标
//        //std::cout<<std::endl;
//        //std::cout<<"4 points of camera fram"<<std::endl;
////        for(int i=0;i<4;i++)
////        {
////            std::cout<<small_P3f[i]<<std::endl;
////        }
//		if(depth_avail){
//			std::vector<cv::Mat> obj_xyz(3);//物体坐标系
//			obj_xyz[0]=small_P3f[far_point]-small_P3f[0];
//			cv::normalize(obj_xyz[0],obj_xyz[0]);
//            //std::cout<<"obj_xyz[0] normalize"<<std::endl<<obj_xyz[0]<<std::endl;

//			if(near_point<0){
//                obj_xyz[1]=small_P3f[-near_point]-small_P3f[0];
//				obj_xyz[2]=obj_xyz[1].cross(obj_xyz[0]);
//                //std::cout<<"near_point<0"<<std::endl;
//			}
//			else{
//				obj_xyz[1]=small_P3f[near_point]-small_P3f[0];
//				obj_xyz[2]=obj_xyz[0].cross(obj_xyz[1]);
//                //std::cout<<"near_point>0"<<std::endl;
//			}
//			cv::normalize(obj_xyz[2],obj_xyz[2]);
//            //std::cout<<"obj_xyz[2] normalize"<<std::endl<<obj_xyz[2]<<std::endl;

//			obj_xyz[1]=obj_xyz[2].cross(obj_xyz[0]);
//			cv::normalize(obj_xyz[1],obj_xyz[1]);
//            //std::cout<<"obj_xyz[1] normalize"<<std::endl<<obj_xyz[1]<<std::endl;

//            /*********求解RT**********/
//            //投影到物体坐标系
//			cv::Mat obj_mean=cv::Mat::zeros(3,1,CV_32FC1);
//			cv::Mat obj_points=cv::Mat::zeros(4,3,CV_32FC1);
//			for(int i=1;i<4;i++){
//				for(int j=0;j<3;j++){
//					cv::Mat temp=small_P3f[i]-small_P3f[0];
//					obj_points.at<float>(i,j)=temp.dot(obj_xyz[j]);
//				}
//				obj_mean.at<float>(0,0)+=obj_points.at<float>(i,0);
//				obj_mean.at<float>(0,1)+=obj_points.at<float>(i,1);
//				obj_mean.at<float>(0,2)+=obj_points.at<float>(i,2);
//			}
//            //std::cout<<"obj_points"<<std::endl<<obj_points<<std::endl;
//			obj_mean/=4;
//			for(int i=0;i<4;i++){
//				obj_points.at<float>(i,0)-=obj_mean.at<float>(0,0);
//				obj_points.at<float>(i,1)-=obj_mean.at<float>(0,1);
//				obj_points.at<float>(i,2)-=obj_mean.at<float>(0,2);
//			}

//			cv::Mat cam_mean=cv::Mat::zeros(3,1,CV_32FC1);
//			cv::Mat cam_points=cv::Mat(3,4,CV_32FC1);
//			for(int j=0;j<4;j++){
//				for(int i=0;i<3;i++){
//					cam_points.at<float>(i,j)=small_P3f[j].at<float>(0,i);
//				}
//				cam_mean.at<float>(0,0)+=cam_points.at<float>(0,j);
//				cam_mean.at<float>(0,1)+=cam_points.at<float>(1,j);
//				cam_mean.at<float>(0,2)+=cam_points.at<float>(2,j);
//			}
//            //std::cout<<"cam_points"<<std::endl<<cam_points<<std::endl;
//			cam_mean/=4;
//			for(int j=0;j<4;j++){
//				cam_points.at<float>(0,j)-=cam_mean.at<float>(0,0);
//				cam_points.at<float>(1,j)-=cam_mean.at<float>(0,1);
//				cam_points.at<float>(2,j)-=cam_mean.at<float>(0,2);
//			}
		
//			cv::Mat H=cam_points*obj_points;
//            //std::cout << "-H:" << std::endl << H << std::endl;
//			// svd(H)
//			cv::Mat u, v, s;
//			cv::SVD::compute(H, s, u, v); //这里求得的v是vt
//			v=v.t();
//			//std::cerr << "- size(u): " << u.rows << ", " << u.cols << std::endl;
//			//std::cerr << "- s: " << s << std::endl;
//			//std::cerr << "- v: " << v << std::endl;
//			cv::Mat r=v*u.t();
//			if(cv::determinant(r)<0){
//                //std::cout<<"reflection detected"<<std::endl;
//				v.at<float>(0,2)*=-1;
//				v.at<float>(1,2)*=-1;
//				v.at<float>(2,2)*=-1;
//				r=v*u.t();
//			}
//            cv::Mat t=r*cam_mean+obj_mean;
//            //std::cout<<"t:"<<std::endl<<t<<std::endl;
//            //std::cout << "-R:" << std::endl << r << std::endl;
//            //std::cout<<"-T:"<<std::endl<<small_P3f[0]<<std::endl;

//            {//if(i_t>=3)
//                cv::Mat r_t=r.t();
//                //计时
//                /*timeval time_pub;
//                gettimeofday(&time_pub,NULL);
//                double time_pub_sec=time_pub.tv_sec+time_pub.tv_usec/1000000.0;
//                //clock_t time_pub = clock();
//                //time_pub_sec = double(time_pub) / CLOCKS_PER_SEC;
//                std::cout<<"time pub r_t: "<<time_pub_sec-time_cur_sec<<std::endl;
//                std::cout<<"time pub ==: "<<time_pub_sec<<std::endl;*/

//                pub_position(small_P3f[0],r_t);

//                //std::cout<<"pub_position"<<std::endl;
//                //cv::waitKey(0);
//                //exit(-1);
//            }

//			char position[40];
//			snprintf(position, 40, "[%.03f,%.03f,%.03f]", small_P3f[0].at<float>(0,0), small_P3f[0].at<float>(0,1), small_P3f[0].at<float>(0,2));
//            cv::putText(I, position, cv::Point(p.x+50, p.y+50), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255),2);
//		}
//        //绘制方向向量
//        circle(I, small_P2d[0], 2, cv::Scalar(255),-1);
//        line(I,small_P2d[0],small_P2d[far_point],cv::Scalar(0),2);
//        line(I,small_P2d[0],small_P2d[abs(near_point)],cv::Scalar(100),2);


		// add a title
		char title[20];
		snprintf(title,20, "%.02f fps", fps);
		cv::putText(I, title, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255),2);
		// show
		cv::imshow("tracking...", I);
		cv::waitKey(1);

        //计时
        /*timeval time_imshow;
        gettimeofday(&time_imshow,NULL);
        double time_imshow_sec=time_imshow.tv_sec+time_imshow.tv_usec/1000000.0;
        //clock_t time_imshow = clock();
        //double time_imshow_sec = double(time_imshow) / CLOCKS_PER_SEC;
        std::cout<<"time imshow: "<<time_imshow_sec-time_prev_sec_<<std::endl;*/
        time_prev_sec_ = time_cur_sec;

		// write results
        //cv::imwrite(i_fn_out, I);
	}

	void myShow(const cv::Mat &i_I, const std::vector<Tracker::Particle> i_particles, const int i_t, bool track) {
		cv::Mat I = i_I.clone();
		// draw X
		for (int k = 0; k < i_particles.size(); k++)
		{
			cv::Mat poly;
			cv::Mat i_X = i_particles[k].state.X;
			if (params_.state_domain == STATE_DOMAIN::SE3) {
				cv::Mat X_rot = i_X * template_poly_pnts_;
				poly = params_.K * X_rot.rowRange(0, 3);
			}
			else {
				poly = i_X * template_poly_pnts_; //还原顶点
			}
			std::vector<cv::Point> pnts;
			for (int i = 0; i < poly.cols; ++i) {
				cv::Point p((int)std::round(poly.at<float>(0, i)),(int)std::round(poly.at<float>(1, i)));
				pnts.push_back(p);
			}
			const cv::Point *pts[1] = { pnts.data() };
			const int npts[1] = { (int)pnts.size() };
			polylines(I, pts, npts, 1, true, cv::Scalar(255, 0, 0), 1); //绘制包围多边形
			//cv::waitKey(1);
		}
		// show
		cv::imshow("checking...", I);
		cv::waitKey(1);
		// write results
		std::string i_fn_out;
		if (track)
		{
			i_fn_out = params_.id + "/" + std::to_string(i_t) + std::string("_1") + std::string(".png"); //输出图片文件路径和文件名
		}
		else
		{
			i_fn_out = params_.id + "/" + std::to_string(i_t) + std::string("_2") + std::string(".png"); //输出图片文件路径和文件名
		}
		//cv::imwrite(i_fn_out, I);
		return;
	}



	void Run() {
		// track
        for (unsigned int t = params_.start_frame; t<params_.end_frame; t++) {
            std::string fn_out = params_.id + std::string("/") + std::to_string(t) + std::string(".jpg"); //输出图片文件路径和文件名
            //std::cout<<"fn_out:"<<fn_out<<std::endl;
			Tracker::Inputs I;  //像素归一化后的图像
            read_inputs(t, I, I_ORI);	//I的图像为灰度图，且像素经过了归一化
            //my_read_inputs(I);
			if (I.I.empty())
			{
				std::cout << "读图失败" << std::endl;
			}
//            cv::imshow("test", I.I);
//            cv::waitKey(0);

			cv::Mat X_opt;
            if (t == params_.init_frame){
//                if(!autoget_template_poly_pnts(params_.I_template_conners,params_.template_xs, params_.template_ys, far_point, near_point,1)){
//                    std::cout<<"get_template_poly_pnts ellor"<<std::endl;
//                    t=0;
//                    continue;
//                }
//                if(get_template_poly_pnts(params_.template_xs, params_.template_ys, far_point, near_point)==false){
//					std::cout<<"get_template_poly_pnts ellor"<<std::endl;
//					return;
//				}
				/*
				std::cout<<"params_.template_xs: ";
				for(int i=0;i<params_.template_xs.size();i++){
					std::cout<<params_.template_xs[i]<<" ";
				}
				std::cout<<std::endl<<"params_.template_ys: ";
				for(int i=0;i<params_.template_ys.size();i++){
					std::cout<<params_.template_ys[i]<<" ";
				}
				std::cout<<std::endl;*/
				Init(I, X_opt);		//对于彩图，X_opt为3X3向量，对角线为1，最后一列前两行为模板顶点X和Y的均值，粒子都在中心点
                //std::cout<<"init finished"<<std::endl;
			}
			else
				Track(I, t, X_opt);

			Show(I_ORI, X_opt, t, fn_out);
            //cv::waitKey(0);
			//std::cout<<"show finished"<<std::endl;
		}
	}
};


int main(int argc, char** argv) {
//    ros::init(argc, argv, "gpf");
//    ros::NodeHandle nh;
//    message_filters::Subscriber<sensor_msgs::Image> image_rgb_sub(nh, "/kinect2/qhd/image_color_rect", 1);
//    message_filters::Subscriber<sensor_msgs::Image>image_depth_sub(nh, "/kinect2/qhd/image_depth_rect", 1);
//    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(image_rgb_sub, image_depth_sub, 10);
//    sync.registerCallback(boost::bind(&RecognitionCallback, _1, _2));
//    colorimg_sub=nh.subscribe("/kinect2/qhd/image_color_rect",1,colorimgSubCallback);
//    position_publisher=nh.advertise<gpf::obj_tool_transform>("/gpf/position", 1, true);

	
	std::string file=argv[1];//std::string("/home/qcrong/thesis_ws/src/gpf/cereal_test_Aff2.conf");
	//std::string file = std::string("/home/qcrong/thesis_ws/src/gpf/cereal_test_Aff2.conf");
	Tracker tracker(file);
	

	tracker.Run();
    feature.release();
    descript.release();
	return 0;
}
