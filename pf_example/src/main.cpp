#include <stdio.h>
#include <string.h>
#include <ctype.h>	//C标准函数库中的头文件
#include "defs.h"
#include "utils.h"
#include "particles.h"
#include "observation.h"
#include <opencv2/opencv.hpp>
#include <time.h>

#pragma warning( disable:4996) 
#pragma comment (lib, "libgsl.a")  //表示链接libgsl.a这个库,和在工程设置里写上链入libgsl.a的效果一样（两种方式等价，或说一个隐式一个显式调用）
/******************************** Definitions ********************************/

///* command line options */
//#define OPTIONS ":p:oah"

/* default number of particles */
#define PARTICLES 100

/* default basename and extension of exported frames */
//#define EXPORT_BASE "./frames/frame_"
//#define EXPORT_EXTN ".png"

/* maximum number of frames for exporting */
#define MAX_FRAMES 2048

/********************************* Structures ********************************/

/* maximum number of objects to be tracked */
#define MAX_OBJECTS 1  //如果是多个目标，后面程序要多加个for循环

typedef struct params {
	CvPoint loc1[MAX_OBJECTS];
	CvPoint loc2[MAX_OBJECTS];
	IplImage* objects[MAX_OBJECTS];
	char* win_name;
	IplImage* orig_img;
	IplImage* cur_img;
	int n;
} params;


/***************************** Function Prototypes ***************************/

int get_regions(IplImage*, CvRect**);
void mouse(int, int, int, int, void*);
histogram** compute_ref_histos(IplImage*, CvRect*, int);
int export_ref_histos(histogram**, int);



/********************************** Globals **********************************/
char* vid_file = "/home/qcrong/thesis_ws/src/pf_example/test.mp4";     /* input video file name */
int num_particles = PARTICLES;    /* 粒子个数 */
int show_all = 1;                 /* =1 显示每个粒子（黄色框）；=0 只显示跟踪窗口 */
int export1 = FALSE;               /* =FALSE 不输出特征（直方图），=TRUE 输出特征（直方图），保存为.dat文件 */

//static CvMemStorage* storage = 0;

// Create a new Haar classifier  opencv人脸检测
//static CvHaarClassifierCascade* cascade = 0;

// Function prototype for detecting and drawing an object from an image
void detect_and_draw(IplImage* image);

// Create a string that contains the cascade name
//const char* cascade_name = "haarcascade_fullbody.xml";
/*********************************** Main ************************************/

int main(int argc, char** argv)
{
	gsl_rng* rng;
	IplImage* frame, *hsv_frame, *frames[MAX_FRAMES];
	IplImage** hsv_ref_imgs;
	histogram** ref_histos;
	CvCapture* video;
	particle* particles = NULL, *new_particles;
	CvScalar color;
	CvRect* regions;
	int num_objects = 0;
	double s;
	int i, j, k, w, h, x, y;
	char path[255];
	/* parse command line and initialize random number generator 解析命令行并初始化随机数生成器 */
	gsl_rng_env_setup();  //建立随机数生成器环境
	rng = gsl_rng_alloc(gsl_rng_mt19937); //随机数生成器的创建
	gsl_rng_set(rng, time(NULL));//随机数生成器的初始化

	//video = cvCaptureFromFile(vid_file); //从视频文件读图像
	video = cvCaptureFromCAM(0);       //直接从摄像头读图像
	if (!video)
		fatal_error("couldn't open video file %s", vid_file);
	i = 0;

	//cascade = (CvHaarClassifierCascade*)cvLoad( cascade_name, 0, 0, 0 );//加载行人检测分类器
	//storage = cvCreateMemStorage(0);
	while (frame = cvQueryFrame(video))
	{
		if (!frame)
		{
			break;
		}
		hsv_frame = bgr2hsv(frame);
		frames[i] = cvCloneImage(frame);

		/* allow user to select object to be tracked in the first frame */
		if (num_objects == 0)
		{
			w = frame->width;
			h = frame->height;
			fprintf(stderr, "Select object region to track\n");
			/*
			//frame = cvLoadImage("E:/csource/Peopledetect/s1.JPG",0);
			CvSeq* object = cvHaarDetectObjects( frame, cascade, storage,1.05,2,
			CV_HAAR_FIND_BIGGEST_OBJECT,cvSize(30, 60),cvSize(w,h) );
			num_objects = object->total;
			printf("%d pedestrian have been detected!\n",num_objects);
			//如果检测到目标，则获取目标区域
			if(num_objects > 0)
			{
			regions = (CvRect*)cvGetSeqElem( object, 0 );
			regions->y -=  regions->height/1.5;
			if(regions->y < 0)
			regions->y = 0;
			regions->height = regions->height*1.5;
			CvPoint pt1, pt2;
			pt1.x = regions->x;
			pt2.x = (regions->x+regions->width);
			pt1.y = regions->y;
			pt2.y = (regions->y+regions->height);

			// Draw the rectangle in the input image
			cvRectangle( frame, pt1, pt2, CV_RGB(255,0,0), 3, 8, 0 );
			cvShowImage("haar detected result",frame);
			cvWaitKey(0);
			}

			else
			{
			//鼠标手动获取跟踪区域
			fprintf( stderr, "Please select a object\n" );
			num_objects = get_regions( frame, &regions );
			}*/
			num_objects = get_regions(frame, &regions);
			if (num_objects > 0)
			{
				/* compute reference histograms and distribute particles */
				ref_histos = compute_ref_histos(hsv_frame, regions, num_objects);
				if (export1)
					export_ref_histos(ref_histos, num_objects);
				particles = init_distribution(regions, ref_histos, num_objects, num_particles);
			}

		}
		else
		{
			//检测到目标后，进行粒子滤波进行跟踪
			/* perform prediction and measurement for each particle */
			for (j = 0; j < num_particles; j++)
			{
				particles[j] = transition(particles[j], w, h, rng);
				s = particles[j].s;
				particles[j].w = likelihood(hsv_frame, cvRound(particles[j].y),
					cvRound(particles[j].x),
					cvRound(particles[j].width * s),
					cvRound(particles[j].height * s),
					particles[j].histo);
			}

			/* normalize weights and resample a set of unweighted particles */
			normalize_weights(particles, num_particles);
			new_particles = resample(particles, num_particles);
			free(particles);
			particles = new_particles;
			//	}

			/* display all particles if requested */
			qsort(particles, num_particles, sizeof(particle), &particle_cmp);
			if (show_all)
			for (j = num_particles - 1; j > 0; j--)
			{
				color = CV_RGB(255, 255, 0);
				display_particle(frames[i], particles[j], color);
			}

			/* display most likely particle */
			color = CV_RGB(255, 0, 0);
			display_particle(frames[i], particles[0], color);
			cvNamedWindow("Video", 1);
			cvShowImage("Video", frames[i]);
			//保存跟踪结果图像为jpg格式
			sprintf(path, "%s%d.jpg", "D:\\test", i);
			cvSaveImage(path, frames[i], 0);
			if (cvWaitKey(5) == 27)
				break;
		}
		cvReleaseImage(&hsv_frame);

		i++;
	}
	cvReleaseCapture(&video);
	//cvReleaseHaarClassifierCascade( &cascade );
	//cvReleaseMemStorage(&storage);
}


/************************** Function Definitions *****************************/
/*
  Allows the user to interactively select object regions.

  @param frame the frame of video in which objects are to be selected
  @param regions a pointer to an array to be filled with rectangles
  defining object regions

  @return Returns the number of objects selected by the user
  */
int get_regions(IplImage* frame, CvRect** regions)
{
	char* win_name = "First frame";
	params p;
	CvRect* r;
	int i, x1, y1, x2, y2, w, h;

	/* use mouse callback to allow user to define object regions */
	p.win_name = win_name;
	p.orig_img = cvCloneImage(frame);
	p.cur_img = NULL;
	p.n = 0;
	cvNamedWindow(win_name, 1);
	cvShowImage(win_name, frame);
	cvSetMouseCallback(win_name, &mouse, &p);
	cvWaitKey(0);
	cvDestroyWindow(win_name);
	cvReleaseImage(&(p.orig_img));
	if (p.cur_img)
		cvReleaseImage(&(p.cur_img));

	/* extract regions defined by user; store as an array of rectangles */
	if (p.n == 0)
	{
		*regions = NULL;
		return 0;
	}
	r = (CvRect *)malloc(p.n * sizeof(CvRect));
	for (i = 0; i < p.n; i++)
	{
		x1 = MIN(p.loc1[i].x, p.loc2[i].x);
		x2 = MAX(p.loc1[i].x, p.loc2[i].x);
		y1 = MIN(p.loc1[i].y, p.loc2[i].y);
		y2 = MAX(p.loc1[i].y, p.loc2[i].y);
		w = x2 - x1;
		h = y2 - y1;

		/* ensure odd width and height 确保奇数宽度和高度???*/
		w = (w % 2) ? w : w + 1;
		h = (h % 2) ? h : h + 1;
		r[i] = cvRect(x1, y1, w, h);
	}
	*regions = r;
	return p.n;
}



/*
  Mouse callback function that allows user to specify the initial object
  regions.  Parameters are as specified in OpenCV documentation.
  */
void mouse(int event, int x, int y, int flags, void* param)
{
	params* p = (params*)param;
	CvPoint* loc;
	int n;
	IplImage* tmp;
	static int pressed = FALSE;

	/* on left button press, remember first corner of rectangle around object */
	if (event == CV_EVENT_LBUTTONDOWN)
	{
		n = p->n;
		if (n == MAX_OBJECTS)
			return;
		loc = p->loc1;
		loc[n].x = x;
		loc[n].y = y;
		pressed = TRUE;
	}

	/* on left button up, finalize the rectangle and draw it in black */
	else if (event == CV_EVENT_LBUTTONUP)
	{
		n = p->n;
		if (n == MAX_OBJECTS)
			return;
		loc = p->loc2;
		loc[n].x = x;
		loc[n].y = y;
		cvReleaseImage(&(p->cur_img));
		p->cur_img = NULL;
		cvRectangle(p->orig_img, p->loc1[n], loc[n], CV_RGB(0, 0, 0), 1, 8, 0);
		cvShowImage(p->win_name, p->orig_img);
		pressed = FALSE;
		p->n++;
	}

	/* on mouse move with left button down, draw rectangle as defined in white */
	else if (event == CV_EVENT_MOUSEMOVE  &&  flags & CV_EVENT_FLAG_LBUTTON)
	{
		n = p->n;
		if (n == MAX_OBJECTS)
			return;
		tmp = cvCloneImage(p->orig_img);
		loc = p->loc1;
		cvRectangle(tmp, loc[n], cvPoint(x, y), CV_RGB(255, 255, 255), 1, 8, 0);
		cvShowImage(p->win_name, tmp);
		if (p->cur_img)
			cvReleaseImage(&(p->cur_img));
		p->cur_img = tmp;
	}
}



/*
  Computes a reference histogram for each of the object regions defined by
  the user 计算用户定义的每个对象区域的参考直方图

  @param frame video frame in which to compute histograms; should have been
  converted to hsv using bgr2hsv in observation.h
  @param regions regions of \a frame over which histograms should be computed
  @param n number of regions in \a regions
  @param export if TRUE, object region images are exported

  @return Returns an \a n element array of normalized histograms corresponding
  to regions of \a frame specified in \a regions.
  */
histogram** compute_ref_histos(IplImage* frame, CvRect* regions, int n)
{
	histogram** histos = (histogram **)malloc(n * sizeof(histogram*));
	IplImage* tmp;
	int i;

	/* extract each region from frame and compute its histogram */
	for (i = 0; i < n; i++)
	{
		cvSetImageROI(frame, regions[i]);
		tmp = cvCreateImage(cvGetSize(frame), IPL_DEPTH_32F, 3);
		cvCopy(frame, tmp, NULL);
		cvResetImageROI(frame);
		histos[i] = calc_histogram(&tmp, 1);
		normalize_histogram(histos[i]);
		cvReleaseImage(&tmp);
	}

	return histos;
}



/*
  Exports reference histograms to file

  @param ref_histos array of reference histograms
  @param n number of histograms

  @return Returns 1 on success or 0 on failure
  */
int export_ref_histos(histogram** ref_histos, int n)
{
	char name[32];
	char num[3];
	FILE* file;
	int i;

	for (i = 0; i < n; i++)
	{
		sprintf(num, "%02d", i);
		strcpy(name, "hist_");
		strcat(name, num);
		strcat(name, ".dat");
		if (!export_histogram(ref_histos[i], name))
			return 0;
	}

	return 1;
}

