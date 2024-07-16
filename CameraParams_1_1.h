#pragma once
//从电脑面向二代装置，左边的相机57，右边的相机82
#include <opencv2/opencv.hpp>

//摄像头的分辨率（图形尺寸：宽度）
static const int IMAGEWIDTH = 1280;
//摄像头的分辨率（图形尺寸：高度）  
static const int IMAGEHEIGHT = 1024;
static const double fx1 = 3438.33288873065;
static const double fy1 = 3438.12305473444;
static const double cx1 = 632.475504815606;
static const double cy1 = 573.812260891906;
static const double skew1 = -0.289598741803333;

static const double fx2 = 3423.59596542802;
static const double fy2 = 3423.59976962599;
static const double cx2 = 631.949263433127;
static const double cy2 = 494.004307589312;
static const double skew2 = -0.346570898118861;


//内参矩阵  MATLAB原生矩阵需要转置后使用
//fx  0    u0
//0   fy   v0
//0   0    1

static const cv::Mat CAMERAMATRIXLEFT = (cv::Mat_<double>(3, 3) << fx1, skew1, cx1, 0, fy1, cy1, 0, 0, 1);

//内参矩阵  MATLAB原生矩阵需要转置后使用
//fx  0    u0
//0   fy   v0
//0   0    1
static const cv::Mat CAMERAMATRIXRIGHT = (cv::Mat_<double>(3, 3) << fx2, skew2, cx2, 0, fy2, cy2, 0, 0, 1);

//外参矩阵

//T平移矩阵 1相机为57 2相机为82
//-89.4549989460323	0.653359338122292	2.37565471579666
const double TR[3] = {
		-87.1199750296595, 1.18155898266222, -0.292490200913241
};
static const cv::Mat TRANRIGHT_LEFT = (cv::Mat_<double>(3, 1)
		<< -87.1199750296595, 1.18155898266222, -0.292490200913241);
//R旋转矩阵 1相机为57 2相机为82 MATLAB原生矩阵需要转置后使用
static const cv::Mat ROTRIGHT_LEFT = (cv::Mat_<double>(3, 3)
		<< 0.999528914495547, -0.00112879326024514, 0.0306704240780729,
		0.000706552680322023, 0.999904879104987, 0.0137743793091460,
		-0.0306830551064124, -0.0137462201283817, 0.999434635962512);

//相机畸变参数 K1 K2 P1 P2 K3
static const cv::Mat DISTORTIONLEFT = (cv::Mat_<double>(5, 1)
		<< -0.0628578391748908, 0.414251200071182, 0.00109467966111133, 0.000162900018103355, 3.45769818149944);
//相机畸变参数 K1 K2 P1 P2 K3
static const cv::Mat DISTORTIONRIGHT = (cv::Mat_<double>(5, 1)
		<< -0.0662696317542926,	0.635132387298899	,-0.000574277894820003,	-0.000660601167243331,-0.559555029923310);

