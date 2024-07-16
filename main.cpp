#include <iostream>
#include <opencv2/opencv.hpp>
#include<opencv2/imgproc.hpp>
#include "CameraParams_1_1.h"

double baseline = std::sqrt(-89.4549989460323 * -89.4549989460323 + 0.653359338122292 * 0.653359338122292 +
                            2.37565471579666 * 2.37565471579666);
using namespace cv;
using namespace std;
// ����������������������������������������������������

//����ͷ�ķֱ���/ͼƬ�ߴ�
cv::Size imageSize = cv::Size(IMAGEWIDTH, IMAGEHEIGHT);
cv::Mat cameraMatrix1 = CAMERAMATRIXLEFT;
cv::Mat cameraMatrix2 = CAMERAMATRIXRIGHT;

//��ӦMatlab����1����������K1K2P1P2K3
cv::Mat distCoeffs1 = DISTORTIONLEFT;
cv::Mat distCoeffs2 = DISTORTIONRIGHT;

//R��ת����
cv::Mat R = ROTRIGHT_LEFT;
//Tƽ�ƾ���
cv::Mat T = TRANRIGHT_LEFT;
//ӳ���
cv::Mat map1x, map1y, map2x, map2y;
//У����ת����R��ͶӰ����P ��ͶӰ����Q
cv::Mat R1, R2, P1, P2, Q;
//ͼ��У��֮�󣬻��ͼ����вü��������validROI����ָ�ü�֮�������
cv::Rect validROI1, validROI2;
// ����������������������������������������������������
typedef struct {
	cv::Point2i cpt;
	int agl= 0;
} CptAgl;
CptAgl CptAgl1;
CptAgl CptAgl2;

//��ʾͼƬ
void DisplayImage(String winname, cv::Mat img) {
	cv::namedWindow(winname, WINDOW_NORMAL);
	cv::imshow(winname, img);
	cv::waitKey();
}

// ͼ��ƴ��
Mat ConcatImage(cv::Mat &img1, cv::Mat &img2) {
	cv::Mat retMat;
	// ͼ��ƴ��
	hconcat(img1, img2, retMat);
	// ���ƺ���
	int nLineCount = 10;
	double nStepHeight = retMat.size().height / (double) nLineCount;
	for (int i = 1; i < nLineCount; i++) {
		line(retMat, Point(-1, i * nStepHeight),
		     Point(retMat.size().width, i * nStepHeight), Scalar(0, 255, 0), 4, LINE_AA);
	}
	return Mat(retMat);
}

void calculate(double x1, double y1, double x2, double y2) {
	double d = x2 - x1;
	double z = -baseline * 2117.53611159527 / (d);
	double x = z * (x1 - 573.683521813225) / 2117.53611159527;
	double y = -z * (y1 - 564.320552825790) / 2117.73147357458;
	printf("x:%f,y:%f,z:%f\n", x, y, z);
}
int main() {
	// ��ȡͼƬ
	cv::Mat imgL = cv::imread("C:\\Users\\29451\\Desktop\\calibrationresulttest\\cam1_4774.png");
	cv::Mat imgR = cv::imread("C:\\Users\\29451\\Desktop\\calibrationresulttest\\cam0_4775.png");
	//ͼ��ƴ�Ӳ���ʾ
	//DisplayImage("���������ԭʼͼ��", ConcatImage(imgL, imgR));
	// ���������У�������ת����
	cv::stereoRectify(cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, imageSize, R, T, R1, R2, P1, P2, Q,
	                  cv::CALIB_ZERO_DISPARITY,//������CALIB_ZERO_DISPARITY ������0
	                  -1, imageSize, &validROI1, &validROI2);
	cv::initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, imageSize, CV_32FC1, map1x, map1y);
	cv::initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, imageSize, CV_32FC1, map2x, map2y);

	// ��ͼ��У����ʼ��ʱ
	double timeZero = double(getTickCount());

	// remap�������ͼ���ж���
	cv::remap(imgL, imgL, map1x, map1y, cv::INTER_LINEAR);
	cv::remap(imgR, imgR, map2x, map2y, INTER_LINEAR);

	// ͼ��ƴ�Ӳ���ʾ
	//DisplayImage("���������У����ͼ��", ConcatImage(imgL, imgR));

	// ����������������������������������������������������
	// ͼ��������ͼΪ��
	Mat img = imgL.clone();
	// תΪ�Ҷ�ͼ
	cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
	//DisplayImage("��˹�˲�ǰ", img);
	// ��˹�˲�
	cv::GaussianBlur(img, img, cv::Size(5, 5), 3, 3);
	//DisplayImage("��˹�˲���", img);

	// ��ֵ��
	cv::Mat binary;
	cv::threshold(img, binary, 60, 255, cv::THRESH_BINARY);
	//DisplayImage("��ֵ��", binary);
	// ��������
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	// ���������ֵ
	double minArea = 20000.0; // ��С���
	double maxArea = 50000.0; // ������

	// �ҵ���ת�ľ��β������������
	for (size_t i = 0; i < contours.size(); i++) {
		double area = cv::contourArea(contours[i]);
		if (area > minArea && area < maxArea) {
			cv::RotatedRect minRect = cv::minAreaRect(contours[i]);
			CptAgl1.cpt = minRect.center;
			CptAgl1.agl = minRect.angle;
			if (minRect.size.height < 0.5 * img.rows && minRect.size.width < 0.5 * img.cols) {
				// ��ȡ���ε��ĸ�����
				cv::Point2f rect_points[4];
				minRect.points(rect_points);

				// ��ԭͼ�ϻ��ƾ���
				for (int j = 0; j < 4; j++) {
					cv::line(imgL, rect_points[j], rect_points[(j + 1) % 4], cv::Scalar(0, 255, 0), 2);
					// ��ʽ���ǵ�����
					std::string label = "(" + std::to_string(static_cast<int>(rect_points[j].x)) + ", " +
					                    std::to_string(static_cast<int>(rect_points[j].y)) + ")";

					// ���ƽǵ�����
					cv::putText(imgL, label, rect_points[j], cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 1);
				}
				//DisplayImage("��������", img);
				cv::imwrite("left.jpg", imgL);
			}
		}
	}


	// ����������������������������������������������������
	// ͼ��������ͼΪ��
	Mat imgRc = imgR.clone();
	// תΪ�Ҷ�ͼ
	cv::cvtColor(imgRc, imgRc, cv::COLOR_BGR2GRAY);
	//DisplayImage("��˹�˲�ǰ", img);
	// ��˹�˲�
	cv::GaussianBlur(imgRc, imgRc, cv::Size(5, 5), 3, 3);
	//DisplayImage("��˹�˲���", img);

	// ��ֵ��
	cv::Mat binaryR;
	cv::threshold(imgRc, binaryR, 60, 255, cv::THRESH_BINARY);
	//DisplayImage("��ֵ��", binaryR);
	// ��������
	std::vector<std::vector<cv::Point>> contoursR;
	cv::findContours(binaryR, contoursR, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	// �ҵ���ת�ľ��β������������
	for (size_t i = 0; i < contoursR.size(); i++) {
		double area = cv::contourArea(contoursR[i]);
		if (area > minArea && area < maxArea) {
			cv::RotatedRect minRectR = cv::minAreaRect(contoursR[i]);
			CptAgl2.cpt = minRectR.center;
			CptAgl2.agl = minRectR.angle;
			if (minRectR.size.height < 0.5 * img.rows && minRectR.size.width < 0.5 * img.cols) {
				// ��ȡ���ε��ĸ�����
				cv::Point2f rect_points[4];
				minRectR.points(rect_points);

				// ��ԭͼ�ϻ��ƾ���
				for (int j = 0; j < 4; j++) {
					cv::line(imgR, rect_points[j], rect_points[(j + 1) % 4], cv::Scalar(0, 255, 0), 1);
					// ��ʽ���ǵ�����
					std::string label = "(" + std::to_string(static_cast<int>(rect_points[j].x)) + ", " +
					                    std::to_string(static_cast<int>(rect_points[j].y)) + ")";

					// ���ƽǵ�����
					cv::putText(imgR, label, rect_points[j], cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 1);
				}
				//DisplayImage("��������", imgR);
				cv::imwrite("right.jpg", imgR);
			}
		}
	}
	calculate(670, 402, 366, 403);
	calculate(882, 400, 579, 401);
	calculate(673, 614, 371, 614);
	calculate(885, 612, 581, 611);
	cout << "Time: " << ((double) getTickCount() - timeZero) / getTickFrequency() * 1000 << "ms" << endl;
	//
	//	x:-91.059108,y:48.526445,z:-625.399430
	//	x:-29.429552,y:-14.719702,z:-627.470289
	//	x:-91.642686,y:-14.034174,z:-623.342195
	waitKey(0);
	return 0;
}


