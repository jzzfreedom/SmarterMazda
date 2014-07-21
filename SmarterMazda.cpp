#include <cv.h>
#include <highgui.h>
#include "SmarterMazda.h"

using namespace cv;
using namespace std;

Mat Front_Camera_Matrix;
Mat Front_Distort_Matrix;
Mat Back_Camera_Matrix;
Mat Back_Distort_Matrix;
/*
void init_front_camera()
{
	Front_Camera_Matrix = Mat::zeros(3,3,CV_32FC1);
	Front_Camera_Matrix.at<float>(0,0) = 293.716547320860570;
	Front_Camera_Matrix.at<float>(1,1) = 259.741695162271410;
	Front_Camera_Matrix.at<float>(2,2) = 1.000;
	Front_Camera_Matrix.at<float>(0,2) = 348.514914313399630;
	Front_Camera_Matrix.at<float>(1,2) = 235.385404582136230;

	Front_Distort_Matrix = Mat::zeros(1,5,CV_32FC1);
	Front_Distort_Matrix.at<float>(0,0) = 0.069866698804567;
	Front_Distort_Matrix.at<float>(0,1) = -0.058162272964267;
	Front_Distort_Matrix.at<float>(0,2) = 0.000370643369402;
	Front_Distort_Matrix.at<float>(0,3) = -0.001231482336972;
}
*/

void init_front_camera()
{
	Front_Camera_Matrix = Mat::zeros(3,3,CV_32FC1);
	Front_Camera_Matrix.at<float>(0,0) = 291.493724634612700;
	Front_Camera_Matrix.at<float>(1,1) = 257.966836314136460;
	Front_Camera_Matrix.at<float>(2,2) = 1.000;
	Front_Camera_Matrix.at<float>(0,2) = 346.651491402996670;
	Front_Camera_Matrix.at<float>(1,2) = 233.627294770128540;

	Front_Distort_Matrix = Mat::zeros(1,5,CV_32FC1);
	Front_Distort_Matrix.at<float>(0,0) = 0.114308075295631;
	Front_Distort_Matrix.at<float>(0,1) = -0.084770167768904;
	Front_Distort_Matrix.at<float>(0,2) = -0.000657379069175;
	Front_Distort_Matrix.at<float>(0,3) = 0.002791584989720;
}



void init_back_camera()
{
	Back_Camera_Matrix = Mat::zeros(3,3,CV_32FC1);
	Back_Camera_Matrix.at<float>(0,0) = 285.797456478870570;
	Back_Camera_Matrix.at<float>(1,1) = 248.902829528141130;
	Back_Camera_Matrix.at<float>(2,2) = 1.000;
	Back_Camera_Matrix.at<float>(0,2) = 366.418587723627810;
	Back_Camera_Matrix.at<float>(1,2) = 257.054193501866280;

	Back_Distort_Matrix = Mat::zeros(1,5,CV_32FC1);
	Back_Distort_Matrix.at<float>(0,0) = 0.059051663762715;
	Back_Distort_Matrix.at<float>(0,1) = -0.045899292472864;
	Back_Distort_Matrix.at<float>(0,2) = 0.001326408278578;
	Back_Distort_Matrix.at<float>(0,3) = -0.004629694306098;
}

Mat Odd_Even_Split(Mat raw)
{
	Mat split_frame;
	int width = raw.cols;
	int height = raw.rows;
	split_frame = Mat::zeros(height/2, width, CV_8UC3);
	int sum = 0;
	for (int i=0; i<height/2; i++)
	{
		for (int j=0; j<width; j++)
		{
			split_frame.at<Vec3b>(i,j) = raw.at<Vec3b>(2*i, j);
		}
		sum++;
	}
	resize(split_frame, split_frame, Size(width, height));
	return split_frame;
}

Mat front_perspective(Mat raw)
{
	Point2f before[4];
	Point2f after[4];

	before[0].x = 280;
	before[0].y = 240;
	before[1].x = 1;
	before[1].y = 360;
	before[2].x = 420;
	before[2].y = 240;
	before[3].x = 719;
	before[3].y = 360;

	after[0].x = 80;
	after[0].y = 60;
	after[1].x = 80;
	after[1].y = 420;
	after[2].x = 160;
	after[2].y = 60;
	after[3].x = 160;
	after[3].y = 420;

	Mat trans = getPerspectiveTransform(before, after);
	Mat final;
	warpPerspective(raw,final, trans, Size(240,480), cv::INTER_LINEAR);
	return final;
}

Mat back_perspective(Mat raw)
{
	Point2f before[4];
	Point2f after[4];

	before[0].x = 240;
	before[0].y = 180;
	before[1].x = 1;
	before[1].y = 360;
	before[2].x = 480;
	before[2].y = 180;
	before[3].x = 719;
	before[3].y = 360;

	after[0].x = 60;
	after[0].y = 60;
	after[1].x = 60;
	after[1].y = 420;
	after[2].x = 180;
	after[2].y = 60;
	after[3].x = 180;
	after[3].y = 420;

	Mat trans = getPerspectiveTransform(before, after);
	Mat final;
	warpPerspective(raw,final, trans, Size(240,480), cv::INTER_LINEAR);
	return final;
}

