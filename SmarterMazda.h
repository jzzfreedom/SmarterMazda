#include <cv.h>
#include <highgui.h>

using namespace cv;


#ifndef SmarterMazda_H
#define SmarterMazda_H


	void init_front_camera();
	void init_back_camera();
	Mat Odd_Even_Split(Mat raw);
	Mat front_perspective(Mat raw);
	Mat back_perspective(Mat raw);

//void init_back_camera(Mat &camera_matrix, Mat &distort_matrix);
	//Mat get_front_camera_matrix();

#endif
