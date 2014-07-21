#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/ml/ml.hpp>
#include <assert.h>
#include <math.h>
#include "SmarterMazdaLine.h"

using namespace std;
using namespace cv;

#define DEBUG true
#define PI 3.1415926
#define Angle_Threshold 80
#define Delta_Angle_Threshold 6
#define Delta_Middle_Threshold 25
#define Road_Width 120
#define Road_Width_Error 20
#define Road_Brightness_Threshold 100



double getSlope(Vec4i l)
{
	double temp = l[2]-l[0];
	if (temp < 0.01) temp++;
	double slope = abs((l[3]-l[1])/temp);
	return slope;
}

double getAngle(Vec4i l)
{
	double temp = l[2]-l[0];
	if (temp < 0.01) temp++;
	double slope = abs((l[3]-l[1])/temp);
	double angle = atan(slope)/PI*180;
	return angle;
}


bool compare_two_lines(Vec4i L, Vec4i LL, double angle)
{
	//bool similar = false;
	double angle2 = getAngle(LL);
	//assert(slope1 > 0);
	//assert(slope2 > 0);
	double delta_angle = abs(angle-angle2);

	if (delta_angle > Delta_Angle_Threshold) return false;

	double middle1 = L[0]+L[2];
	double middle2 = LL[0]+LL[2];
	double delta_middle = abs(middle1-middle2);
	//cout<<"Comparing: "<<delta_angle<<"   "<<delta_middle<<endl;
	if (delta_middle > Delta_Middle_Threshold) return false;

	return true;
}

Vec4i combine_two_lines(Vec4i L1, Vec4i L2)
{
	Vec4i combined_line;
	if (L1[3]>L1[1])
	{
		swap(L1[3], L1[1]);
		swap(L1[2], L1[0]);
	}

	if (L2[3]>L2[1])
	{
		swap(L2[3], L2[1]);
		swap(L2[2], L2[0]);
	}
	combined_line[0] = (L1[0]+L2[0])/2;
	combined_line[2] = (L1[2]+L2[2])/2;
	combined_line[1] = (L1[1]>L2[1])?L1[1]:L2[1];
	combined_line[3] = (L1[3]<L2[3])?L1[3]:L2[3];

	return combined_line;
}



void combine_line(vector<Vec4i> lines, vector<Vec4i> *combined_lines)
{
	vector<Vec4i> temp_lines;

	for (int i = 0; i < lines.size(); ++i)
	{
		Vec4i L = lines[i];
		double angle = getAngle(L);
		if (angle<Angle_Threshold) continue;
		
		bool combined = false;
		//cout<<"#############################Angle is: "<<angle<<endl;
		
		for (int j = 0; j < temp_lines.size(); ++j)
		{
			//if similar, combine
			Vec4i new_L = temp_lines[j];

			bool similar = compare_two_lines(L, new_L, angle);
			if (similar)
			{
				
				//combine
				temp_lines[j] = combine_two_lines(L, new_L);//update
				//cout<<"Find out a combination!"<<endl;
				combined = true;
				break;
			}
		}
		
		if (!combined)//a new possible line detected
		{
			temp_lines.push_back(L);
			//cout<<"Number of lines: "<<temp_lines.size()<<endl;
		}
		
	}
	for (int i = 0; i<temp_lines.size(); ++i)
	{
		combined_lines->push_back(temp_lines[i]);
	}
}
int road_matching (Vector <Vec4i> combined_lines, int &left_bound, int &right_bound)
{
	int flag = 0;
	if (combined_lines.size()<2) return 0;
	for (int i = 0; i < combined_lines.size()-1; ++i)
	{
		for (int j = i+1; j < combined_lines.size(); ++j)
		{
			int middle1 = (combined_lines[i][0]+combined_lines[i][2])/2;
			int middle2 = (combined_lines[j][0]+combined_lines[j][2])/2;
			int delta_middle = abs(middle1-middle2);

			if ((delta_middle <= (Road_Width+Road_Width_Error))&&(delta_middle >= (Road_Width-Road_Width_Error)))
			{
				left_bound = middle1;
				right_bound = middle2;
				cout<<"Road Width is: "<<delta_middle<<endl;
				return 1;
			}

		}
	}

	return flag;
}

void eliminating_lines(vector<Vec4i> *combined_lines, Mat raw_gray)
{
	//cout<<"This pic depth is: "<<raw_gray.channels()<<endl;
	for (vector<Vec4i>::iterator iter = combined_lines->begin();iter!=combined_lines->end();)
	{
		Vec4i L = *iter;
		//cout<<L[0]<<" "<<L[1]<<endl;
		int H_min = (L[1]<L[3])?L[1]:L[3];
		int H_max = (L[1]>L[3])?L[1]:L[3];
		int sum = 0;
		int pixel_sum = 0;
		for (int y = H_min; y < H_max; ++y)
		{
			int x = (L[2]-L[0])*(y-L[1])/(L[3]-L[1])+L[0];
			sum++;
			pixel_sum += raw_gray.at<unsigned char>(y,x);
			//raw_gray.at<unsigned char>(y,x) = 0;
		}
		if ((pixel_sum/sum) < Road_Brightness_Threshold)
		{
			iter = combined_lines->erase(iter);
		}
		else
		{
			iter++;
		}
		//cout<<"Average of this line is: "<<pixel_sum/sum<<" num is: "<<sum<<endl;
	}

}


/*This function is designed to detect lane*/
Mat getLine(Mat raw)
{
	//Mat Line;
	Mat raw_gray;//grayscale image of Mat raw
	Mat pure_line;//result image

		cvtColor(raw, raw_gray, CV_BGR2GRAY);//convert to grayscale
		GaussianBlur(raw_gray, raw_gray, Size(11,11), 0, 0);//Gaussian Blur function
		Canny(raw_gray, raw_gray, 0, 30, 3);//Canny line detection
		vector<Vec4i> lines;//vector to contain lines detected by hough transformation
		vector<Vec4i> combined_lines;
		HoughLinesP(raw_gray, lines, 3, CV_PI/180, 180, 180, 30);


		/*Draw original result after hough transformation*/
		if (DEBUG)
		{
			pure_line = Mat::zeros(raw.rows, raw.cols, CV_8UC3);
			cvtColor(raw_gray, pure_line, CV_GRAY2BGR);
			for( size_t i = 0; i < lines.size(); i++ )
			{
				Vec4i l = lines[i];
				//line( pure_line, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 1, CV_AA);
			}
		}
		
		
		
		//clustering lines detected by Hough Transformation
		combine_line(lines, &combined_lines);

		//drawing lines after clustering
		if (DEBUG)
		{
			for( size_t i = 0; i < combined_lines.size(); i++ )
			{
				Vec4i l = combined_lines[i];
				line( pure_line, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,255,255), 2, CV_AA);
			}
		}



		//eliminate lines caused by shadow and other situation. I hate roads in MICHIGAN!!

		eliminating_lines(&combined_lines, raw_gray);

		if (DEBUG)
		{
			for( size_t i = 0; i < combined_lines.size(); i++ )
			{
				Vec4i l = combined_lines[i];
				line( pure_line, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,255,0), 2, CV_AA);
			}
		}

		 //model matching to find lane
		 int left_bound;
		 int right_bound;
		 int matching_flag = 0;
		 
		 matching_flag = road_matching(combined_lines, left_bound, right_bound);
		 if (matching_flag == 1)//find road
		 {
			 line( pure_line, Point(left_bound, 240), Point(right_bound, 240), Scalar(255,0,0), 8, CV_AA);
		 }
		 

		 //namedWindow("GRAY", 1);
		 //imshow("GRAY", raw_gray);
		return pure_line;
}