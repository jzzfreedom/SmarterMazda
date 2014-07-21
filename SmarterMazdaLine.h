#include <cv.h>
#include <highgui.h>

using namespace cv;


#ifndef SmarterMazdaLine_H
#define SmarterMazdaLine_H



double getSlope(Vec4i l);
double getAngle(Vec4i l);
bool compare_two_lines(Vec4i L, Vec4i LL, double angle);
Vec4i combine_two_lines(Vec4i L1, Vec4i L2);
void combine_line(vector<Vec4i> lines, vector<Vec4i> *combined_lines);
int road_matching (Vector <Vec4i> combined_lines, int &left_bound, int &right_bound);
void eliminating_lines(vector<Vec4i> *combined_lines, Mat raw_gray);
Mat getLine(Mat raw);



#endif
