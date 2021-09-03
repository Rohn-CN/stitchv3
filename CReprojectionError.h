#pragma once
#include <vector>
#include <opencv.hpp>
using namespace std;
using namespace cv;
class CReprojectionError
{
public:
	static vector<float> getRepErr(Mat M_dst, Mat M_src, vector<Point2f> dst_pts, vector<Point2f>src_pts, Mat mask);
};

