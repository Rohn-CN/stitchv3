#pragma once
#include <opencv.hpp>
using namespace cv;
class CMerge
{
public:
	static Mat creatMask(int w,int h);
	static void merge(Mat H, const Mat& imag2,  Mat& dst,const Mat &mask,Mat &maskdst,int w,int h,cv::Size matSize);
};

