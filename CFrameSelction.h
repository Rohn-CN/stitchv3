#pragma once
#include <vector>
#include <opencv.hpp>
#include "CSiftMatcher.h"
using namespace std;
using namespace cv;
class CFrameSelction
{
public:
	static vector<int> getRelativeFrames(vector<Point2f> currentFrame,vector< vector<Point2f> > trajectory, vector<float>& relativeOverlap);
	static bool isKeyFrame(ImageInfo img_info1, cv::Mat image2, vector<ImageInfo>& imageinfo);
	static vector<Point2f> getCurrentCorner(ImageInfo img_current, ImageInfo img_last, vector<cv::Mat> HomoList);
};
 