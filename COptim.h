#pragma once
#include <vector>
#include <opencv.hpp>
#include"CReprojectionError.h"
#include "CSiftMatcher.h"
const float RATIO = 0.25;
using namespace std;
class COptim
{
public:
	static void minRepOptimizer( vector<string>filelist, vector<ImageInfo> imageinfo, vector<Mat> &HomoList, vector<int> relativeFrame,vector<vector<Point2f> >&trajectory,vector<float> relativeOverlap);
	static void relativeMergeOptimizer(vector<string>filelist, vector<ImageInfo> imageinfo, vector<Mat>& HomoList, vector<int> relativeFrame, vector<vector<Point2f> >& trajectory, vector<float> relativeOverlap);
};

