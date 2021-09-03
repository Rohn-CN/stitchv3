#pragma once
#include <opencv.hpp>
using namespace std;
using namespace cv;
static Ptr<SIFT> sift_detector = SIFT::create(1000);
const  int MIN_MATCH_COUNT = 20;
struct MatchInfo
{
	Mat M;
	Mat inliers;
	vector<Point2f>src_pts, dst_pts;
};
struct ImageInfo
{
	int w, h, c;
	vector<KeyPoint> keypoints;
	Mat descriptor;
};
class CSiftMatcher
{
public:
	static ImageInfo getImgInfo(const Mat &image);
	static Mat getHomo(const Mat& img1, const Mat& img2);
	static MatchInfo getMatchInfo(const ImageInfo& imginfo1, const ImageInfo& imginfo2);
	static Mat myImread(const string filename,float ratio);
};

