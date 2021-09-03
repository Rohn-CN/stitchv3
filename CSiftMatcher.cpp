#include "CSiftMatcher.h"

using namespace std;
using namespace cv;


Mat CSiftMatcher::myImread(const string filename,float ratio)
{
	Mat src = imread(filename);
	Mat image;
	resize(src, image, Size(src.cols * ratio, src.rows * ratio));
	return image;
}

ImageInfo CSiftMatcher::getImgInfo(const Mat&image)
{
	ImageInfo imageinfo;
	vector <KeyPoint>keypoints;
	Mat descriptors;


	sift_detector->detectAndCompute(image, noArray(), keypoints, descriptors);

	imageinfo.w = image.cols; imageinfo.h = image.rows; imageinfo.c = image.channels();
	imageinfo.descriptor = descriptors;
	imageinfo.keypoints = keypoints;

	return imageinfo;

}

MatchInfo CSiftMatcher::getMatchInfo(const ImageInfo& imginfo1, const ImageInfo& imginfo2)
{
	MatchInfo matchinfo;
	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
	vector<vector<DMatch> > matches;
	vector<DMatch> goods;
	matcher->knnMatch(imginfo1.descriptor, imginfo2.descriptor, matches, 2);
	for (int i = 0; i < matches.size(); i++) {
		if (matches[i][0].distance < matches[i][1].distance * 0.7) {
			goods.push_back(matches[i][0]);
		}
	}
	vector<Point2f> dst_pts, src_pts;
	if (goods.size() > MIN_MATCH_COUNT)
	{
		for (int i = 0; i < goods.size(); i++)
		{
			dst_pts.push_back(imginfo1.keypoints[goods[i].queryIdx].pt);
			src_pts.push_back(imginfo2.keypoints[goods[i].trainIdx].pt);
		}
	}

	Mat mask;
	Mat homo = findHomography(src_pts, dst_pts, mask, cv::RANSAC, 0.5);
	homo.convertTo(homo, CV_32F);
	matchinfo.dst_pts = dst_pts;
	matchinfo.src_pts = src_pts;
	matchinfo.M = homo;
	matchinfo.inliers = mask;
	return matchinfo;
}
