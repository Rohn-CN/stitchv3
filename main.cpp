#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <io.h>
#include <opencv.hpp>
#include<math.h>
#include<time.h>
#include<numeric>
#include "CSiftMatcher.h"
#include "CReprojectionError.h"
#include "CMerge.h"
#include "CFrameSelction.h"
#include "COptim.h"
using namespace std;


//读文件
void getAllFiles(string path, vector<string>& files, string fileType)
{
	long long hFile = 0;
	struct _finddata_t fileinfo;
	string p;
	if ((hFile = _findfirst(p.assign(path).append("\\*" + fileType).c_str(), &fileinfo)) != -1) {
		do {
			files.push_back(p.assign(path).append("\\").append(fileinfo.name));
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
}
//用第一张照片初始化
void init(string firstImage, cv::Mat& mask, vector<ImageInfo>& imageinfo, vector<vector<Point2f> >& trajectory, vector<cv::Mat> homoList, cv::Mat& dst, cv::Mat& maskdst, cv::Size matSize)
{
	cv::Mat image1 = CSiftMatcher::myImread(firstImage, RATIO);
	ImageInfo img_based = CSiftMatcher::getImgInfo(image1);
	int h = img_based.h, w = img_based.w, c = img_based.c;
	mask = CMerge::creatMask(w, h);
	vector<Vec3f> corner_points{ Vec3f(0, 0, 1.0) ,Vec3f(0, h, 1.0) ,Vec3f(w, 0, 1.0) ,Vec3f(w, h, 1.0) };
	vector<Point2f> corner1;
	for (int i = 0; i < corner_points.size(); i++)
	{
		Mat H = homoList[0];
		Mat corner = H * corner_points[i];
		corner1.push_back(Point2f(corner.at<float>(0, 0) / corner.at<float>(2, 0),
			corner.at<float>(1, 0) / corner.at<float>(2, 0)));
	}
	trajectory.push_back(corner1);
	imageinfo.push_back(img_based);
	warpPerspective(image1, dst, homoList[0], matSize);
	warpPerspective(mask, maskdst, homoList[0], matSize);
}

int main()
{
	string path = "D:\\stitch_pic\\jjxz\\rgb";
	vector<string>filelist;
	getAllFiles(path, filelist, ".JPG");
	sort(filelist.begin(), filelist.end());
	cv::Mat image = CSiftMatcher::myImread(filelist[0], RATIO);
	ImageInfo img_based = CSiftMatcher::getImgInfo(image);

	int h = img_based.h, w = img_based.w, c = img_based.c;
	float homo_based[3][3] = { 1,0,2000,0,1,2000,0,0,1 };
	cv::Size matSize = Size(5 * max(w, h), 5 * max(w, h));

	//vector<cv::Mat> homoList{cv::Mat::eye(3,3,CV_32F)};
	vector<cv::Mat> homoList{ cv::Mat(3,3,CV_32F,homo_based) };
	vector<ImageInfo> imageinfo;
	vector< vector<Point2f> > trajectory;
	vector<string> fileSelected;
	Mat dst, maskdst, mask;
	init(filelist[0], mask, imageinfo, trajectory, homoList, dst, maskdst, matSize);

	fileSelected.push_back(filelist[0]);
	cout << "初始化完成";
	vector<float> relativeOverlap;
	for (int i = 1; i < filelist.size(); i++) {
		cv::Mat img_current = CSiftMatcher::myImread(filelist[i], RATIO);

		if (CFrameSelction::isKeyFrame(imageinfo.back(), img_current, imageinfo))
		{
			fileSelected.push_back(filelist[i]);
			cout << "载入" << fileSelected.back() << endl;
			cout << "载入成功" << endl;
			vector<Point2f> currentCorner = CFrameSelction::getCurrentCorner(imageinfo.back(), imageinfo.at(imageinfo.size() - 2), homoList);
			vector<int> relativeFrame = CFrameSelction::getRelativeFrames(currentCorner, trajectory, relativeOverlap);
			while (1) {
				try {
					COptim::minRepOptimizer(fileSelected, imageinfo, homoList, relativeFrame, trajectory, relativeOverlap);
					break;
				}
				catch (Exception e) {
					cout << "一个异常";
					relativeFrame.pop_back();
					relativeOverlap.pop_back();
					continue;
				}
			}
			CMerge::merge(homoList.back(), img_current, dst, mask, maskdst, w, h, matSize);
			cout << "over" << endl;
			fflush(stdout);
			if (fileSelected.size() % 4 == 0) {
				imwrite("D:\\stitch_pic\\test831Gao\\" + to_string(i) + ".jpg", dst);
			}
		}
		else
		{
			continue;
		}
	}




}