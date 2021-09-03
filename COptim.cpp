#include "COptim.h"
#include <numeric>
#include "CSiftMatcher.h"

void COptim::minRepOptimizer(vector<string>filelist, vector<ImageInfo> imageinfo, vector<Mat>& HomoList, vector<int> relativeFrame, vector<vector<Point2f> >& trajectory, vector<float> relativeOverlap)
{
	vector<cv::Mat> imgsTmp;
	vector<MatchInfo> matchInfoTmp;
	vector<ImageInfo> imageInfoTmp;
	vector<float> errorCurrent;
	int h = imageinfo[0].h, w = imageinfo[0].w;
	for (int i = 0; i < relativeFrame.size(); i++)
	{
		Mat img0 = CSiftMatcher::myImread(filelist[relativeFrame[i]], RATIO);
		imgsTmp.push_back(img0);
		imageInfoTmp.push_back(imageinfo[relativeFrame[i]]);
		matchInfoTmp.push_back(CSiftMatcher::getMatchInfo(imageinfo[relativeFrame[i]], imageinfo.back()));
	}
	//开始计算每一张的重投影误差
	for (int i = 0; i < relativeFrame.size(); i++)
	{
		int numOfPt = 0;
		float sumOfErr = 0.0;
		Mat H_j0 = matchInfoTmp[i].M;
		Mat H_j = HomoList[relativeFrame[i]] * H_j0;
		for (int j = 0; j < relativeFrame.size(); j++)
		{
			if (j != i)
			{
				vector<float>Err = CReprojectionError::getRepErr(
					HomoList[relativeFrame[j]], H_j,
					matchInfoTmp[j].dst_pts, matchInfoTmp[j].src_pts, matchInfoTmp[j].inliers
				);
				numOfPt += Err.size()/relativeOverlap[i];
				sumOfErr += std::accumulate(Err.begin(), Err.end(), 0.0) / relativeOverlap[i];
			}
		}
		errorCurrent.push_back(sumOfErr / numOfPt*relativeOverlap[i]);
	}
	int idxOfMinErr = min_element(errorCurrent.begin(), errorCurrent.end()) - errorCurrent.begin();
	int idxToMatch = relativeFrame[idxOfMinErr];
	Mat H = HomoList[idxToMatch] * matchInfoTmp[idxOfMinErr].M;
	cout << "选择帧：" << idxToMatch << endl;
	HomoList.push_back(H);
	vector<Vec3f> corner_points{ Vec3f(0, 0, 1.0) ,Vec3f(0, h, 1.0) ,Vec3f(w, 0, 1.0) ,Vec3f(w, h, 1.0) };
	vector<Point2f> currentCorner;
	for (int i = 0; i < corner_points.size(); i++)
	{
		Mat cor_rep = H * corner_points[i];
		currentCorner.push_back(Point2f(cor_rep.at<float>(0, 0) / cor_rep.at<float>(2, 0),
			cor_rep.at<float>(1, 0) / cor_rep.at<float>(2, 0)));
	}
	trajectory.push_back(currentCorner);
};






vector<Point2f> getRepPoints(cv::Mat H_dst, ImageInfo img_dst)
{
	vector<cv::KeyPoint> keypoints = img_dst.keypoints;
	vector<Vec3f> pt_dst;
	vector <Point2f> pt_dst_rep;
	for (int i = 0; i < keypoints.size(); i++)
	{
		float x = keypoints[i].pt.x;
		float y = keypoints[i].pt.y;
		pt_dst.push_back(Vec3f(x,y,1.0));
	}
	for (int i = 0; i < pt_dst.size(); i++)
	{
		Mat rep = H_dst * pt_dst[i];
		pt_dst_rep.push_back(Point2f(rep.at<float>(0, 0) / rep.at<float>(2, 0), rep.at<float>(1, 0) / rep.at<float>(2, 0)));
	}
	return pt_dst_rep;
}

void COptim::relativeMergeOptimizer(vector<string>filelist, vector<ImageInfo> imageinfo, vector<Mat>& HomoList, vector<int> relativeFrame, vector<vector<Point2f> >& trajectory, vector<float> relativeOverlap)
{
	Mat H_dst = HomoList[relativeFrame[0]];
	ImageInfo imginfo_dst = imageinfo[relativeFrame[0]];
	vector<Point2f> pt_dst_all = getRepPoints(H_dst, imginfo_dst);
	Mat des_dst_all = imginfo_dst.descriptor;

	int h = imageinfo[0].h, w = imageinfo[0].w;
	if (relativeFrame.size() == 1){
		Mat H = CSiftMatcher::getMatchInfo(imginfo_dst, imageinfo.back()).M;
		H = HomoList.back() * H;
		HomoList.push_back(H);
		vector<Vec3f> corner_points{ Vec3f(0, 0, 1.0) ,Vec3f(0, h, 1.0) ,Vec3f(w, 0, 1.0) ,Vec3f(w, h, 1.0) };
		vector<Point2f> currentCorner;
		for (int i = 0; i < corner_points.size(); i++)
		{
			Mat cor_rep = H * corner_points[i];
			currentCorner.push_back(Point2f(cor_rep.at<float>(0, 0) / cor_rep.at<float>(2, 0),
				cor_rep.at<float>(1, 0) / cor_rep.at<float>(2, 0)));
		}
		trajectory.push_back(currentCorner);

	}
	else
	{
		for (int i = 0; i < relativeFrame.size() - 1; i++)
		{
			H_dst = HomoList[relativeFrame[i+1]];
			imginfo_dst = imageinfo[relativeFrame[i+1]];
			vector<Point2f> pt_dst = getRepPoints(H_dst, imginfo_dst);
			Mat des_dst = imginfo_dst.descriptor;

			//链接
			pt_dst_all.insert(pt_dst_all.end(), pt_dst.begin(), pt_dst.end());
			vconcat(des_dst_all, des_dst, des_dst_all);
		}
		Mat des_src_all = imageinfo.back().descriptor;
		vector<KeyPoint> kp_src_all = imageinfo.back().keypoints;


		Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
		vector<vector<DMatch> > matches;
		vector<DMatch> goods;

		matcher->knnMatch(des_src_all, des_dst_all, matches, 2);
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
				dst_pts.push_back(pt_dst_all[goods[i].trainIdx]);
				src_pts.push_back(kp_src_all[goods[i].queryIdx].pt);
			}
		}
		Mat mask;
		Mat H = findHomography(src_pts, dst_pts, mask, cv::RANSAC, 0.5);
		H.convertTo(H, CV_32F);
		HomoList.push_back(H);
		vector<Vec3f> corner_points{ Vec3f(0, 0, 1.0) ,Vec3f(0, h, 1.0) ,Vec3f(w, 0, 1.0) ,Vec3f(w, h, 1.0) };
		vector<Point2f> currentCorner;
		for (int i = 0; i < corner_points.size(); i++)
		{
			Mat cor_rep = H * corner_points[i];
			currentCorner.push_back(Point2f(cor_rep.at<float>(0, 0) / cor_rep.at<float>(2, 0),
				cor_rep.at<float>(1, 0) / cor_rep.at<float>(2, 0)));
		}
		trajectory.push_back(currentCorner);
	}
	
}
