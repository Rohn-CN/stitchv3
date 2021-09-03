#include "CReprojectionError.h"
#include <math.h>
vector<float> CReprojectionError::getRepErr(Mat M_dst, Mat M_src, vector<Point2f> dst_pts, vector<Point2f>src_pts, Mat mask)
{
	//遍历mask，取出dst_pts和src_pts中非0元素
	vector<Point2f>dst_rep, src_rep;
	for (int i = 0; i < mask.rows; i++) {

		if (mask.at<uchar>(i, 0) != 0)
		{
			Mat temp1 = M_dst * (Vec3f(dst_pts[i].x, dst_pts[i].y, 1.0));
			Mat temp2 = M_src * (Vec3f(src_pts[i].x, src_pts[i].y, 1.0));
			dst_rep.push_back(Point2f(temp1.at<float>(0, 0) / temp1.at<float>(2, 0), temp1.at<float>(1, 0) / temp1.at<float>(2, 0)));
			src_rep.push_back(Point2f(temp2.at<float>(0, 0) / temp2.at<float>(2, 0), temp2.at<float>(1, 0) / temp2.at<float>(2, 0)));
		}
	}
	//用指针很快，大概快一倍，但是会出问题
	//uchar* p;
	//for (int i = 0; i < mask.rows; i++)
	//{
	//	p = mask.ptr<uchar>(i);
	//	if ((int)p[i] != 0) {
	//		Mat temp1 = M_dst * (Vec3f(dst_pts[i].x, dst_pts[i].y, 1.0));
	//		Mat temp2 = M_src * (Vec3f(src_pts[i].x, src_pts[i].y, 1.0));
	//		dst_rep.push_back(Point2f(temp1.at<float>(0, 0) / temp1.at<float>(2, 0), temp1.at<float>(1, 0) / temp1.at<float>(2, 0)));
	//		src_rep.push_back(Point2f(temp2.at<float>(0, 0) / temp2.at<float>(2, 0), temp2.at<float>(1, 0) / temp2.at<float>(2, 0)));
	//	}
	//}
	vector<float> rep_err;
	for (int i = 0; i < dst_rep.size(); i++) {
		rep_err.push_back(sqrt(pow((dst_rep[i].x - src_rep[i].x), 2) + pow((dst_rep[i].y - src_rep[i].y), 2)));
	}
	return rep_err;
}
