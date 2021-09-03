#include "CFrameSelction.h"
//从大到小的argsort
vector<int> argsort(const vector<float>& array)
{
	vector<int> array_index;
	for (int i = 0; i < array.size(); i++) {
		array_index.push_back(i);
	}
	sort(array_index.begin(), array_index.end(),
		[&array](int pos1, int pos2){return (array[pos1] > array[pos2]); });
	return array_index;
}

//输入两个四边形轮廓，输出重叠区域比
float calcOverlap(vector<Point2f>pts1, vector<Point2f> pts2)
{
	RotatedRect rect1 = cv::minAreaRect(pts1);
	RotatedRect rect2 = cv::minAreaRect(pts2);
	float areaRect2 = rect2.size.width * rect2.size.height;
	vector<Point2f> vertices;

	int intersectionType = cv::rotatedRectangleIntersection(rect1, rect2, vertices);

	if (vertices.size() == 0)
		return 0.0;
	else {
		vector<Point2f> order_pts;
		cv::convexHull(Mat(vertices), order_pts, true);
		float area = cv::contourArea(order_pts);
		return area / areaRect2;
	}
}
vector<Point2f> CFrameSelction::getCurrentCorner(ImageInfo img_current, ImageInfo img_last, vector<cv::Mat> HomoList)
{
	int w = img_current.w, h = img_current.h;
	MatchInfo matchinfo = CSiftMatcher::getMatchInfo(img_last, img_current);
	vector<Vec3f> corner_points{ Vec3f(0, 0, 1.0) ,Vec3f(0, h, 1.0) ,Vec3f(w, 0, 1.0) ,Vec3f(w, h, 1.0) };
	Mat H = matchinfo.M;
	vector<Point2f> currentCorner;
	for (int i = 0; i < corner_points.size(); i++)
	{
		Mat cor_rep = HomoList.back() * H * corner_points[i];
		currentCorner.push_back(Point2f(cor_rep.at<float>(0, 0) / cor_rep.at<float>(2, 0),
			cor_rep.at<float>(1, 0) / cor_rep.at<float>(2, 0)));
	}
	return currentCorner;
}

vector<int> CFrameSelction::getRelativeFrames(vector<Point2f> currentFrame, vector< vector<Point2f> > trajectory, vector<float>& relativeOverlap)
{
	vector<float> overlap;
	for (int i = 0; i < trajectory.size(); i++)
	{
		float ol = calcOverlap(trajectory[i], currentFrame);
		overlap.push_back(ol);
	}
	vector<int> idx = argsort(overlap);
	vector<int> idx_cut;
	vector<float> idx_cut_overlap;
	for (int i = 0; i < idx.size(); i++)
	{

		if (overlap[idx[i]] >= 0.6) {
			idx_cut.push_back(idx[i]);
			idx_cut_overlap.push_back(overlap[idx[i]]);
			cout << idx[i] << "重叠度: " << overlap[idx[i]] << " ";
		}
	}
	relativeOverlap = idx_cut_overlap;
	return idx_cut;
}
bool CFrameSelction::isKeyFrame(ImageInfo img_info1, cv::Mat image2, vector<ImageInfo>& imageinfo)
{
	ImageInfo img_info2 = CSiftMatcher::getImgInfo(image2);
	MatchInfo matchinfo = CSiftMatcher::getMatchInfo(img_info1, img_info2);
	Mat H = matchinfo.M;
	int w = img_info2.w;
	int h = img_info2.h;
	vector<Vec3f> corner_points{ Vec3f(0, 0, 1.0) ,Vec3f(0, h, 1.0) ,Vec3f(w, 0, 1.0) ,Vec3f(w, h, 1.0) };
	vector<Point2f>corner_rep;
	vector<Point2f>corner1;
	for (int i = 0; i < corner_points.size(); i++)
	{
		Mat cor_rep = H * corner_points[i];
		corner_rep.push_back(Point2f(cor_rep.at<float>(0, 0) / cor_rep.at<float>(2, 0),
			cor_rep.at<float>(1, 0) / cor_rep.at<float>(2, 0)));
		corner1.push_back(Point2f(corner_points[i][0], corner_points[i][1]));
	}
	float overlap = calcOverlap(corner1, corner_rep);

	cout << "当前帧重叠度：" << overlap << endl;
	if (overlap > 0.7 && overlap < 0.9)
	{
		imageinfo.push_back(img_info2);
		return true;
	}
	else
	{
		return false;
	}

}
