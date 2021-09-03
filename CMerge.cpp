#include "CMerge.h"
#include <math.h>
Mat CMerge::creatMask(int w, int h)
{
	Mat mask = Mat::zeros(h, w, CV_32F);
	int x0 = w / 2;
	int y0 = h / 2;
	for (int i = 0; i < mask.rows; i++) {
		int dy = pow(i - y0, 2);
		for (int j = 0; j < mask.cols; j++)
		{
			int dx = pow(j - x0, 2);
			mask.at<float>(i, j) = 1 - sqrt(dx + dy) / sqrt(x0 * x0 + y0 * y0);
		}
	}
	return mask;
}
void CMerge::merge(Mat H, const Mat& img, Mat& dst, const Mat& mask, Mat& maskdst,int w,int h,cv::Size matSize)
{
	Mat dst1,maskdst1;
	warpPerspective(img, dst1, H, matSize);
	warpPerspective(mask, maskdst1, H, matSize);

	for (int row = 0; row < dst.rows; row++) {
		for (int col = 0; col < dst.cols; col++)
		{
			if (maskdst1.at<float>(row,col)>=maskdst.at<float>(row,col)) {
				dst.at<Vec3b>(row, col) = dst1.at<Vec3b>(row, col);
				maskdst.at<float>(row, col) = maskdst1.at<float>(row, col);
			}
		}
	}
}