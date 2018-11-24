#include<opencv2\opencv.hpp>
#include<iostream>
#include"pose_estimation.h"
using namespace std;
using namespace cv;
int main()
{
	VideoCapture capture("1.mp4");
	while (1)
	{
		Mat frame;
		capture >> frame;
		Mat src;
		resize(frame, src, Size(frame.cols / 6, frame.rows / 6), (0, 0), (0, 0), 1);
		Mat seg_img;
		Mat imgThresholded = segment_img(src);
		vector<vector<Point>>contours;
		vector<Vec4i>hierarchy;
		findContours(imgThresholded, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_NONE);
		/*Mat contour = Mat::zeros(imgThresholded.rows, imgThresholded.cols, CV_8SC3);
		drawContours(contour, contours, -1, Scalar(200, 200, 200), 2);*/
		vector<int>squar_index;
		vector<Point2f>point_img;
		point_img = pick_point(imgThresholded, contours, squar_index);
		vector<Point2f> ranked_point_img=ranking(point_img);
		Mat output=pose_estimation(imgThresholded, ranked_point_img);
		imshow("pose_estimation", output);
		waitKey(30);
	}
	return 0;
}