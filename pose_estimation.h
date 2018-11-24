#ifndef SEGMENT_H_
#define SEGMENT_H_
#include <opencv2/opencv.hpp>
#include <iostream>
using namespace cv;
using namespace std;
Mat segment_img(Mat &src);
vector<Point2f> pick_point(Mat &srcImage, vector<vector<Point> > &contours, vector<int> squar_index);
Mat pose_estimation(Mat&, vector<Point2f>);
vector<Point2f>ranking(vector<Point2f>);
#endif // SEGMENT_H_