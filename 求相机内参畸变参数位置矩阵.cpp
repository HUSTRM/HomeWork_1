/*--------------------------------------------------------------------------------------------------------------------------------------
Copyright (C),2018---,   HUST Liu
 File name:求相机内参畸变参数位置矩阵.cpp
 Author: 刘峻源       Version: 4      Date:2018.11.7
 Description: 
 利用openCV内置函数，搞：
 1、相机标定、求相机内参、畸变参数
 2、获取旋转矩阵R、位移矩阵T
-----------------------------------------------------------------------------------------------------------------------------------------
 Others:NONE
 May Function List: cornerSubPix 、calibrateCamera、goodFeaturesToTrack、solvePnP
-----------------------------------------------------------------------------------------------------------------------------------------
 History  as folwing :
 NONE
  --------------------------------------------------------------------------------------------------------------------------------------*/
  /*--------------------------------------------------------------------------------------------------------------------------------------
  -  openCV 标准开头
  ---------------------------------------------------------------------------------------------------------------------------------------*/

#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace std;
using namespace cv;
/*--------------------------------------------------------------------------------------------------------------------------------------
-  读写文件函数用的库
---------------------------------------------------------------------------------------------------------------------------------------*/
#include <iostream>
#include <fstream>
int main()
{
	//定义乱七八糟的变量
	float object_width= 12.3;
	float object_hight = 8.8;
	Size im_size;
	Size point_count = Size(9, 6);
	Size sqr_size = Size(35, 35);
	Point2f output_point2;
	vector<Point2f> Tp_image_points;  
	vector<vector<Point3f>> all_im_sqr_points;
	vector<vector<Point2f>> all_im_points;
	vector<Point3f> temp_PointSet;
	vector<Point3f> real_position_point;
	vector<Point2f> output_point;
	Mat input_image;
	Mat input_image2 = imread("D:/2234.jpg");
	Mat im_gray;
	Mat im_gray2;
	Mat camera_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0));
	Mat dist_coeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));
	Mat world_real_point = Mat(4, 3, CV_32FC1, Scalar::all(0));
	Mat image_point = Mat(4, 2, CV_32FC1, Scalar::all(0));
	Mat rod_r;
	Mat transmatrix;
	vector<Mat>  rvecsMat;
	vector<Mat> tvecsMat;
	vector<Mat>  rvecsMat1;
	vector<Mat> tvecsMat1;
	input_image=imread("D:/11.8相机标定/WIN_20181108_12_35_12_Pro.jpg");
	if (!input_image.data)
	{
		cout << "Reading Error" << endl;
	}
	else
	{
		cout << "Reading Successfuly" << endl;
	}
	im_size.height = input_image.rows;
	im_size.width = input_image.cols;
	cout << "image height：" << im_size.height << endl;
	cout << "image width：" << im_size.width << endl;
	cvtColor(input_image, im_gray, CV_RGB2GRAY);
	cvtColor(input_image2, im_gray2, CV_RGB2GRAY);
	if (findChessboardCorners(im_gray, point_count, Tp_image_points) == 0)
	{
		cout << "can not find coners" << endl;
	}
	cornerSubPix(im_gray,Tp_image_points, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
	drawChessboardCorners(im_gray, point_count, Tp_image_points, false);
	imshow("im_gray",im_gray);
	cout << Tp_image_points << endl;
	for (int i = 0; i < point_count.height; i++)
	{
		for (int j = 0; j < point_count.width; j++)
		{
			Point3f realPoint;
			realPoint.x = i * sqr_size.width;
			realPoint.y = j * sqr_size.height;
			realPoint.z = 0;
			temp_PointSet.push_back(realPoint);
		}
	}
	all_im_sqr_points.push_back(temp_PointSet);
	all_im_points.push_back(Tp_image_points);
	cout << "point of real world:" << temp_PointSet << endl;
	calibrateCamera(all_im_sqr_points,all_im_points,im_size,camera_matrix,dist_coeffs, rvecsMat, tvecsMat, 0);
	cout << "camera_matrix :" << endl;
	cout << camera_matrix << endl;
	cout << rvecsMat[0] << endl;
	cout << tvecsMat[0] << endl;
	cout << "dist_coeffs" << dist_coeffs << endl;
	goodFeaturesToTrack(im_gray2,output_point,4,0.01,500,Mat(),3,false,0.05);
	imshow("find", im_gray2);
	for (int flag = 0; flag < 4;flag++)
	{
		cout << "point" << output_point[flag];
		cout << endl;
	}

	for (int flag1 = 0; flag1< 4; flag1++)
	{
		for (int flag2 = flag1 + 1; flag2 < 4; flag2++)
		{
			if (output_point[flag2].y< output_point[flag1].y)
			{
				output_point2 = output_point[flag1];
				output_point[flag1] = output_point[flag2];
				output_point[flag2] = output_point2;
			}
		}
	}
	if (output_point[0].x> output_point[1].x)
	{
		output_point2 = output_point[0];
		output_point[0] = output_point[1];
		output_point[1] = output_point2;
	}
	if (output_point[2].x > output_point[3].x)
	{
		output_point2 = output_point[2];
		output_point[2] = output_point[3];
		output_point[3] = output_point2;
	}
	//监视总体排序效果
	cout << "output_point" << endl;
	for (int cout_flag = 0; cout_flag < 4; cout_flag++)
	{
		cout << output_point[cout_flag] << endl;
	}

	cout << "real_position" << endl;
	for (int flag2= 0; flag2< 2; flag2++)
	{
		for (int flag1 = 0; flag1 < 2; flag1++)
		{
			Point3f tp_point;
			tp_point.y = flag2 * object_hight;
			tp_point.x = flag1 * object_width;
			tp_point.z = 0.;
			real_position_point.push_back(tp_point);
		}
	}
	/*cout << world_real_point << endl;
	for (int cout_flag = 0; cout_flag < 4; cout_flag++)
	{
		cout << real_position_point[cout_flag] << endl;
	}
	for(int flag=0;flag<4;flag++)
	{
		uchar* point_to_wrp = world_real_point.ptr<uchar>(flag);
		point_to_wrp[0] = real_position_point[flag].x;
		cout << "检测结果" << point_to_wrp[0] << endl;
	}
	for (int flag = 0; flag < 4; flag++)
	{
		uchar* point_to_wrp = world_real_point.ptr<uchar>(flag);
		point_to_wrp[1] = real_position_point[flag].y;
	}	
	for (int flag = 0; flag < 4; flag++)
	{
		uchar* point_to_wrp = world_real_point.ptr<uchar>(flag);
		point_to_wrp[2] = real_position_point[flag].z;
	}
*/
	for (int flag = 0; flag < 4; flag++)
	{
		world_real_point.at<float>(flag, 0) = real_position_point[flag].x;
	}
	for (int flag = 0; flag < 4; flag++)
	{
		world_real_point.at<float>(flag, 1) = real_position_point[flag].y;
	}
	for (int flag = 0; flag < 4; flag++)
	{
		world_real_point.at<float>(flag, 2) = real_position_point[flag].z;
	}
	cout << world_real_point << endl;
	for (int flag = 0; flag < 4; flag++)
	{
		image_point.at<float>(flag, 0) = output_point[flag].x;
	}
	for (int flag = 0; flag < 4; flag++)
	{
		image_point.at<float>(flag, 1) = output_point[flag].y;
	}
	solvePnP(world_real_point, image_point, camera_matrix, dist_coeffs,rod_r, transmatrix);
	cout << "position matrix:" << endl;
	cout << rod_r << endl;
	cout << transmatrix << endl;
	waitKey(0);
}