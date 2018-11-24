#include"pose_estimation.h"
#include"math.h"
#define pi 3.1415926535;
//�ָ�ͼ��
Mat segment_img(Mat& src)
{
	Mat gray, dst, quzao;
	cvtColor(src, gray, COLOR_BGR2GRAY);
	blur(gray, quzao, Size(3, 3));
	Canny(quzao, dst, 50, 200, 3);
	return dst;
}
vector<Point2f> pick_point(Mat &srcImage, vector<vector<Point> > &contours, vector<int> squar_index)
{
	vector<Point2f>point_img;
	vector<vector<Point2f>>temp_img;
	for (size_t c = 0; c < squar_index.size(); c++) 
	{
		vector<Point> hull;
		convexHull(contours[squar_index[c]], hull);//͹��

		//pick four point
		vector<Point> squar;
		size_t num = hull.size();
		if (num < 4)//������С�ڣ�
			continue;
		else if (num >= 4) {
			float max_area;
			for (int m = 0; m < num - 3; m++) {
				for (int n = m + 1; n < num - 2; n++) {
					for (int j = n + 1; j < num - 1; j++) {
						for (int k = j + 1; k < num; k++) {
							vector<Point> squar_tmp;
							squar_tmp.push_back(hull[m]);
							squar_tmp.push_back(hull[n]);
							squar_tmp.push_back(hull[j]);
							squar_tmp.push_back(hull[k]);
							if (m == 0 && n == 1 && j == 2 && k == 3) {
								max_area = fabs(contourArea(Mat(squar_tmp)));
								squar.clear();
								squar = squar_tmp;
							}
							else {
								float area = fabs(contourArea(Mat(squar_tmp)));
								if (area > max_area) {
									max_area = area;
									squar.clear();
									squar = squar_tmp;
								}
							}
						}
					}
				}
			}
		}

		if (squar.size() != 4) {//��������������ڣ�************************
			continue;
		}

		int num_board = 0;
		for (int i = 0; i < squar.size(); i++) {
			num_board += (squar[i].x < 10) || (squar[i].x > srcImage.cols - 10) ||
				(squar[i].y < 10) || (squar[i].y > srcImage.rows - 10);
		}
		if (num_board > 0) {    //���ڱ߽�
			continue;
		}
		//���ĵ�����
		vector<Point> squar_sort = squar;

		//sort_point(squar,squar_sort,srcImage);

		for (int i = 0; i < squar_sort.size(); i++) {
			point_img.clear();
			for (size_t num_p = 0; num_p < squar_sort.size(); num_p++) {
				// point_img.push_back(squar_sort[num_p] * (1 / minifactor));
				point_img.push_back(squar_sort[num_p]);
			}
		}
		temp_img.push_back(point_img);
	}
	if (temp_img.size() == 0) {
		return vector<Point2f>();
	}

	if (temp_img.size() > 2) {
		point_img.clear();
		// cout<<"ʶ��Ŀ�̫�࣡"<<endl;
		return point_img;
	}

	if (temp_img.size() == 2) {//�������������2
		double a1 = contourArea(temp_img[0], true);
		double a2 = contourArea(temp_img[1], true);
		// cout << "a1= " << a1 << endl << "a2= " << a2 << endl;
		if (a1 > a2) {
			point_img.clear();
			point_img = temp_img[0];
		}
	}
	vector<Point2f> point_temp = point_img;
	point_img.clear();
	point_img.push_back(point_temp[1]);
	point_img.push_back(point_temp[2]);
	point_img.push_back(point_temp[3]);
	point_img.push_back(point_temp[0]);
	return point_img;
}
vector<Point2f>ranking(vector<Point2f>point)//�ܹ��ĸ����������е�˳�����һ���Ĺ��ɣ��������Ķ�λ����Ӱ��
{
	int thelast;
	vector<Point2f> back_point;
	double max = 0;
	for (int i = 1; i < point.size(); i++)
	{
		double length;
		length = pow(point[0].x - point[i].x, 2) + pow(point[0].y - point[i].y, 2);
		if (length > max)
		{
			thelast = i;//ѡ��һ����Ϊ����ԭ�㣬������ϵʱ������Խ��ߣ�������ʵ��������ﻹ�����⣬��������Ƕȵı仯��
			//���ȵı仯����ܶ࣬�Խ��߲�һ�������һֱû�뵽���ʵİ취��
		}
	}
	back_point[0].x = point[0].x;
	back_point[0].y = point[0].y;
	back_point[3].x = point[thelast].x;
	back_point[3].y = point[thelast].y;
	int sign = 0;
	for (int i = 0; i < 4; i++)
	{
		if (i == 0 || i == thelast)
			continue;
		else if(sign==0)
		{
			back_point[2].x = point[i].x;
			back_point[2].y = point[i].y;
			sign++;
		}
		else if (sign == 1)
		{
			back_point[3].x = point[i].x;
			back_point[3].y = point[i].y;
		}
	}
	return back_point;
}
Mat pose_estimation(Mat& pose, vector<Point2f>pose_point)
{
	vector<Point3f>world_point;
	for (int i = 0; i < 3; i++)
		world_point[i].z = 0;
	world_point[0].x = world_point[0].y = 0;//Ϊ�򻯲����˴�ӡ�ĺ�ɫ������
	world_point[1].x = 170; world_point[1].y = 0;
	world_point[2].x = 0; world_point[2].y = 170;//170mm
	Mat cameramatrix = (Mat_<double>(3, 3) << 3132.9, 0, 2135.4, 0, 3143.6, 1593.2, 0, 0, 1.0000);
	Mat distCoeffs = (Mat_<double>(1, 5) << 0.1665, -0.3226, 0, 0, 0);
	Mat rvec(3, 3, DataType<double>::type);
	Mat tvec(3, 1, DataType<double>::type);
	solvePnP(world_point, pose_point, cameramatrix, distCoeffs, rvec, tvec, 1);
	double rm[9];
	cv::Mat rotM(3, 3, CV_64FC1, rm);
	Rodrigues(rvec, rotM);
	vector<Point3f> tempPointSet;
	tempPointSet[0].x = 0; tempPointSet[0].y = 0; tempPointSet[0].z = 0;
	tempPointSet[1].x = 100; tempPointSet[1].y = 0; tempPointSet[1].z = 0;
	tempPointSet[2].x = 0; tempPointSet[2].y = 100; tempPointSet[2].z = 0;
	tempPointSet[3].x = 0; tempPointSet[3].y = 0; tempPointSet[3].z = 100;
	vector<Point2f> imgpoints;
	projectPoints(tempPointSet, rvec, tvec, cameramatrix, distCoeffs, imgpoints);
	line(pose, imgpoints[0], imgpoints[1], Scalar(0, 0, 255), 5, 8, 0);
	line(pose, imgpoints[0], imgpoints[2], Scalar(0, 0, 255), 5, 8, 0);
	line(pose, imgpoints[0], imgpoints[3], Scalar(0, 0, 255), 5, 8, 0);
	double Ang_X, Ang_Y, Ang_Z, X, Y, Z;
	Ang_X = (asin(rotM.at<double>(1, 0) / cos(asin(-rotM.at<double>(2, 0)))) /pi)*180;
	Ang_Y = (asin(-rotM.at<double>(2, 0)) /pi)*180;
	Ang_Z = (asin(rotM.at<double>(2, 1) / cos(asin(-rotM.at<double>(2, 0)))) /pi) * 180;
	X = rotM.at<double>(0, 0) * world_point[1].x + rotM.at<double>(0, 1)  * world_point[1].y + rotM.at<double>(0, 2)  * world_point[1].z + tvec.at<double>(0, 0);
	Y = rotM.at<double>(1, 0) * world_point[1].x + rotM.at<double>(1, 1)  * world_point[1].y + rotM.at<double>(1, 2)  * world_point[1].z + tvec.at<double>(1, 0);
	Z = rotM.at<double>(2, 0) * world_point[1].x + rotM.at<double>(2, 1)  * world_point[1].y + rotM.at<double>(2, 2)  * world_point[1].z + tvec.at<double>(2, 0);
	cout << "X = " << X << "; Y = " << Y << "; Z = " << Z << endl;
	cout << "Ang_X = " << Ang_X << " ; Ang_Y = " << Ang_Y << " ; Ang_Z = " << Ang_Z << endl;//֮ǰ���þ�������㲢����Eigen�������������⣬һֱû���
	return pose;
}