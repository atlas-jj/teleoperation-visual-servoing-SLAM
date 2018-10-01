/*************************************************************************
	> File Name: triangular_dist.h
	> Author:
	> Mail:
	> Created Time: Mon 20 Feb 2017 09:23:05 PM MST
 ************************************************************************/

#ifndef _MATH_HELPER_H
#define _MATH_HELPER_H

#include <iostream>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <exception>
#include <vector>
#include <opencv2/opencv.hpp>
// #ifndef Included_PATH_POINT_H
//   #define Included_PATH_POINT_H
//   #include "path_point.h"
// #endif


using namespace std;

class math_helper
{
    public:
       math_helper(double b);
       ~math_helper();
       double getPx(double x);
       double getRandX();
       double getRandY();
       static vector<short int> getU_Path(vector < vector < cv::Point2d> > _inputPoints);//get U path from vector vector point
       static vector<short int> getV_Path(vector < vector < cv::Point2d> > _inputPoints);//get V path from vector vector point
       static vector<double> getInverseQuaternion(double qx, double qy, double qz, double qw);
       static cv::Point3d getCentroid(cv::Mat src);
       static cv::Mat translate2Centroid(cv::Mat src, cv::Point3d cp);
       static cv::Mat translate2Centroid(cv::Mat src);
       static cv::Mat Point2Mat(cv::Point3d p);
       static cv::Mat vectorPoints2Mat(std::vector<cv::Point3d> ps);
       static cv::Mat MatDotProduct(cv::Mat m1, cv::Mat m2);//return m1.dot(m2). opencv dot product only returns scalar values.
       //static vector< vector <path_point> > getBackStrokes(vector < vector < Point> > _inputPoints, vector<Eigen::Vector4f> path_robot_r, <Eigen::Vector4f> normals_robot_r);
      // static vector< vector <path_point> > getBackStrokes(vector < vector < Point> > _inputPoints, const normal_surface_calc::targetPoints::ConstPtr& msg);
       static double FrobeniusNorm(cv::Mat src);
       static cv::Mat Mat2Homogenous(cv::Mat src);//src N*3
       static cv::Mat stackMatVertic(std::vector<cv::Mat> srcV);//stack mat vector
       static cv::Mat stackMatHoriz(std::vector<cv::Mat> srcV);//stack mat vector horizonally
       static cv::Mat inverseDiagMat(cv::Mat digMat);
       static cv::Mat pinv(cv::Mat mat);
       //skew sysmetric from a vector
       static cv::Mat skewFromVect(double x1, double x2, double x3); //
       static cv::Mat getIdentity();
    private:
       double st_d;
};

#endif
