/*************************************************************************
	> File Name: triangular_dist.h
	> Author:
	> Mail:
	> Created Time: Mon 20 Feb 2017 09:23:05 PM MST
 ************************************************************************/

#ifndef _TRIANGULAR_DIST_H
#define _TRIANGULAR_DIST_H
#endif
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
using namespace cv;
class math_helper
{
    public:
       math_helper(double b);
       ~math_helper();
       double getPx(double x);
       double getRandX();
       double getRandY();
       static vector<short int> getU_Path(vector < vector < Point2d> > _inputPoints);//get U path from vector vector point
       static vector<short int> getV_Path(vector < vector < Point2d> > _inputPoints);//get V path from vector vector point
       //static vector< vector <path_point> > getBackStrokes(vector < vector < Point> > _inputPoints, vector<Eigen::Vector4f> path_robot_r, <Eigen::Vector4f> normals_robot_r);
      // static vector< vector <path_point> > getBackStrokes(vector < vector < Point> > _inputPoints, const normal_surface_calc::targetPoints::ConstPtr& msg);

    private:
       double st_d;
};
