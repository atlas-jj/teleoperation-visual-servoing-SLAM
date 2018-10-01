#pragma once
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <fstream>
#include <exception>
#include <vector>
#include <opencv2/opencv.hpp>
#ifndef Included_MATH_HELPER_H
#define Included_MATH_HELPER_H
#include "math_helper.h"
#endif
#ifndef Included_STRING_CONVERTOR_H
#define Included_STRING_CONVERTOR_H
#include "string_convertor.h"
#endif
using namespace std;

class chris_ICP
{
    public:
       chris_ICP();
       ~chris_ICP();
       static cv::Mat best_fit_transform(cv::Mat src, cv::Mat dst);//N*3 Matrix
       static void test(string filePath );//test best_fit_transform
       static cv::Mat solve_Rotation_SVD(cv::Mat src, cv::Mat dst);//using SVD to solve the rotation matrix
       static cv::Mat solve_Scaling_Matrix(cv::Mat src, cv::Mat dst, cv::Mat R);
       static cv::Mat best_fit_transform2(cv::Mat src, cv::Mat dst);
       static double transformError(cv::Mat src, cv::Mat dst, cv::Mat T);
    private:

};
