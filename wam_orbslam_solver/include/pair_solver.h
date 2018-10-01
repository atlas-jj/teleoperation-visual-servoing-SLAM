#pragma once
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <exception>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include "keyFrame.h"
//#include "icpPointToPoint.h"
#include "chris_ICP.h"
// #include <pcl_ros/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/registration/icp.h>

using namespace std;

class pair_solver
{
    public:
       pair_solver(int _numThres);
       ~pair_solver();
       int AddNewPair(keyFrame r_kf, keyFrame c_kf);
       int GetPairCount();
       void Save2File();
       bool isReady();
       void solveTranslationICP();//using icp and translation correspondents to solve the final transformation T
       void solveOrientationSDP(cv::Mat R_c0_r);//using least square and SVD to solve the final rotation from camera to end effector.
       cv::Mat getT_initCam();//return _T_translation
       cv::Mat getR_initCam2Robot();//return _R_initCam2Robot
       cv::Mat getR_cam2e();//return _R_cam2e
       void solveOrientationSDP();
    private:
      cv::Mat _T_translation;//transformation matrix T 4X4, from camera to robot
      cv::Mat _R_initCam2Robot;//rotation matrix, from initial camera frame to robot base frame.
      cv::Mat _R_cam2e;//rotation matrix R 3X3, from camera to end effector

      int pairCount;
      bool readyMark;
      int numThres;// = 100;//pairs needed for solver.
      std::vector< keyFrame > RobotKFs;
      std::vector< keyFrame > CameraKFs;

};
