#pragma once
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <exception>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include "math_helper.h"
#include "pose.h"

/*****
--PBVS controller
--under desired end effector's frame, desired pose are zero vectors.
--
****/
class pbvs
{
  public:
    pbvs();
    pbvs(double _lamda);//_normalizeJointScale: unit degree.
    ~pbvs();
    //_T, transformation from end effector frame to robot base frame.
    cv::Mat compute_qdot(cv::Mat _J, pose _p, cv::Mat _T);
    static cv::Mat normalize_qdot(cv::Mat q_dot , double _normalizeJointScale);
    static cv::Mat normalize_qdot(cv::Mat q_dot);
    static bool check_qdot(cv::Mat qdot, double threshold);//threshold, unit in degrees
    //return current pose defined in desired pose frame
    //_p_current, current pose defined in robot base.
    //p_desired, desired pose defined in robot base.
    //which is translation: end effector origin defined in robot base.
    //rotation: R from robot base to end effector, is exactly WAM arm's output.
    //trans_e2b: current frame origin defined in base frame.
    //rot_b2e: rotation from base frame to current frame.
    //trans_ed2b: desired frame origin defined in base frame
    //rot_b2ed: rotation from base frame to desired frame.
    static pose getRelativePose(cv::Mat trans_e2b, cv::Mat rot_b2e, cv::Mat trans_ed2b, cv::Mat rot_b2ed);

  private:
    cv::Mat J; // jacobian of robot.
    pose currentPose;
    cv::Mat T;//transformation from end effector frame to robot base frame
    double lamda;//lamda for PBVS control
    //double normalizeJointScale;//unit: radius, for safety, it's wise to keep all joint updates at the same scale.


};
