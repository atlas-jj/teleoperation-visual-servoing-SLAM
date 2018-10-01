#pragma once
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <exception>
#include <vector>
// #include <ros/ros.h>
// #include <tf/transform_broadcaster.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/Pose.h>
// #include <geometry_msgs/Point.h>
#include <opencv2/opencv.hpp>
#include "math_helper.h"


class pose
{
  public:
    pose();
    ~pose();
    pose(cv::Mat trans, cv::Mat matRot, int type);//type 0: using rotation matrix,   type 1: using screw motion vector
    cv::Mat get_t();
    cv::Mat get_R();
    cv::Mat get_thetaU();
    static cv::Mat getScrewRotation(cv::Mat m);
    static cv::Mat getRotationMatrix(cv::Mat screwVector);
    static cv::Mat getVelocityTransform(cv::Mat T);//get velocity transformation from transfromation SE(3) T

    static cv::Mat getTransformationE2B(cv::Mat trans_e2b, cv::Mat rot_b2e);
  private:
    cv::Mat translation;//vector
    cv::Mat rotation;//SO(3) matrix
    cv::Mat screwRotation;//vector, rotation matrix expressed using screw motion.
};
