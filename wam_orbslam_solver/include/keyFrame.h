#ifndef KEYFRAME_H
#define KEYFRAME_H
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <exception>
#include <vector>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <opencv2/opencv.hpp>

using namespace std;
class keyFrame
{
    public:
       keyFrame(int mnid, double timesecs, cv::Mat _TWC);
       keyFrame(int mnid, double timesecs, double x, double y, double z, double qx, double qy, double qz, double qw);
       keyFrame();
       ~keyFrame();
       int mnFrameId;
       double timeStamp;//secs
       //geometry_msgs::Point position;
       //geometry_msgs::Quaternion orientation;
       cv::Mat TWC;
       cv::Mat translation;
       cv::Mat rotation;

       tf::Quaternion kq;

       static double getBaseLine(cv::Mat t1, cv::Mat t2);


    private:
      void setQuaternion();

};

#endif
