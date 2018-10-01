#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <exception>
#include <vector>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;
class transformation2D
{
   public:
      transformation2D();
      ~transformation2D();
      transformation2D(cv::Point _t, double _scale, double _rotation);
      cv::Point2d doTransformation(cv::Point2d p1);
      vector< vector <cv::Point2d> > doTransformation(cv::Point2d _transPoint, vector < vector < cv::Point> > _inputPoints, double _scale, double _rotation);//do transformation, make the first point be the same as transPoint
      //vector< vector <double> > doTransformation(Point2d _transPoint, vector < vector < Point> > _inputPoints, double _scale, double _rotation);//do transformation, make the first point be the same as transPoint

   private:
     cv::Point t;
     double scale;
     double rotation;
     cv::Point2d firstPoint;
};
