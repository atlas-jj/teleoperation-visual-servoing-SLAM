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
      transformation2D(Point _t, double _scale, double _rotation);
      Point2d doTransformation(Point2d p1);
      vector< vector <Point2d> > doTransformation(Point2d _transPoint, vector < vector < Point> > _inputPoints, double _scale, double _rotation);//do transformation, make the first point be the same as transPoint
      //vector< vector <double> > doTransformation(Point2d _transPoint, vector < vector < Point> > _inputPoints, double _scale, double _rotation);//do transformation, make the first point be the same as transPoint

   private:
     Point t;
     double scale;
     double rotation;
     Point2d firstPoint;
};
