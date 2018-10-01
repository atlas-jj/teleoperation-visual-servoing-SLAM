#include "../include/transformation2D.h"

/** constuctor
_t: 2d translation
_scale: scale
_rotation: rotation in degrees

*/
transformation2D::transformation2D(Point _t, double _scale, double _rotation)
{
  t=_t;
  scale=_scale;
  rotation=_rotation;
}

transformation2D::transformation2D(){}

transformation2D::~transformation2D(){}

//do scale first
//then do translation to origin, i.e, make the first point be the origin point.
//then do rotation
//then do another translation, make the first point be the t point.
Point2d transformation2D::doTransformation(Point2d p1)
{
   //get the 2*3 transformation matrix
   //cout<<"p1"<<p1<<endl;
   double r=rotation*CV_PI/180;
   p1.x=p1.x*scale-firstPoint.x;
   p1.y=p1.y*scale-firstPoint.y;
   Mat_<double> transformation(2,3);
   transformation(0,0)=cos(r);transformation(0,1)=-sin(r);transformation(0,2)=t.x+firstPoint.x;
   transformation(1,0)=sin(r);transformation(1,1)=cos(r);transformation(1,2)=t.y+firstPoint.y;

   //std::cout<<"matrix:"<<transformation<<endl;
   cv::Mat_<double>p13(3,1);
   p13(0,0)=p1.x;
   p13(1,0)=p1.y;
   p13(2,0)=1.0;
   cv::Mat_<double> p2=transformation*p13;
   return Point2d(p2(0,0),p2(1,0));
}

vector< vector <Point2d> > transformation2D::doTransformation(Point2d _transPoint, vector < vector < Point> > _inputPoints, double _scale, double _rotation)
{
  scale=_scale;
  rotation=_rotation;
  vector< vector < Point2d> > outputPoints;
  Point relativeTransPoint(0,0);
  size_t strokesNum=_inputPoints.size();
  if(strokesNum>0)
   {
     firstPoint.x=_inputPoints[0][0].x*scale;
     firstPoint.y=_inputPoints[0][0].y*scale;
     relativeTransPoint.x=_transPoint.x - firstPoint.x;
     relativeTransPoint.y=_transPoint.y - firstPoint.y;
   }
   t=relativeTransPoint;

  for(int m=0;m<strokesNum;m++)
  {
    vector<Point2d> strokePoints;
    size_t pointNum=_inputPoints[m].size();
    for(int n=0;n<pointNum;n++)
    {
      Point2d tfPoint=doTransformation(_inputPoints[m][n]);
      strokePoints.push_back(tfPoint);
    }
    outputPoints.push_back(strokePoints);
  }

  return outputPoints;
}

// vector< vector <double> > transformation2D::doTransformation(Point2d _transPoint, vector < vector < Point> > _inputPoints, double _scale, double _rotation)
// {
//   vector< vector <Point2d> > tfPoints=doTransformation(Point2d _transPoint, vector < vector < Point> > _inputPoints, double _scale, double _rotation);
//
// }
