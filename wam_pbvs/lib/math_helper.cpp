/*************************************************************************
	> File Name: norm_dist.cpp
	> Jun Jin.:
	> Mail:jjin5@ualberta.ca
    > ---- norm distribution functions.----
	> Created Time: Mon 20 Feb 2017 09:23:14 PM MST
 ************************************************************************/

#include "math_helper.h"


 vector<short int> math_helper::getU_Path(vector < vector < cv::Point2d> > _inputPoints)//get U path from vector vector point
 {
    size_t strokesNum=_inputPoints.size();
    vector<short int> u_path;
    for(int i=0;i<strokesNum;i++)
       for(int j=0;j<_inputPoints[i].size();j++)
       {
         u_path.push_back((int)(_inputPoints[i][j].x));
       }
    return u_path;
 }

 vector<short int> math_helper::getV_Path(vector < vector < cv::Point2d> > _inputPoints)//get V path from vector vector point
 {
   size_t strokesNum=_inputPoints.size();
   vector<short int> v_path;
   for(int i=0;i<strokesNum;i++)
      for(int j=0;j<_inputPoints[i].size();j++)
      {
        v_path.push_back((int)(_inputPoints[i][j].y));
      }
   return v_path;
 }

vector<double> math_helper::getInverseQuaternion(double qx, double qy, double qz, double qw)
{
  double qsquares = qx*qx + qy*qy + qz*qz + qw*qw;
  vector<double> rt;

  rt.push_back(-qx/qsquares);
  rt.push_back(-qy/qsquares);
  rt.push_back(-qz/qsquares);
  rt.push_back(qw/qsquares);

  return rt;
}

cv::Point3d math_helper::getCentroid(cv::Mat src)
{
  double fx;
  double fy;
  double fz;
  size_t numP=src.rows;
  for(size_t i=0;i<numP;i++)
  {
    fx += src.at<double>(i,0);
    fy += src.at<double>(i,1);
    fz += src.at<double>(i,2);
  }
  return cv::Point3d(fx/numP, fy/numP, fz/numP);
}

cv::Mat math_helper::translate2Centroid(cv::Mat src, cv::Point3d cp)
{
  size_t numP=src.rows;
  for(size_t i=0;i<numP;i++)
  {
     src.at<double>(i,0)= src.at<double>(i,0) - cp.x;
     src.at<double>(i,1)= src.at<double>(i,1) - cp.y;
     src.at<double>(i,2)= src.at<double>(i,2) - cp.z;
  }
  return src;
}

cv::Mat math_helper::translate2Centroid(cv::Mat src)
{
  cv::Point3d cp = getCentroid(src);
  return translate2Centroid(src, cp);
}

cv::Mat MatDotProduct(cv::Mat m1, cv::Mat m2)
{

}

cv::Mat math_helper::Point2Mat(cv::Point3d p)
{
  cv::Mat t(3,1,cv::DataType<double>::type);
  t.at<double>(0,0) = p.x;
  t.at<double>(1,0) = p.y;
  t.at<double>(2,0) = p.z;
  return t;
}

//stack matrix together in rows.
cv::Mat math_helper::stackMatVertic(std::vector<cv::Mat> srcV)
{
  size_t num = srcV.size();
  int cols = srcV[0].cols;
  int rows = srcV[0].rows;

  cv::Mat t(num * rows, cols,cv::DataType<double>::type);

  for(size_t i=0;i<num;i++)
    srcV[i].copyTo(t.rowRange(i*rows,(i+1)*rows).colRange(0,cols));
  return t;
}


//stack matrix together in rows.
cv::Mat math_helper::stackMatHoriz(std::vector<cv::Mat> srcV)
{
  size_t num = srcV.size();
  int cols = srcV[0].cols;
  int rows = srcV[0].rows;

  cv::Mat t(rows, num * cols,cv::DataType<double>::type);

  for(size_t i=0;i<num;i++)
    srcV[i].copyTo(t.rowRange(0,rows).colRange(i * cols,(i+1)*cols));
  return t;
}

cv::Mat math_helper::vectorPoints2Mat(std::vector<cv::Point3d> ps)
{
  size_t pnum = ps.size();
  cv::Mat T(pnum,3,cv::DataType<double>::type);
  for(size_t i=0;i<pnum;i++)
  {
      T.at<double>(i,0) = ps[i].x;
      T.at<double>(i,1) = ps[i].y;
      T.at<double>(i,2) = ps[i].z;
  }
  return T;
}

double math_helper::FrobeniusNorm(cv::Mat src)
{
  double norm = 0;
  for (size_t i=0; i<src.rows; i++)
    for (size_t j=0; j<src.cols; j++)
      norm += src.at<double>(i,j)*src.at<double>(i,j);
  return sqrt(norm);

}

//src : N*3
//output: 4*N
cv::Mat math_helper::Mat2Homogenous(cv::Mat src)
{
   cv::Mat rt(src.rows,4,cv::DataType<double>::type);
   src.copyTo(rt.rowRange(0,src.rows).colRange(0,3));
   for(size_t i=0; i< src.rows; i++)
       rt.at<double>(i,3) =1;
   return rt.t();
}

//dig mat is a vector only defines its diagnal values.
cv::Mat math_helper::inverseDiagMat(cv::Mat digMat)
{
  size_t matSize = digMat.rows;
  cv::Mat matZeros = cv::Mat::zeros(matSize,matSize,CV_64FC1);
  for(size_t i = 0; i< matSize;i++)
  {
    if( abs(digMat.at<double>(i,0)) >0.000001)
      matZeros.at<double>(i,i) = 1/digMat.at<double>(i,0);
    else
      matZeros.at<double>(i,i) = 1;
  }
  return matZeros;
}

cv::Mat math_helper::pinv(cv::Mat mat)
{
//   cv::Mat S, U, Vt;
//
//   cv::SVD::compute(mat, S, U, Vt);//s singlular values, u: left singular vectors, vt:right singular vectors transpose
//   //J=U s V.t()
//   cout<<"S"<<endl<<S<<endl;
//   cv::Mat inv_S = inverseDiagMat(S);
//
//   cout<<"inv_S"<<endl<<inv_S<<endl;
//   cv::invert()
  cv::Mat inv_J;// = Vt.t() * inv_S * U.t();
  cv::invert(mat, inv_J, cv::DECOMP_SVD);
  return inv_J;
}

cv::Mat math_helper::skewFromVect(double x1, double x2, double x3)
{
  cv::Mat T(3,3,cv::DataType<double>::type);
  T.at<double>(0,0) = 0; T.at<double>(0,1) = -x3; T.at<double>(0,2) = x2;
  T.at<double>(1,0) = x3; T.at<double>(1,1) = 0; T.at<double>(1,2) = -x1;
  T.at<double>(2,0) = -x2; T.at<double>(2,1) = x1; T.at<double>(2,2) = 0;
  return T;
}

cv::Mat math_helper::getIdentity()
{
  return (cv::Mat_<double>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
}

// /**normals_robot_r must be normalized.
// */
//  vector< vector <path_point> > math_helper::getBackStrokes(vector < vector < Point> > _inputPoints, vector<Eigen::Vector4f> path_robot_r, <Eigen::Vector4f> normals_robot_r)
//  {
//    vector< vector <path_point> > rtVects;
//      int strokesNum=_inputPoints.size();
//      int counter=0;
//      for(int i=0;i<strokesNum;i++)
//      {
//         vector<path_point> thisStroke;
//         for(int j=0;j<_inputPoints[i].size();j++)
//         {
//             Point3d _p(path_robot_r[counter].x(), path_robot_r[counter].y(), path_robot_r[counter].z());
//             Point3d _n(normals_robot_r[counter].x(), normals_robot_r[counter].y(), normals_robot_r[counter].z());
//             path_point thisP(_p,_n);
//             thisStroke.push_back(path_point);
//             counter++;
//         }
//         rtVects.push_back(thisStroke);
//       }
//       return rtVects;
//  }
