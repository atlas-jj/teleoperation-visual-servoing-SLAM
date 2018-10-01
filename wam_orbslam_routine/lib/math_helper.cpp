/*************************************************************************
	> File Name: norm_dist.cpp
	> Jun Jin.:
	> Mail:jjin5@ualberta.ca
    > ---- norm distribution functions.----
	> Created Time: Mon 20 Feb 2017 09:23:14 PM MST
 ************************************************************************/

#include "../include/math_helper.h"


 vector<short int> math_helper::getU_Path(vector < vector < Point2d> > _inputPoints)//get U path from vector vector point
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

 vector<short int> math_helper::getV_Path(vector < vector < Point2d> > _inputPoints)//get V path from vector vector point
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
