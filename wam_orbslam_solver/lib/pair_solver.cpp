#include "pair_solver.h"

pair_solver::pair_solver(int _numThres)
{
  pairCount=0;
  numThres = _numThres;
  readyMark=false;
}

pair_solver::~pair_solver()
{}

bool pair_solver::isReady()
{
  return readyMark;
}

int pair_solver::AddNewPair(keyFrame r_kf, keyFrame c_kf)
{
  if(pairCount<numThres)
  {
    pairCount++;
    RobotKFs.push_back(r_kf);
    CameraKFs.push_back(c_kf);
  }
  // if(pairCount>= numThres)
  //   readyMark=true;
}

int pair_solver::GetPairCount()
{
  return pairCount;
}

//use translation pairs to solve the transfromation from camera to robot
void pair_solver::solveTranslationICP()
{
  std::vector<cv::Point3d> srcV;
  std::vector<cv::Point3d> dstV;

  for(size_t k=0;k<pairCount;k++)
  {
    keyFrame ckf = CameraKFs[k];
    keyFrame rkf = RobotKFs[k];
    srcV.push_back(cv::Point3d(ckf.translation.at<double>(0,0), ckf.translation.at<double>(1,0), ckf.translation.at<double>(2,0)));
    dstV.push_back(cv::Point3d(rkf.translation.at<double>(0,0), rkf.translation.at<double>(1,0), rkf.translation.at<double>(2,0)));
  }

  cv::Mat src = math_helper::vectorPoints2Mat(srcV);
  cv::Mat dst = math_helper::vectorPoints2Mat(dstV);
  _T_translation = chris_ICP::best_fit_transform2(src, dst);
  _R_initCam2Robot = _T_translation.rowRange(0,3).colRange(0,3);
  std::cout<<"transformation matrix"<<std::endl<<_T_translation<<endl;
}

/**previous code using libicp
int32_t dim =3;
int32_t num = pairCount;
// allocate model and template memory
double* M = (double*)calloc(3*num,sizeof(double));
double* T = (double*)calloc(3*num,sizeof(double));
cout << endl << "model created!" << endl;
for(size_t k=0;k<num;k++)
{
  keyFrame ckf = CameraKFs[k];
  keyFrame rkf = RobotKFs[k];
  M[k*3+0] = ckf.translation.at<double>(0,0);// = qx; t.at<double>(1,0) = qy; t.at<double>(2,0) = qz;;
  M[k*3+1] = ckf.translation.at<double>(1,0);
  M[k*3+2] = ckf.translation.at<double>(2,0);
  T[k*3+0] = rkf.translation.at<double>(0,0);// = qx; t.at<double>(1,0) = qy; t.at<double>(2,0) = qz;;
  T[k*3+1] = rkf.translation.at<double>(1,0);
  T[k*3+2] = rkf.translation.at<double>(2,0);
}
// start with identity as initial transformation
// in practice you might want to use some kind of prediction here
Matrix R = Matrix::eye(3);
Matrix t(3,1);
// run point-to-plane ICP (-1 = no outlier threshold)
cout << endl << "Running ICP (point-to-plane, no outliers)" << endl;
IcpPointToPoint icp(M,num,dim);
icp.fit(T,num,R,t,-1);

// results
cout << endl << "Transformation results:" << endl;
cout << "R:" << endl << R << endl << endl;
cout << "t:" << endl << t << endl << endl;

// free memory
free(M);
free(T);
**/

/** PCL code example for solving icp
// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
// //in data: camera coodinates.
// //out data: end effector coodinates.
//
// std::vector< keyFrame > RobotKFs;
// std::vector< keyFrame > CameraKFs;
//
// // Fill in the CloudIn data
// cloud_in->width    = pairCount;
// cloud_in->height   = 1;
// cloud_in->is_dense = false;
// cloud_in->points.resize (cloud_in->width * cloud_in->height);
//
// cloud_out->width    = pairCount;
// cloud_out->height   = 1;
// cloud_out->is_dense = false;
// cloud_out->points.resize (cloud_out->width * cloud_out->height);
//
// for (size_t i = 0; i < cloud_in->points.size (); ++i)
// {
//   keyFrame ckf = CameraKFs[i];
//   keyFrame rkf = RobotKFs[i];
//   cloud_in->points[i].x = ckf.translation.at<double>(0,0);// = qx; t.at<double>(1,0) = qy; t.at<double>(2,0) = qz;;
//   cloud_in->points[i].y = ckf.translation.at<double>(1,0);
//   cloud_in->points[i].z = ckf.translation.at<double>(2,0);
//   cloud_out->points[i].x = rkf.translation.at<double>(0,0);// = qx; t.at<double>(1,0) = qy; t.at<double>(2,0) = qz;;
//   cloud_out->points[i].y = rkf.translation.at<double>(1,0);
//   cloud_out->points[i].z = rkf.translation.at<double>(2,0);
// }
// std::cout << "Saved " << cloud_in->points.size () << " data points to input:" << std::endl;
//
// pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
// icp.setInputCloud(cloud_in);
// icp.setInputTarget(cloud_out);
// pcl::PointCloud<pcl::PointXYZ> Final;
// icp.align(Final);
// std::cout << "has converged:" << icp.hasConverged() << " score: " <<
// icp.getFitnessScore() << std::endl;
// std::cout << icp.getFinalTransformation() << std::endl;
**/

//use the orientation pairs to solve the rotation from camera to robot.
//R_c0_r: rotation from initial camera frame to robot base
//output rotation matrix, from end effector to camera
void pair_solver::solveOrientationSDP(cv::Mat R_c0_r)
{
  std::vector<cv::Mat> srcV;
  std::vector<cv::Mat> dstV;

  for(size_t k=0;k<pairCount;k++)
  {
    keyFrame ckf = CameraKFs[k];
    keyFrame rkf = RobotKFs[k];
    srcV.push_back(rkf.rotation * R_c0_r);
    dstV.push_back(ckf.rotation);
  }

  cv::Mat src = math_helper::stackMatHoriz(srcV);
  cv::Mat dst = math_helper::stackMatHoriz(dstV);
  cout<<"src : "<<src.rows<<","<<src.cols<<endl;
  cout<<"dst : "<<dst.rows<<","<<dst.cols<<endl;
  _R_cam2e = chris_ICP::solve_Rotation_SVD(src, dst);
  std::cout<<"_R_cam2e matrix"<<std::endl<<_R_cam2e<<endl;
}

//use the orientation pairs to solve the rotation from camera to robot.
//output rotation matrix, from end effector to camera
void pair_solver::solveOrientationSDP()
{
  std::vector<cv::Mat> srcV;
  std::vector<cv::Mat> dstV;

  for(size_t k=0;k<pairCount;k++)
  {
    keyFrame ckf = CameraKFs[k];
    keyFrame rkf = RobotKFs[k];
    srcV.push_back(rkf.rotation * _R_initCam2Robot);
    dstV.push_back(ckf.rotation);
  }

  cv::Mat src = math_helper::stackMatHoriz(srcV);
  cv::Mat dst = math_helper::stackMatHoriz(dstV);
  cout<<"src : "<<src.rows<<","<<src.cols<<endl;
  cout<<"dst : "<<dst.rows<<","<<dst.cols<<endl;
  _R_cam2e = chris_ICP::solve_Rotation_SVD(src.t(), dst.t());
  std::cout<<"_R_cam2e matrix"<<std::endl<<_R_cam2e<<endl;
  //validation
  double error = math_helper::FrobeniusNorm(dstV[1] - _R_cam2e * srcV[1]);
  std::cout<<"rotation error"<<std::endl<<error<<endl;
  cv::Mat directR = dstV[1] * srcV[1].t();
  std::cout<<"direct estimating Rotation matrix"<<std::endl<<directR<<endl;

}

//return _T_translation
cv::Mat pair_solver::getT_initCam()
{
  return _T_translation;
}

//return _R_orientation
cv::Mat pair_solver::getR_initCam2Robot()
{
  return _R_initCam2Robot;
}

cv::Mat pair_solver::getR_cam2e()
{
  return _R_cam2e;
}

void pair_solver::Save2File() {
  cout << endl << "Saving to file " <<  " ..." << endl;
  //save to file,
  //p.txt timesecs_c, cx, cy, cz, timesecs_r, rx, ry, rz
  ofstream f1;
  f1.open("p.txt");
  f1 << fixed;
  //q.txt timesecs_c, qcx, qcy, qcz, qcw, timesecs_r, qrx, qry, qrz, qrw
  ofstream f2;
  f2.open("q.txt");
  f2 << fixed;

  for(int i=0;i<pairCount;i++)
  {
    keyFrame ckf = CameraKFs[i];
    keyFrame rkf = RobotKFs[i];
    f1<<setprecision(2)<<ckf.timeStamp<<" "<<rkf.timeStamp<<" "<< setprecision(9) << ckf.translation.at<double>(0,0)<<" "<<ckf.translation.at<double>(1,0)<<" "<<
    ckf.translation.at<double>(2,0)<<" "<<rkf.translation.at<double>(0,0)<<" "<<rkf.translation.at<double>(1,0)<<" "<<
    rkf.translation.at<double>(2,0)<<endl;

    f2<<setprecision(2)<<ckf.timeStamp<<" "<<rkf.timeStamp<<" "<<setprecision(9)<<ckf.kq[0]<<" "<<ckf.kq[1]<<" "<<ckf.kq[2]<<" "<<ckf.kq[3]<<" "<<
    rkf.kq[0]<<" "<<rkf.kq[1]<<" "<<rkf.kq[2]<<" "<<rkf.kq[3]<<endl;
  }
  f1.close();
  f2.close();
  cout << endl << "paired samples saved!" << endl;

  readyMark=true;
}
