#include "chris_ICP.h"

chris_ICP::chris_ICP()
{}

chris_ICP::~chris_ICP()
{}

//src: N*3
//dst: N*3
cv::Mat chris_ICP::best_fit_transform(cv::Mat src, cv::Mat dst)
{
  //translate points to their centroids.
  cv::Point3d centroid_A = math_helper::getCentroid(src);
  cv::Point3d centroid_B = math_helper::getCentroid(dst);
  cv::Mat src2 = math_helper::translate2Centroid(src, centroid_A);
  cv::Mat dst2= math_helper::translate2Centroid(dst, centroid_B);

  //rotation Matrix
  cout<<"src2 : "<<src2.rows<<","<<src2.cols<<endl;
  cout<<"dst2 : "<<dst2.rows<<","<<dst2.cols<<endl;
  cv::Mat H = src2.t()*(dst2);
  //Singular value decomposition :
  cv::Mat A, S, U, Vt;
  cv::SVD::compute(H, S, U, Vt);//w singlular values, u: left singular vectors, vt:right singular vectors transpose//H=U W V.t()
  cout<<"Vt : "<<Vt.rows<<","<<Vt.cols<<endl;
  cout<<"Vt : "<<U.rows<<","<<U.cols<<endl;
  cv::Mat R = Vt.t()*(U.t());
  //special reflection case
  if(cv::determinant(R)<0)
  {
    for(int i=0;i<3;i++)
      Vt.at<double>(2,i) *=-1;
    R = Vt.t()*(U.t());
  }

  //translation
  cv::Mat c_A = math_helper::Point2Mat(centroid_A);
  cv::Mat c_B = math_helper::Point2Mat(centroid_B);
  cv::Mat t = c_B - R*(c_A);

  //copy rotation and translation to T (4*4)
  cv::Mat T=cv::Mat::zeros(4,4,cv::DataType<double>::type);
  T.at<double>(3,3) = 1;
  t.copyTo(T.rowRange(0,3).col(3));
  R.copyTo(T.rowRange(0,3).colRange(0,3));

  return T;
}

void chris_ICP::test(string filePath )
{
  std::ifstream input( filePath.c_str() );
  std::vector<cv::Point3d> srcV;
  std::vector<cv::Point3d> dstV;
  for( std::string line; getline( input, line ); )
  {
    std::vector<double> r = string_convertor::fromString2Array(line);
    srcV.push_back(cv::Point3d(r[2],r[3],r[4]));
    dstV.push_back(cv::Point3d(r[5],r[6],r[7]));
  }
  cv::Mat src = math_helper::vectorPoints2Mat(srcV);
  cv::Mat dst = math_helper::vectorPoints2Mat(dstV);
  cv::Mat T = chris_ICP::best_fit_transform2(src, dst);
  std::cout<<"transformation matrix"<<std::endl<<T<<endl;
}

//src: N*3
//dst: N*3
cv::Mat chris_ICP::solve_Rotation_SVD(cv::Mat src, cv::Mat dst)//using SVD to solve the rotation matrix
{
  //translate points to their centroids.
  cv::Point3d centroid_A = math_helper::getCentroid(src);
  cv::Point3d centroid_B = math_helper::getCentroid(dst);
  cv::Mat src2 = math_helper::translate2Centroid(src, centroid_A);
  cv::Mat dst2= math_helper::translate2Centroid(dst, centroid_B);

  //rotation Matrix
  cout<<"src2 : "<<src2.rows<<","<<src2.cols<<endl;
  cout<<"dst2 : "<<dst2.rows<<","<<dst2.cols<<endl;
  cv::Mat H = src2.t()*(dst2);
  //Singular value decomposition :
  cv::Mat A, S, U, Vt;
  cv::SVD::compute(H, S, U, Vt);//w singlular values, u: left singular vectors, vt:right singular vectors transpose//H=U W V.t()
  cout<<"Vt : "<<Vt.rows<<","<<Vt.cols<<endl;
  cout<<"Vt : "<<U.rows<<","<<U.cols<<endl;
  cv::Mat R = Vt.t()*(U.t());
  //special reflection case, make sure R is positive
  if(cv::determinant(R)<0)
  {
    for(int i=0;i<3;i++)
      Vt.at<double>(2,i) *=-1;
    R = Vt.t()*(U.t());
  }
  return R;
}

//src : N *3
//dst : N * 3
//R : 3 * 3
cv::Mat chris_ICP::solve_Scaling_Matrix(cv::Mat src, cv::Mat dst, cv::Mat R)
{
    //dst.t() = R* src.t()
    //cout<<"src : "<<src.rows<<","<<src.cols<<endl;
    //cout<<"dst : "<<dst.rows<<","<<dst.cols<<endl;
    cv::Mat src2= src.t();
    cv::Mat dst2 = dst.t();
    //cout<<"src2 : "<<src2.rows<<","<<src2.cols<<endl;
  //  cout<<"dst2 : "<<dst2.rows<<","<<dst2.cols<<endl;
    cv::Mat C=dst2 * src;
  //  cout<<"C : "<<C.rows<<","<<C.cols<<endl;

    cv::Mat D=cv::Mat::zeros(3,3,cv::DataType<double>::type);
    for(int k=0;k<3;k++)
    {
      double k_up =0;
      double k_down =0;
      for(int i=0;i<3;i++)
        k_up += C.at<double>(i,k)*R.at<double>(i,k);
      for(int i=0;i<dst2.cols;i++)
        k_down += src2.at<double>(k,i)*src2.at<double>(k,i);

      D.at<double>(k,k) =k_up/k_down;
    }
    return D;
}

//src: N*3
//dst: N*3
cv::Mat chris_ICP::best_fit_transform2(cv::Mat src, cv::Mat dst)
{
  cv::Point3d centroid_A = math_helper::getCentroid(src);
  cv::Point3d centroid_B = math_helper::getCentroid(dst);
  cv::Mat src3 = math_helper::translate2Centroid(src, centroid_A);
  cv::Mat dst3= math_helper::translate2Centroid(dst, centroid_B);

  //using iterations.
  cv::Mat src2= src3.t();//3*N
  cv::Mat dst2 = dst3.t();
  //dst2 = R*D*src2 + t
  cv::Mat D = (cv::Mat_<double>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
  cv::Mat R = (cv::Mat_<double>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
  double error_init = math_helper::FrobeniusNorm(dst2 - R*D*src2);
  double error_diff = 9999;
  double converg_thres = 0.000000001;
  int it_count = 0;
  int max_it = 200;
  while(it_count < max_it && error_diff > converg_thres)
  {
    //update src;
    src3 = (D * src2).t();
    //update R
    R = solve_Rotation_SVD(src3, dst3);
    //update D
    D = solve_Scaling_Matrix(src3, dst3, R);
    //update error
    double error = math_helper::FrobeniusNorm(dst3.t() - R*D*(src3.t()));
    //update error_diff
    error_diff = abs(error - error_init);
    //update error_init
    error_init = error;

    cout<<"Rotation matrix: R "<<endl<<R<<endl;
    cout<<"scale matrix: D "<<endl<<D<<endl;
    cout<<"error "<<endl<<error<<endl;
    cout<<"error_diff "<<endl<<error_diff<<endl;

  }
  //solving translation
  //translation
  cv::Mat c_A = math_helper::Point2Mat(centroid_A);
  cv::Mat c_B = math_helper::Point2Mat(centroid_B);
  cv::Mat t = c_B - R*D*(c_A);
  cout<<"translation "<<endl<<t<<endl;
  // //or
  // cv::Mat t_opt = dst3.t() - R*D*(src3.t()); //t_opt: 3*N
  // cout<<"t_opt : "<<t_opt.rows<<","<<t_opt.cols<<endl;
  // cout<<"t_opt : "<<t_opt<<endl;
  // cv::Point3d t_p = math_helper::getCentroid(t_opt.t());
  // t = (cv::Mat_<double>(3,1) << t_p.x, t_p.y, t_p.z);
  //cout<<"translation "<<endl<<t_p<<endl;
  //copy rotation and translation to T (4*4)
  cv::Mat T=cv::Mat::zeros(4,4,cv::DataType<double>::type);


  T.at<double>(3,3) = 1;
  t.copyTo(T.rowRange(0,3).col(3));
  R.copyTo(T.rowRange(0,3).colRange(0,3));

  //recheck the final error.
  cv::Mat src_h = math_helper::Mat2Homogenous(src3);
  cv::Mat dst_h = math_helper::Mat2Homogenous(dst3);
  cout<<"src_h : "<<src_h.rows<<","<<src_h.cols<<endl;
  cout<<"dst_h : "<<dst_h.rows<<","<<dst_h.cols<<endl;
  //dst_h =  T * src_h
  double finalError = math_helper::FrobeniusNorm(dst_h - T*src_h);
  cout<<"final error "<<endl<<finalError<<endl;
  return T;
}

//src: N*3
//dst: N*3
//T: 4*4
double chris_ICP::transformError(cv::Mat src, cv::Mat dst, cv::Mat T)
{

}
