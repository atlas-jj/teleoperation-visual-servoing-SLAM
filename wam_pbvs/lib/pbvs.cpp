#include "pbvs.h"
 
pbvs::pbvs()
{
  lamda=0.1;
  //normalizeJointScale = 0.01745329251;//1 degree
}
pbvs::~pbvs(){}
pbvs::pbvs(double _lamda)
{
  lamda=_lamda;
  //normalizeJointScale=_normalizeJointScale*0.01745329251;
} 

//T, is the current transformation from end effector frame to robot base.
//_J, is the jacobian defined as from joint space to linear velocity in robot base frame
//_p, is the current end effector pose defined in desired end effector frame
//_p, is composed of Rotation: from current end effector frame to desired frame
//   translation: origin of desired frame defined in current end effector frame
cv::Mat pbvs::compute_qdot(cv::Mat _J, pose _p, cv::Mat _T)
{
  //update J, and currentPose
  J=_J;
  currentPose = _p;
  T=_T;

  cv::Mat R = currentPose.get_R();
  cv::Mat t = currentPose.get_t();
  cout<<"R"<<endl<<R<<endl;
  cout<<"t"<<endl<<t<<endl;
  cv::Mat thetaU = currentPose.get_thetaU();
  cout<<"thetaU"<<endl<<thetaU<<endl;
  cv::Mat Ve_b = pose::getVelocityTransform(T);
  //cout<<"Ve_b"<<endl<<Ve_b<<endl;
  cv::Mat Ve_desired = -lamda * R.t() * t;
  cv::Mat We_desired = -lamda * thetaU;/////////////////////////////////////only changes here.....
  //cout<<"Ve_desired"<<endl<<Ve_desired<<endl;
  //cout<<"We_desired"<<endl<<We_desired<<endl;
  //stack two vectors together
  cv::Mat V(6, 1,cv::DataType<double>::type);
  Ve_desired.copyTo(V.rowRange(0,3).col(0));
  We_desired.copyTo(V.rowRange(3,6).col(0));
  cout<<"V desired"<<endl<<V<<endl;
  //do velocity transformation
  cv::Mat V_eb = Ve_b * V;
  cout<<"V_eb"<<endl<<V_eb<<endl;
  //solve persudo invers of jacobian
  cv::Mat inv_J = math_helper::pinv(J);
  cout<<"inv_J"<<endl<<inv_J<<endl;
  cv::Mat q_dot = inv_J * V_eb;
  cout<<"q_dot"<<endl<<q_dot<<endl;
  if(math_helper::FrobeniusNorm(inv_J)>100 )//in case of singularity
    for(size_t i=0;i<q_dot.rows;i++)
      q_dot.at<double>(i,0)=0.01;
  return q_dot;
  //you need to normalize delta_q, when you perform delta_q = q_dot * delta_t in your main controller
  //after that, you can send it to the robot.
}

//normalize the joint delta value for safety
//_normalizeJointScale : unit degrees
cv::Mat pbvs::normalize_qdot(cv::Mat q_dot, double _normalizeJointScale)
{
  double normalizeJointScale=_normalizeJointScale*0.01745329251;
  double norm = math_helper::FrobeniusNorm(q_dot);
  if(norm != 0)
    return q_dot * normalizeJointScale / norm;
  else
    return q_dot;
}


//normalize the joint delta value for safety, using default scale value, which is 1 degree
cv::Mat pbvs::normalize_qdot(cv::Mat q_dot)
{
  double normalizeJointScale=0.01745329251;
  double norm = math_helper::FrobeniusNorm(q_dot);
  if(norm != 0)
    return q_dot * normalizeJointScale / norm;
  else
    return q_dot;
} 

bool pbvs::check_qdot(cv::Mat qdot, double threshold)
{
  double thres = threshold*0.01745329251;
  for(size_t i=0;i<qdot.rows;i++)
    if(abs(qdot.at<double>(i,0))>thres)
        return false;
  return true;
}

//return relative pose
pose pbvs::getRelativePose(cv::Mat trans_e2b, cv::Mat rot_b2e, cv::Mat trans_ed2b, cv::Mat rot_b2ed)
{
  cv::Mat ed_e_B = trans_e2b - trans_ed2b;
  cv::Mat trans_e2ed = rot_b2ed * ed_e_B;
  cv::Mat rot_e2ed = rot_b2ed * rot_b2e.t();
  pose p(trans_e2ed, rot_e2ed, 0);
  //cout<<"relativePost: "<<endl<<"trans:"<<endl<<p.get_t()<<endl<<"rot:"<<endl<<p.get_R()<<endl;
  return p;
}
