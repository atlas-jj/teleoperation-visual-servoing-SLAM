
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <map>
#include <ctime>
#include <sstream>
#include <algorithm>
#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "math_helper.h"
#include "string_convertor.h"
#include "transformation2D.h"
#include "signature_visualization/pathData.h"
#include "wam_msgs/MatrixMN.h"
#include "wam_srvs/JointMove.h"
#include "sensor_msgs/JointState.h"
#include "pose.h"
#include "pbvs.h"
#include "colormod.h"

//using namespace cv;
using namespace std;
Color::Modifier c_red(Color::FG_RED);
Color::Modifier c_yellow(Color::FG_YELLOW);
Color::Modifier c_green(Color::FG_GREEN);
Color::Modifier c_default(Color::FG_DEFAULT);


int dofNum = 6;
wam_srvs::JointMove mv_srv;
ros::ServiceClient Joint_move_client ;
cv::Mat current_trans;
cv::Mat current_rot;

cv::Mat currentJacobian(6,dofNum,cv::DataType<double>::type);
std::vector<double> current_Joint_pose;
bool ready_signal1 = false;
bool ready_signal2 = false;
bool ready_signal3 = false;
bool lock = false;
//scan along columns
void wamToolJacobianCallback(const wam_msgs::MatrixMN::ConstPtr& jacobianMessage)
{
  for (int i = 0; i < 6; i++)
  {
	  for (int j = 0; j < dofNum; j++)
		{
       currentJacobian.at<double>(i,j)=jacobianMessage->data[i+j*6];
       //currentJacobian.at<double>(i,1)=jacobianMessage->data[i+3*6];
    }
  }
  ready_signal1=true;
 }

 void wamPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
 {
   //cout<<"wam msgs"<<endl;
    geometry_msgs::Pose thisPose = msg->pose;
    cv::Mat t(3,1,cv::DataType<double>::type);
    cv::Mat R(3,3,cv::DataType<double>::type);

    t.at<double>(0,0) = thisPose.position.x; t.at<double>(1,0) = thisPose.position.y; t.at<double>(2,0) = thisPose.position.z;

    tf::Quaternion q(thisPose.orientation.x, thisPose.orientation.y , thisPose.orientation.z, thisPose.orientation.w);
    tf::Matrix3x3 rMatrix(q);
    for(int i=0;i<3;i++)
      for(int j=0;j<3;j++)
          R.at<double>(i,j)= rMatrix[i][j];
    current_trans = t;
    current_rot = R;
    ready_signal2=true;
 }

 void wamJointsCallback(const sensor_msgs::JointState::ConstPtr& msg)
 {
    current_Joint_pose=msg->position;
    //cout<<"joint pose obtained:"<<endl;//<<initial_Joint_pose[1]<<endl;
     //cout<<initial_Joint_pose[0]<<"  "<<initial_Joint_pose[1]<<"  "<<initial_Joint_pose[2]<<" "<<initial_Joint_pose[3]<<" "<< initial_Joint_pose[4]
    // <<" "<<initial_Joint_pose[5]<<" "<<initial_Joint_pose[6]<<endl;
    ready_signal3=true;
 }

 //inStr "0 0 0 0"
 void moveRobotByQdot(cv::Mat qdot)
 {
   lock=true;
   std::vector<float> jnts;
   for(size_t i=0;i<dofNum;i++)
   {
     float newJ = (float)(current_Joint_pose[i] + qdot.at<double>(i,0));
     jnts.push_back(newJ);
   }
   jnts.push_back(current_Joint_pose[6]);
   mv_srv.request.joints = jnts;
   cout<<"send to robot to the  position "<<endl;
   //string_convertor::printOutStdVector(jnts);
   // cout << "Press any key to continue..." << endl;
   // getchar();
   Joint_move_client.call(mv_srv);///////////////////////////////////////////////////
   //boost::this_thread::sleep( boost::posix_time::milliseconds(1000) );
   lock=false;
 }

double get_error(pose p, int mode)//mode=0, orientation;  mode=1, position
{
  if(mode == 1)
    return math_helper::FrobeniusNorm(p.get_t());
  else
  {
    double trR = cv::trace(p.get_R())[0];
    
    return acos((trR-1)/2);
  }
}

double error_convrge_thres(int mode)
{
  if(mode ==1)//position
    return 0.001;
  else
    return 0.0001;
}

double error_thres(int mode)
{
  if(mode ==1)
    return 0.01;
  else
    return 0.01;
}

void pbvsControl(cv::Mat _desired_trans_ed2b, cv::Mat _desired_rot_b2ed, int mode, double lamda)//mode=0, orientation;  mode=1, position
{
  //compute relative pose
  pose relativePose = pbvs::getRelativePose(current_trans, current_rot, _desired_trans_ed2b, _desired_rot_b2ed);
  //std::cout<<"relative Pose"<<endl<<relativePose<<endl;
  
  //PBVS while loop
  double error =get_error(relativePose, mode);
  std::cout<<c_green<<"error:  "<<error<<c_default<<endl;
  pbvs controller(lamda);
  double error_converge = 999;
  //getchar();
  while(abs(error_converge)>error_convrge_thres(mode)&&abs(error)>error_thres(mode))//0.01
  {
    cv::Mat T = pose::getTransformationE2B(current_trans, current_rot);
    std::cout<<"transformation T:"<<endl<<T<<endl;
  
    cv::Mat qdot = controller.compute_qdot(currentJacobian, relativePose, T);
    std::cout<<c_yellow<<"qdot"<<endl<<qdot * 57.2957795131<<c_default<<endl;
    //normalize qdot.
    //cv::Mat qdot_normalize = pbvs::normalize_qdot(qdot,10);
    //std::cout<<c_yellow<<"delta q"<<endl<<qdot_normalize * 57.2957795131<<c_default<<endl;
    std::cout <<c_green<< "current_error: " <<error<<"   error_converge: "<<error_converge<<c_default<< endl;
    std::cout <<c_red<< "command moving robot! Press any key to continue..." <<c_default<< endl;
    getchar();
    //boost::this_thread::sleep( boost::posix_time::milliseconds(500) );
    moveRobotByQdot(qdot);

    //reset ready_signal
    ready_signal1=false;ready_signal2=false;ready_signal3=false;
    while(!ready_signal1 || !ready_signal2||!ready_signal3)
    {
      ros::spinOnce();
      boost::this_thread::sleep( boost::posix_time::milliseconds(100) );
    }

    //update error
    std::cout<<"current pose"<<endl<<current_trans<<endl<<current_rot<<endl;
    relativePose = pbvs::getRelativePose(current_trans, current_rot, _desired_trans_ed2b, _desired_rot_b2ed);
    //T = pose::getTransformationE2B(current_trans, current_rot);
    double error_p =get_error(relativePose, mode);
    error_converge = error_p - error;
    error = error_p;

  }
}

//===========================MAIN FUNCTION START===========================

int main(int argc, char* argv[]){

  double incremental = 0.1;
  ros::init(argc, argv, "pbvs");
  ros::NodeHandle nh;
  ros::Subscriber subP = nh.subscribe("/zeus/wam/pose", 1, wamPoseCallback);
  ros::Subscriber jacobian_sub = nh.subscribe("zeus/wam/jacobian",1,wamToolJacobianCallback);
  ros::Subscriber wam_pos_sub=nh.subscribe("/zeus/wam/joint_states",1,wamJointsCallback);
  Joint_move_client = nh.serviceClient<wam_srvs::JointMove>("/zeus/wam/joint_move");


  while(!ready_signal1 || !ready_signal2||!ready_signal3)
  {
    ros::spinOnce();
    boost::this_thread::sleep( boost::posix_time::milliseconds(100) );
  }
  std::cout<<"current pose"<<endl<<current_trans<<endl<<current_rot<<endl;
  std::cout<<"current Jocobian"<<endl<<currentJacobian<<endl;
  //getchar();
  //set desired pose
  cv::Mat desired_trans_ed2b=current_trans + (cv::Mat_<double>(3,1) << 0.2,-0.2,-0.1);
  std::cout<<c_green<<"desired_trans_ed2b"<<endl<<desired_trans_ed2b<<c_default<<endl;
  cv::Mat testRot = (Mat_<double>(3,3)<<1.0000000,  0.0000000,  0.0000000,0.0000000,  0.8660254, -0.5000000,0.0000000,  0.5000000,  0.8660254 );
  cv::Mat desired_rot_b2ed = current_rot*testRot;//(Mat_<double>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
  std::cout<<c_red<<"desired_rot_b2ed"<<endl<<desired_rot_b2ed<<c_default<<endl;

  //firstly, do orientation change, keep position be fixed.
  //pbvsControl(current_trans,desired_rot_b2ed,0, 0.8);
  //then keep orientation and move the position.
  //pbvsControl(desired_trans_ed2b,current_rot,1,0.5);

  ros::spin();
    //declaration of function
  ros::shutdown();
  //Joint_move_client = nh.serviceClient<wam_srvs::JointMove>("/zeus/wam/joint_move");
//   cv::Mat C = (Mat_<double>(3,1) << -0.001665, -0.0163841, -0.4884146 );
//   cv::Mat R = pose::getRotationMatrix(C);
//
//   cout<<"rotation matrix: "<<endl<<R<<endl;
//   cv::Mat skewMotion = pose::getScrewRotation(R);
//   cout<<"screw motion vector: "<<endl<<skewMotion<<endl;
//   cv::Mat sm = math_helper::skewFromVect(0.1,0.2,0.3);
//   cout<<"screw sysmetric matrix: "<<endl<<sm<<endl;
  // ros::spinOnce();
    //cv::namedWindow("view");
    //cv::startWindowThread();

      //declaration of function
  	//cv::destroyWindow("view");
    return 0;
}
