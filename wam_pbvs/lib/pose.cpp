#include "pose.h"

pose::pose()
{

}

pose::~pose(){}

pose::pose(cv::Mat trans, cv::Mat matRot, int type)
{
  translation=trans;
  if(type==0) //using rotation matrix
  {
    rotation = matRot;
    screwRotation = getScrewRotation(matRot);
  }
  else //using screw motion vector
  {
    screwRotation = matRot;
    rotation = getRotationMatrix(matRot);
  }
}

cv::Mat pose::getScrewRotation(cv::Mat m)
{
  double angle,x,y,z; // variables for result
	double epsilon = 0.01; // margin to allow for rounding errors
	double epsilon2 = 0.1; // margin to distinguish between 0 and 180 degrees

	if ((abs(m.at<double>(0,1)-m.at<double>(1,0))< epsilon)
	  && (abs(m.at<double>(0,2)-m.at<double>(2,0))< epsilon)
	  && (abs(m.at<double>(1,2)-m.at<double>(2,1))< epsilon)) {
		// singularity found
		// first check for identity matrix which must have +1 for all terms
		//  in leading diagonaland zero in other terms
		if ((abs(m.at<double>(0,1)+m.at<double>(1,0)) < epsilon2)
		  && (abs(m.at<double>(0,2)+m.at<double>(2,0)) < epsilon2)
		  && (abs(m.at<double>(1,2)+m.at<double>(2,1)) < epsilon2)
		  && (abs(m.at<double>(0,0)+m.at<double>(1,1)+m.at<double>(2,2)-3) < epsilon2)) {
			// this singularity is identity matrix so angle = 0
			return (cv::Mat_<double>(3,1) << 0,0,0);//// zero angle, arbitrary
		}
		// otherwise this singularity is angle = 180
		angle = M_PI;
		double xx = (m.at<double>(0,0)+1)/2;
		double yy = (m.at<double>(1,1)+1)/2;
		double zz = (m.at<double>(2,2)+1)/2;
		double xy = (m.at<double>(0,1)+m.at<double>(1,0))/4;
		double xz = (m.at<double>(0,2)+m.at<double>(2,0))/4;
		double yz = (m.at<double>(1,2)+m.at<double>(2,1))/4;
		if ((xx > yy) && (xx > zz)) { // m.at<double>(0,0) is the largest diagonal term
			if (xx< epsilon) {
				x = 0;
				y = 0.7071;
				z = 0.7071;
			} else {
				x = sqrt(xx);
				y = xy/x;
				z = xz/x;
			}
		} else if (yy > zz) { // m.at<double>(1,1) is the largest diagonal term
			if (yy< epsilon) {
				x = 0.7071;
				y = 0;
				z = 0.7071;
			} else {
				y = sqrt(yy);
				x = xy/y;
				z = yz/y;
			}
		} else { // m.at<double>(2,2) is the largest diagonal term so base result on this
			if (zz< epsilon) {
				x = 0.7071;
				y = 0.7071;
				z = 0;
			} else {
				z = sqrt(zz);
				x = xz/z;
				y = yz/z;
			}
		}
    return (cv::Mat_<double>(3,1) << x*angle, y*angle, z*angle);
	}
	// as we have reached here there are no singularities so we can handle normally
	double s = sqrt((m.at<double>(2,1) - m.at<double>(1,2))*(m.at<double>(2,1) - m.at<double>(1,2))
		+(m.at<double>(0,2) - m.at<double>(2,0))*(m.at<double>(0,2) - m.at<double>(2,0))
		+(m.at<double>(1,0) - m.at<double>(0,1))*(m.at<double>(1,0) - m.at<double>(0,1))); // used to normalise
	if (abs(s) < 0.001) s=1;
		// prevent divide by zero, should not happen if matrix is orthogonal and should be
		// caught by singularity test above, but I've left it in just in case
	angle = acos(( m.at<double>(0,0) + m.at<double>(1,1) + m.at<double>(2,2) - 1)/2);
	x = (m.at<double>(2,1) - m.at<double>(1,2))/s;
	y = (m.at<double>(0,2) - m.at<double>(2,0))/s;
	z = (m.at<double>(1,0) - m.at<double>(0,1))/s;
   return (cv::Mat_<double>(3,1) << x*angle, y*angle, z*angle);
}

//screwVector: 3*1
cv::Mat pose::getRotationMatrix(cv::Mat screwVector)
{
  double angle = math_helper::FrobeniusNorm(screwVector);

  if(angle==0)
    return screwVector;//error happens.

  cv::Mat R(3, 3,cv::DataType<double>::type);

  screwVector = screwVector / angle;//normalize to unit.
  double x = screwVector.at<double>(0,0);
  double y = screwVector.at<double>(1,0);
  double z = screwVector.at<double>(2,0);
  double c = cos(angle);
  double s = sin(angle);
  double t = 1.0 - c;

   R.at<double>(0,0) = c + x*x*t;
   R.at<double>(1,1) = c + y*y*t;
   R.at<double>(2,2) = c + z*z*t;

   double tmp1 = x*y*t;
   double tmp2 = z*s;
   R.at<double>(1,0) = tmp1 + tmp2;
   R.at<double>(0,1) = tmp1 - tmp2;
   tmp1 = x*z*t;
   tmp2 = y*s;
   R.at<double>(2,0) = tmp1 - tmp2;
   R.at<double>(0,2) = tmp1 + tmp2;    tmp1 = y*z*t;
   tmp2 = x*s;
   R.at<double>(2,1) = tmp1 + tmp2;
   R.at<double>(1,2) = tmp1 - tmp2;
  return R;
}

cv::Mat pose::get_t()
{
  return translation;
}

cv::Mat pose::get_R()
{
  return rotation;
}

cv::Mat pose::get_thetaU()
{
  return screwRotation;
}

cv::Mat pose::getVelocityTransform(cv::Mat T)
{
  cv::Mat t = T.rowRange(0,3).col(3);
  cv::Mat R = T.rowRange(0,3).colRange(0,3);
  cv::Mat tx = math_helper::skewFromVect(t.at<double>(0,0), t.at<double>(1,0), t.at<double>(2,0));
  cv::Mat txR = tx * R;
  cv::Mat VNC(6, 6,cv::DataType<double>::type);
  R.copyTo(VNC.rowRange(0,3).colRange(0,3));
  txR.copyTo(VNC.rowRange(0,3).colRange(3,6));
  cv::Mat matZeros = cv::Mat::zeros(3,3,CV_32FC1);
  matZeros.copyTo(VNC.rowRange(3,6).colRange(0,3));
  R.copyTo(VNC.rowRange(3,6).colRange(3,6));

  return VNC;
}


//return T: from end effector to robot base
cv::Mat pose::getTransformationE2B(cv::Mat trans_e2b, cv::Mat rot_b2e)
{
  cv::Mat rot_e2b = rot_b2e.t();
  cv::Mat T(4, 4,cv::DataType<double>::type);
  rot_e2b.copyTo(T.rowRange(0,3).colRange(0,3));
  trans_e2b.copyTo(T.rowRange(0,3).col(3));
  T.at<double>(3,0) = 0; T.at<double>(3,1) = 0; T.at<double>(3,2) = 0;T.at<double>(3,3) = 1;
  return T;
}
