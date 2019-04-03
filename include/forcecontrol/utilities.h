#pragma once
#ifndef _EGMCLASS_ULTILITIES_H_
#define _EGMCLASS_ULTILITIES_H_


#ifdef __cplusplus
	#include <cmath>
	#include <iostream>
#else
	#include <math.h>
	#include <stdio.h>
#endif

// Eigen
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <unsupported/Eigen/MatrixFunctions>


namespace UT
{
  typedef Eigen::Vector3d Vector3d;
  typedef Eigen::Matrix3d Matrix3d;
  typedef Eigen::Matrix4d Matrix4d;
  typedef Eigen::MatrixXd MatrixXd;
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  typedef Eigen::Matrix<double, 6, 6> Matrix6d;
  typedef Eigen::Quaterniond Quaterniond;

  typedef Eigen::Vector3f Vector3f;
  typedef Eigen::Matrix3f Matrix3f;
  typedef Eigen::Matrix4f Matrix4f;
  typedef Eigen::MatrixXf MatrixXf;
  typedef Eigen::Matrix<float, 6, 1> Vector6f;
  typedef Eigen::Matrix<float, 6, 6> Matrix6f;
  typedef Eigen::Quaternionf Quaternionf;

  const static double kEpsilon = 1e-7;

	/////////////////////////////////////////////////////////////////////////
	//                   types and static variables
	/////////////////////////////////////////////////////////////////////////

	// typedef int (*printf_ptr) (const char *str, ...);
	// static printf_ptr _printf = printf;

	/////////////////////////////////////////////////////////////////////////
	//                          iostream
	/////////////////////////////////////////////////////////////////////////
  void stream_array_in(ostream &st, double *array, int length)
  {
    for (int i = 0; i<length; i++)
    {
     st << array[i];
     st << "\t";
   }
 }

 void stream_array_in(ostream &st, float *array, int length)
 {
  for (int i = 0; i<length; i++)
  {
   st << array[i];
   st << "\t";
 }
}


void stream_array_in(ostream &st, int *array, int length)
{
  for (int i = 0; i<length; i++)
  {
   st << array[i];
   st << "\t";
 }
}

	/////////////////////////////////////////////////////////////////////////
	//                          scalar
	/////////////////////////////////////////////////////////////////////////

static void truncate(double *ele, const double _min, const double _max)
{
  if ( (*ele) > _max)
    (*ele) = _max;
  else if ( (*ele) < _min)
    (*ele) = _min;
}

	/////////////////////////////////////////////////////////////////////////
	//                          vector&array
	/////////////////////////////////////////////////////////////////////////

static void buf_insert(const double ele, const int size, double * buf)
{
  for (int i = 1; i < size; ++i)
  {
    buf[size - i] = buf[size - 1 - i];
  }
  buf[0] = ele;
}

static void copyArray(const float *src, float *dest, int dim)
{
  for(int i = 0; i<dim; i++)
  {
    dest[i] = src[i];
  }
}

static void copyArray(const double *src, double *dest, int dim)
{
  for(int i = 0; i<dim; i++)
  {
    dest[i] = src[i];
  }
}

static void setArray(float *array, float value, int dim)
{
 for(int i=0; i<dim; i++)
 {
   array[i] = value;
 }
}

static void truncate(float *array, float min, float max, int dim)
{
 for(int i=0; i<dim; i++)
 {
  array[i] = (array[i] > max)? max:array[i];
  array[i] = (array[i] < min)? min:array[i];
  }
}

static double vec_max(const double * vec, const int size)
{
  double m = vec[0];
  double t1;
  for (int i = 0; i < size; ++i)
  {
   t1 = vec[i];
   if (t1 > m) m = t1;
 }
 return m;
}

static double vec_min(const double * vec, const int size)
{
  double m = vec[0];
  double t1;
  for (int i = 0; i < size; ++i)
  {
   t1 = vec[i];
   if (t1 < m) m = t1;
 }
 return m;
}

static double vec_mean(const double * vec, const int size)
{
  double sum = 0;
  for (int i = 0; i < size; ++i)
  {
   sum += vec[i];
 }
 return sum/double(size);
}


static double vec_slope(const double * x, const double * y,const int size)
{
 double avgX = vec_mean(x,size);
 double avgY = vec_mean(y,size);

 double numerator = 0.0;
 double denominator = 0.0;

 double xd = 0;
 for(int i=0; i<size; ++i)
 {
  xd = x[i] - avgX;
  numerator += (xd) * (y[i] - avgY);
  denominator += xd * xd;
}

return numerator / denominator;
}

    // numerical differentiation with low pass filter
    // input x, calculate dx/dt
    // s/(as+1),
static double diff_LPF(const double xdold, const double xnew, const double xold, const double T,const double a)
{
  double As = exp(-T/a);
  return As*xdold + (1 - As)*((xnew - xold)/T);
}

static void truncate6f(Vector6f *v, float min, float max)
{
  for(int i=0; i<6; i++)
  {
    (*v)[i] = ((*v)[i] > max)? max:(*v)[i];
    (*v)[i] = ((*v)[i] < min)? min:(*v)[i];
  }
}

static void truncate6d(Vector6d *v, double min, double max)
{
  for(int i=0; i<6; i++)
  {
    (*v)[i] = ((*v)[i] > max)? max:(*v)[i];
    (*v)[i] = ((*v)[i] < min)? min:(*v)[i];
  }
}

static void truncate6d(Vector6d *v, const Vector6d &min, const Vector6d &max)
{
  for(int i=0; i<6; i++)
  {
    (*v)[i] = ((*v)[i] > max[i])? max[i]:(*v)[i];
    (*v)[i] = ((*v)[i] < min[i])? min[i]:(*v)[i];
  }
}

static void stream_array_in6f(ostream &st, const Vector6f &array)
{
  for (int i = 0; i<6; i++)
  {
    st << array(i);
    st << "\t";
  }
}
static void stream_array_in6d(ostream &st, const Vector6d &array)
{
  for (int i = 0; i<6; i++)
  {
    st << array(i);
    st << "\t";
  }
}

/////////////////////////////////////////////////////////////////////////
//                          Matrices
/////////////////////////////////////////////////////////////////////////
MatrixXd pseudoInverse(const MatrixXd &a,
    double epsilon = numeric_limits<double>::epsilon()) {
  if (a.norm() < epsilon) {
    return  MatrixXd::Zero(a.cols(), a.rows());
  } else {
    Eigen::JacobiSVD< MatrixXd > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
    double tolerance = epsilon * max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
    return svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
  }
}

    /////////////////////////////////////////////////////////////////////////
    //                          Robotics
    /////////////////////////////////////////////////////////////////////////

    /*  Frames/spaces:
            W: world frame
            T: current tool frame
            So: set tool frame with offset
            Tf: transformed generalized space
        Quantities:
            SE3: 4x4 homogeneous coordinates
            se3: 6x1 twist coordinate of SE3
            spt: 6x1 special twist: 3x1 position, 3x1 exponential coordinate for rotation
            td: 6x1 time derivative of twist.
            v: 6x1 velocity, either spatial or body

            wrench: 6x1 wrench. Makes work with body velocity
    */

Matrix3d wedge(const Vector3d &v) {
  Matrix3d v_wedge;
  v_wedge << 0, -v(2), v(1),
  v(2), 0, -v(0),
  -v(1), v(0), 0;
  return v_wedge;
}

Matrix4d wedge6(const Vector6d &t) {
  Matrix4d t_wedge;
  t_wedge <<   0,   -t(5),   t(4),  t(0),
  t(5),     0,   -t(3),  t(1),
  -t(4),   t(3),     0,   t(2),
  0,     0,     0,     0;
  return t_wedge;
}

Matrix3d quat2SO3(const Quaterniond &q) {
  return q.normalized().toRotationMatrix();
}

Matrix3d quat2SO3(double qw, double qx, double qy, double qz) {
  Quaterniond q(qw, qx, qy, qz);
  return q.normalized().toRotationMatrix();
}

Matrix3d so32SO3(const Vector3d &v) {
  double theta = v.norm();
  if (theta > kEpsilon) {
    Vector3d vn = v/theta;
    Matrix3d v_wedge = wedge(v);
    Matrix3d SO3;
    SO3 = Matrix3d::Identity() + v_wedge*sin(theta) +
      v_wedge*v_wedge*(1.0 - cos(theta));
    return SO3;
  } else {
    return Matrix3d::Identity();
  }
}

Vector3d SO32so3(const Matrix3d &R) {
  Vector3d so3;
  double temp_arg_to_cos = (R.trace() - 1.0)/2.0;
  truncate(&temp_arg_to_cos, -1.0, 1.0);
  double theta = acos(temp_arg_to_cos);
  if(fabs(theta) < kEpsilon) {
    so3(0) = 1.0;
    so3(1) = 0.0;
    so3(2) = 0.0;
  } else {
    so3(0) = R(2,1)-R(1,2);
    so3(1) = R(0,2)-R(2,0);
    so3(2) = R(1,0)-R(0,1);
    so3 /= 2.0*sin(theta);
  }
  so3 *= theta;
  return so3;
}

void so32quat(const Vector3d &so3, double *q) {
  double theta = so3.norm();
  if (theta < kEpsilon) {
    q[0] = 1;
    q[1] = 0;
    q[2] = 0;
    q[3] = 0;
  } else {
            // q = [cos(theta/2); sin(theta/2)*so3/theta];
    double sin_theta = sin(theta/2.0)/theta;
    q[0] = cos(theta/2.0);
    q[1] = so3(0)*sin_theta;
    q[2] = so3(1)*sin_theta;
    q[3] = so3(2)*sin_theta;
  }
}
void SO32quat(const Matrix3d &SO3, double *q) {
  Quaterniond q_eigen(SO3);
  q_eigen.normalize();
  q[0] = q_eigen.w();
  q[1] = q_eigen.x();
  q[2] = q_eigen.y();
  q[3] = q_eigen.z();
}

Matrix4d pose2SE3(const double *pose) {
  Matrix4d SE3 = Matrix4d::Identity();
  SE3(0, 3) = pose[0];
  SE3(1, 3) = pose[1];
  SE3(2, 3) = pose[2];
  SE3.block<3,3>(0,0) = quat2SO3(pose[3], pose[4], pose[5], pose[6]);
  return SE3;
}

Matrix4d posemm2SE3(const double *pose) {
  Matrix4d SE3 = Matrix4d::Identity();
  SE3(0, 3) = pose[0]/1000.0;
  SE3(1, 3) = pose[1]/1000.0;
  SE3(2, 3) = pose[2]/1000.0;
  SE3.block<3,3>(0,0) = quat2SO3(pose[3], pose[4], pose[5], pose[6]);
  return SE3;
}

Matrix4d se32SE3(const Vector6d &twist) {
  Matrix4d SE3 = Matrix4d::Identity();
  double theta = twist.tail(3).norm();
  if ( theta < kEpsilon ) {
    // no rotation
    SE3(0, 3) = twist(0);
    SE3(1, 3) = twist(1);
    SE3(2, 3) = twist(2);
  } else {
    Vector3d v = twist.head(3);
    Vector3d w = twist.tail(3);
    Matrix3d R = so32SO3(w);
    v /= theta;
    w /= theta;
    SE3.block<3,3>(0, 0) = R;
    SE3.block<3,1>(0, 3) = (Matrix3d::Identity() - R)*(w.cross(v)) +
        w*w.transpose()*v*theta;
  }
  return SE3;
}

Matrix4d spt2SE3(const Vector6d &spt) {
  Matrix4d SE3 = Matrix4d::Identity();
  SE3.block<3, 3>(0, 0) = so32SO3(spt.tail(3));
  SE3.block<3, 1>(0, 3) = spt.head(3);
  return SE3;
}

Matrix4d SE3Inv(const Matrix4d &SE3) {
  Matrix4d SE3_inv = Matrix4d::Identity();
  SE3_inv.block<3,1>(0, 3) =
      -SE3.block<3,3>(0,0).transpose()*SE3.block<3,1>(0,3);
  SE3_inv.block<3,3>(0,0) = SE3.block<3,3>(0,0).transpose();
  return SE3_inv;
}

Vector6d SE32se3(const Matrix4d &SE3) {
  Vector3d p     = SE3.block<3,1>(0, 3);
  Vector3d omega = SO32so3(SE3.block<3,3>(0,0));
  double theta = omega.norm();
  if (theta < kEpsilon) {
    Vector6d se3;
    se3 << p(0), p(1), p(2), 0, 0, 0;
    return se3;
  } else {
    omega /= theta;
    Matrix3d M =
        (Matrix3d::Identity() - wedge(omega*theta).exp())*
        wedge(omega)+omega*omega.transpose()*theta;
    Vector6d se3;
    se3.head(3) = M.fullPivLu().solve(p);
    se3.tail(3) = omega;
    se3 *= theta;
    return se3;
  }
}

Vector6d SE32spt(const Matrix4d &SE3) {
  Vector6d spt;
  spt.head(3) = SE3.block<3, 1>(0, 3);
  spt.tail(3) = SO32so3(SE3.block<3, 3>(0, 0));
  return spt;
}

Matrix6d SE32Adj(const Matrix4d &SE3) {
  Matrix6d Adj = Matrix6d::Zero();
  Adj.topLeftCorner(3, 3)     = SE3.topLeftCorner(3, 3);
  Adj.bottomRightCorner(3, 3) = SE3.topLeftCorner(3, 3);
  Adj.topRightCorner(3, 3)    =
      wedge(SE3.block<3,1>(0, 3)) * SE3.topLeftCorner(3, 3);
  return Adj;
}

void SE32Pose(const Matrix4d &SE3, double *pose) {
  pose[0] = SE3(0, 3);
  pose[1] = SE3(1, 3);
  pose[2] = SE3(2, 3);
  SO32quat(SE3.block<3,3>(0,0), pose + 3);
}

void SE32Posemm(const Matrix4d &SE3, double *pose) {
  pose[0] = SE3(0, 3)*1000.0;
  pose[1] = SE3(1, 3)*1000.0;
  pose[2] = SE3(2, 3)*1000.0;
  SO32quat(SE3.block<3,3>(0,0), pose + 3);
}

// Return the 6x6 jacobian matrix mapping from spt time derivative
//  to body velocity.
// Jac * spt time derivative = body velocity
Matrix6d JacobianSpt2BodyV(const Matrix3d &R) {
  Matrix6d Jac;
  Jac = Matrix6d::Identity();
  Jac(3, 3) = R(0,2)*R(0,2) + R(1,2)*R(1,2) + R(2,2)*R(2,2);
  Jac(3, 5) = -R(0,0)*R(0,2) - R(1,0)*R(1,2) - R(2,0)*R(2,2);
  Jac(4, 3) = -R(0,0)*R(0,1) - R(1,0)*R(1,1) - R(2,0)*R(2,1);
  Jac(4, 4) = R(0,0)*R(0,0) + R(1,0)*R(1,0) + R(2,0)*R(2,0);
  Jac(5, 4) = -R(0,1)*R(0,2) - R(1,1)*R(1,2) - R(2,1)*R(2,2);
  Jac(5, 5) = R(0,1)*R(0,1) + R(1,1)*R(1,1) + R(2,1)*R(2,1);

  return Jac;
}

void double2float(const double *array_in, float *array_out, int n) {
  for (int i = 0; i < n; ++i)
    array_out[i] = array_in[i];
}
void float2double(const float *array_in, double *array_out, int n) {
  for (int i = 0; i < n; ++i)
    array_out[i] = array_in[i];
}

}

#endif // _EGMCLASS_ULTILITIES_H_
