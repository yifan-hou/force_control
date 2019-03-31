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
    typedef Eigen::Vector3f Vector3f;
    typedef Eigen::Matrix3f Matrix3f;
    typedef Eigen::Matrix4f Matrix4f;
    typedef Eigen::MatrixXf MatrixXf;
    typedef Eigen::Matrix<float, 6, 1> Vector6f;
    typedef Eigen::Matrix<float, 6, 6> Matrix6f;
    typedef Eigen::Quaternionf Quaternionf;

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

	static void truncate(double *ele, const double _max, const double _min)
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

	static void setArray(float *array, float value, int dim)
	{
	    for(int i=0; i<dim; i++)
	    {
	        array[i] = value;
	    }
	}

	static void truncate(float *array, float max, float min, int dim)
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


    static void truncate6f(Vector6f *v, float max, float min)
    {
        for(int i=0; i<6; i++)
        {
            (*v)[i] = ((*v)[i] > max)? max:(*v)[i];
            (*v)[i] = ((*v)[i] < min)? min:(*v)[i];
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

    /////////////////////////////////////////////////////////////////////////
    //                          Matrices
    /////////////////////////////////////////////////////////////////////////
    template<typename _Matrix_Type_>
    _Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, float epsilon = std::numeric_limits<float>::epsilon())
    {
        Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
        double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
        return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
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


    Matrix3f wedge(const Vector3f &v) {
      Matrix3f v_wedge;
      v_wedge << 0, -v(2), v(1),
                v(2), 0, -v(0),
                -v(1), v(0), 0;
      return v_wedge;
    }

    Matrix4f wedge6(const Vector6f &t) {
      Matrix4f t_wedge;
      t_wedge <<   0,   -t(5),   t(4),  t(0),
                   t(5),     0,   -t(3),  t(1),
                   -t(4),   t(3),     0,   t(2),
                      0,     0,     0,     0;
      return t_wedge;
    }

    Matrix3f quat2SO3(const Quaternionf &q) {
      float q11 = q.x()*q.x();
      float q22 = q.y()*q.y();
      float q33 = q.z()*q.z();
      float q01 = q.w()*q.x();
      float q02 = q.w()*q.y();
      float q03 = q.w()*q.z();
      float q12 = q.x()*q.y();
      float q13 = q.x()*q.z();
      float q23 = q.y()*q.z();

      Matrix3f m;
      m << 1.0f - 2.0f*q22 - 2.0f*q33,  2.0f*(q12 - q03),      2.0f*(q13 + q02),
           2.0f*(q12 + q03),     1.0f - 2.0f*q11 - 2.0f*q33,  2.0f*(q23 - q01),
           2.0f*(q13 - q02),     2.0f*(q23 + q01),      1.0f - 2.0f*q11 - 2.0f*q22;
      return m;
    }

    Matrix3f quat2SO3(float qw, float qx, float qy, float qz) {
      float q11 = qx*qx;
      float q22 = qy*qy;
      float q33 = qz*qz;
      float q01 = qw*qx;
      float q02 = qw*qy;
      float q03 = qw*qz;
      float q12 = qx*qy;
      float q13 = qx*qz;
      float q23 = qy*qz;

      Matrix3f m;
      m << 1.0f - 2.0f*q22 - 2.0f*q33, 2.0f*(q12 - q03),      2.0f*(q13 + q02),
          2.0f*(q12 + q03),     1.0f - 2.0f*q11 - 2.0f*q33,  2.0f*(q23 - q01),
          2.0f*(q13 - q02),     2.0f*(q23 + q01),      1.0f - 2.0f*q11 - 2.0f*q22;
      return m;
    }

    Matrix3f so32SO3(const Vector3f &v) {
        return wedge(v).exp();
    }

    Vector3f SO32so3(const Matrix3f &R) {
        Vector3f so3;
        float theta = std::acos((R.trace()-1.0f)/2.0f);
        if(fabs(theta) < 1e-5) {
            so3(0) = 1.0f;
            so3(1) = 0.0f;
            so3(2) = 0.0f;
        } else {
            so3(0) = R(2,1)-R(1,2);
            so3(1) = R(0,2)-R(2,0);
            so3(2) = R(1,0)-R(0,1);
            so3 /= 2.0f*sin(theta);
        }
        so3 *= theta;
        return so3;
    }

    void so32quat(const Vector3f &so3, float *q) {
        float theta = so3.norm();
        if (theta < 1e-5) {
            q[0] = 1;
            q[1] = 0;
            q[2] = 0;
            q[3] = 0;
        } else {
            // q = [cos(theta/2); sin(theta/2)*so3/theta];
            float sin_theta = std::sin(theta/2.0f)/theta;
            q[0] = std::cos(theta/2.0f);
            q[1] = so3(0)*sin_theta;
            q[2] = so3(1)*sin_theta;
            q[3] = so3(2)*sin_theta;
        }
    }
    void SO32quat(const Matrix3f &SO3, float *q) {
        so32quat(SO32so3(SO3), q);
    }

    Matrix4f pose2SE3(const float *pose) {
        Matrix4f SE3 = Matrix4f::Identity();
        SE3(0, 3) = pose[0];
        SE3(1, 3) = pose[1];
        SE3(2, 3) = pose[2];
        SE3.block<3,3>(0,0) = quat2SO3(pose[3], pose[4], pose[5], pose[6]);
        return SE3;
    }

    Matrix4f se32SE3(const Vector6f &twist) {
        Matrix4f SE3 = Matrix4f::Identity();
        float theta = twist.tail(3).norm();
        if ( theta < 1e-6 ) {
            // no rotation
            SE3(0, 3) = twist(0);
            SE3(1, 3) = twist(1);
            SE3(2, 3) = twist(2);
        } else {
            Vector3f v = twist.head(3);
            Vector3f w = twist.tail(3);
            Matrix3f R = so32SO3(w);
            v /= theta;
            w /= theta;
            SE3.block<3,3>(0, 0) = R;
            SE3.block<3,1>(0, 3) = (Matrix3f::Identity() - R)*(w.cross(v)) +
                    w*w.transpose()*v*theta;
        }
        return SE3;
    }

    Matrix4f spt2SE3(const Vector6f &spt) {
        Matrix4f SE3 = Matrix4f::Identity();
        SE3.block<3, 3>(0, 0) = so32SO3(spt.tail(3));
        SE3.block<3, 1>(0, 3) = spt.head(3);
        return SE3;
    }

    Matrix4f SE3Inv(const Matrix4f &SE3) {
        Matrix4f SE3_inv = Matrix4f::Identity();
        SE3_inv.block<3,1>(0, 3) =
                -SE3.block<3,3>(0,0).transpose()*SE3.block<3,1>(0,3);
        SE3_inv.block<3,3>(0,0) = SE3.block<3,3>(0,0).transpose();
        return SE3_inv;
    }

    Vector6f SE32se3(const Matrix4f &SE3) {
        Vector3f p     = SE3.block<3,1>(0, 3);
        Vector3f omega = SO32so3(SE3.block<3,3>(0,0));
        float theta = omega.norm();
        if (theta < 1e-5) {
            Vector6f se3;
            se3 << p(0), p(1), p(2), 0, 0, 0;
            return se3;
        } else {
            omega /= theta;
            Matrix3f M =
                    (Matrix3f::Identity() - wedge(omega*theta).exp())*
                    wedge(omega)+omega*omega.transpose()*theta;
            Vector6f se3;
            se3.head(3) = M.fullPivLu().solve(p);
            se3.tail(3) = omega;
            se3 *= theta;
            return se3;
        }
    }

    Vector6f SE32spt(const Matrix4f &SE3) {
        Vector6f spt;
        spt.head(3) = SE3.block<3, 1>(0, 3);
        spt.tail(3) = SO32so3(SE3.block<3,3>(0,0));
        return spt;
    }

    Matrix6f SE32Adj(const Matrix4f &SE3) {
        Matrix6f Adj = Matrix6f::Zero();
        Adj.topLeftCorner(3, 3) = SE3.topLeftCorner(3, 3);
        Adj.bottomRightCorner(3, 3) = SE3.topLeftCorner(3, 3);
        Adj.topRightCorner(3, 3) =
                wedge(SE3.block<3,1>(0, 3)) * SE3.topLeftCorner(3, 3);
        return Adj;
    }

    void SE32Pose(const Matrix4f &SE3, float *pose) {
        pose[0] = SE3(0, 3);
        pose[1] = SE3(1, 3);
        pose[2] = SE3(2, 3);
        SO32quat(SE3.block<3,3>(0,0), pose + 3);
    }

    // Return the 6x6 jacobian matrix mapping from spt time derivative
    //  to body velocity.
    // Jac * spt time derivative = body velocity
    Matrix6f JacobianSpt2BodyV(const Matrix3f &R) {
        Matrix6f Jac;
        Jac = Matrix6f::Identity();
        Jac(3, 3) = R(0,2)*R(0,2) + R(1,2)*R(1,2) + R(2,2)*R(2,2);
        Jac(3, 5) = -R(0,0)*R(0,2) - R(1,0)*R(1,2) - R(2,0)*R(2,2);
        Jac(4, 3) = -R(0,0)*R(0,1) - R(1,0)*R(1,1) - R(2,0)*R(2,1);
        Jac(4, 4) = R(0,0)*R(0,0) + R(1,0)*R(1,0) + R(2,0)*R(2,0);
        Jac(5, 4) = -R(0,1)*R(0,2) - R(1,1)*R(1,2) - R(2,1)*R(2,2);
        Jac(5, 5) = R(0,1)*R(0,1) + R(1,1)*R(1,1) + R(2,1)*R(2,1);
    }


}

#endif // _EGMCLASS_ULTILITIES_H_
