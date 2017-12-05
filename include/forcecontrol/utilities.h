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
 
// #include <stdarg.h>

namespace UT
{
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



}
	
#endif // _EGMCLASS_ULTILITIES_H_
