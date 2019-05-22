#pragma once

#include "MathLib/src/mathLib.h"
#include <cstdio>
#include <stdexcept>


class ThreeTuple
{
protected:

public:
	//some useful constructors 
	ThreeTuple();
	ThreeTuple(ThreeTuple&);
	ThreeTuple(double x, double y, double z);
	ThreeTuple(double x, double y);
	ThreeTuple(double*);
	~ThreeTuple() = default;

	void setValues(double a_x, double a_y, double a_z)
	{
		x = a_x;
		y = a_y;
		z = a_z;
	}

	void setValues(ThreeTuple& a_p)
	{
		x = a_p.x;
		y = a_p.y;
		z = a_p.z;
	}

	//this is an equality operator.
	bool operator ==(const ThreeTuple& a_p) const
	{
		double dx = x - a_p.x;
		double dy = y - a_p.y;
		double dz = z - a_p.z;
		return (ZERO_WITHIN_EPSILON(dx) && ZERO_WITHIN_EPSILON(dy) && ZERO_WITHIN_EPSILON(dz));
	}

	//this is an equality operator.
	bool operator !=(const ThreeTuple& a_p) const
	{
		return !(*this == a_p);
	}

	//This method is used for debugging purposes. It prints the x, y and z components.
	void print()
	{
		printf("(%lf, %lf, %lf)\n", x, y, z);
	}


	//and a copy operator.
	ThreeTuple& operator =(ThreeTuple& a_p) = default;

public:
	double x;
	double y;
	double z;
};
