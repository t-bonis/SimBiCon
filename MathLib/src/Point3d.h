#pragma once
#include "MathLib/src/ThreeTuple.h"

class Vector3d;

/*=====================================================================================================================================================*
 | This class implements a Point in 3d. It will be stored in homogenous coordinates (x, y, z, w). Every time a w-component is set, the x, y, z will be |
 | rescaled so that w results in being 1 - in other words, the w component is always 1. The x, y and z components are inherited from the Three Tuple   |
 | class that this class extends.                                                                                                                      |
 *=====================================================================================================================================================*/

class Point3d : public ThreeTuple
{
public:


	//	some useful constructors.
	Point3d() = default;

	Point3d(double x, double y, double z) : ThreeTuple(x, y, z)
	{
	}

	Point3d(double x, double y, double z, double w) : ThreeTuple(x, y, z)
	{
		set_w(w);
	}

	Point3d(double x, double y) : ThreeTuple(x, y)
	{
	}

	Point3d(ThreeTuple& p) : ThreeTuple(p)
	{
	}

	Point3d(const Point3d& other) : ThreeTuple(other.x, other.y, other.z)
	{
		this->x = other.x;
		this->y = other.y;
		this->z = other.z;
	}

	Point3d& operator =(const Point3d& other)
	{
		this->x = other.x;
		this->y = other.y;
		this->z = other.z;
		return *this;
	}

	~Point3d() = default;

	//this method is used to set the w component.
	void set_w(double w)
	{
		if (w == 0)
			throw std::logic_error("Cannot set w-component of a point to 0.");
		this->x /= w;
		this->y /= w;
		this->z /= w;
	}

	//*this = p + v * s
	void set_to_offset_from_point(const Point3d& p, const Vector3d& v, double s);

	//addition of a point and a vector - results in a point
	Point3d operator +(const Vector3d& v) const;

	//	setDataFromPosition this vector to the current point
	Point3d& operator +=(const Vector3d& v);

	Point3d& operator /=(double val)
	{
		double v = 1 / val;
		x *= v;
		y *= v;
		z *= v;
		return *this;
	}

	//difference betewwn two points - results in a vector
	Vector3d operator -(const Point3d& p) const;

	//Returns a vector that has all its components multiplied by -1.
	Point3d operator -() const
	{
		return Point3d(-this->x, -this->y, -this->z);
	}
};
