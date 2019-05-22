#include "Point3d.h"
#include "Vector3d.h"


/**
	addition of two vectors - results in a new vector.
*/
Point3d Point3d::operator +(const Vector3d& v) const
{
	return Point3d(this->x + v.x, this->y + v.y, this->z + v.z);
}


/**
	setDataFromPosition this vector to the current point
*/
Point3d& Point3d::operator +=(const Vector3d& v)
{
	this->x += v.x;
	this->y += v.y;
	this->z += v.z;
	return *this;
}


/**
	difference between two points - results in a new vector.
*/
Vector3d Point3d::operator -(const Point3d& p) const
{
	return Vector3d(this->x - p.x, this->y - p.y, this->z - p.z);
}

//*this = p + v * s
void Point3d::set_to_offset_from_point(const Point3d& p, const Vector3d& v, double s)
{
	this->x = p.x + v.x * s;
	this->y = p.y + v.y * s;
	this->z = p.z + v.z * s;
}
