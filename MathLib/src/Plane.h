#pragma once

#include "MathLib/src/Vector3d.h"
#include "MathLib/src/Point3d.h"

class Plane
{
public:
	//a plane is defined by its normal, and a point on it
	Vector3d n;
	Point3d p;
public:
	Plane() = default;
	Plane(const Point3d& p, const Vector3d& n);
	~Plane() = default;
};
