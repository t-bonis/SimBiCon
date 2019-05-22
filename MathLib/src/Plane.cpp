#include "Plane.h"

Plane::Plane(const Point3d& p, const Vector3d& n)
{
	this->n = n;
	//assume the normals are unit vectors.
	//	this->n.toUnit();
	this->p = p;
}
