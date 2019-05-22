#pragma once
#include "MathLib/src/Point3d.h"
#include "MathLib/src/Vector3d.h"
#include "MathLib/src/Plane.h"
#include "CollisionDetectionPrimitive.h"


/*========================================================================================================================================================================*
 * This class implements a plane class that will be used as a collision detection primitive.                                                                              *
 * A plane is represented by the position of one point on the plane, as well as the normal (unit vector) to the plane. We will store these two quantities both in local   *
 * coordinates, and in world coordinates which will be used for the collision detection. NOTE: we will not be evaluating the collision between different planes because   *
 * I will assume that they are all part of fixed objects only.
 *========================================================================================================================================================================*/
class Plane_cdp : public CollisionDetectionPrimitive
{
public:
	
	Plane_cdp(const Vector3d& n, const Point3d& o);
	Plane_cdp(const Plane_cdp& other);
	Plane_cdp();

	~Plane_cdp() = default;

	Plane_cdp(Plane_cdp&& other) = delete;
	Plane_cdp& operator=(const Plane_cdp& other) = delete;
	Plane_cdp& operator=(Plane_cdp&& other) = delete;

	Vector3d get_normal() const { return m_p.n; }
	Point3d get_point_on_plane() const { return m_p.p; };

private:
	//this is the plane, expressed in the local coordinates of the rigid body that owns it
	Plane m_p;
};
