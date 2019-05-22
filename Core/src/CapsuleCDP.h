#pragma once

#include "CollisionDetectionPrimitive.h"
#include "MathLib/src/Point3d.h"
#include "MathLib/src/Capsule.h"

/*========================================================================================================================================================================*
 * This class implements a capsule class that will be used as a collision detection primitive.                                                                            *
 * A capsule is represented by the position of the two end points and the radius length. We will also store a temp position for the world coordinates of the endppoints   * 
 * of the capsule. This will be used when evaluating the contact points with other primitives, and it needs to be updated any time the world position of the object that  *                      
 * owns this capsule changes.                                                                                                                                             *
 *========================================================================================================================================================================*/
class Capsule_cdp : public CollisionDetectionPrimitive
{
public:
	
	Capsule_cdp(Point3d& a, Point3d& b, double r);
	Capsule_cdp();
	Capsule_cdp(const Capsule_cdp& other);

	~Capsule_cdp() = default;

	Capsule_cdp(Capsule_cdp&& other) = delete;
	Capsule_cdp& operator=(const Capsule_cdp& other) = delete;
	Capsule_cdp& operator=(Capsule_cdp&& other) = delete;
	   
	//return the radius of the sphere
	double get_radius() const
	{
		return this->m_c.radius;
	}

	//return the position of the first endpoint.
	Point3d get_a() const
	{
		return m_c.p1;
	}

	//return the position of the first endpoint.
	Point3d get_b() const
	{
		return m_c.p2;
	}

private:
	//a capsule is really just an infinite number of spheres that have the center along a segment. Therefore, to define the capsule we need the
	//two end points and the radius
	Capsule m_c;

};
