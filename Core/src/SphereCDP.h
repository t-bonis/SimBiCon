#pragma once

#include "CollisionDetectionPrimitive.h"
#include "MathLib/src/Point3d.h"
#include "MathLib/src/Sphere.h"

/*========================================================================================================================================================================*
 * This class implements a sphere class that will be used as a collision detection primitive.                                                                             *
 * A sphere is represented by the position of the center and the radius length. We will also store a temp position for the world coordinates of the center of the sphere. *
 * This will be used when evaluating the contact points with other primitives, and it will be automatically 
 *========================================================================================================================================================================*/
class SphereCDP : public CollisionDetectionPrimitive
{
public:
	SphereCDP(Point3d& c_, double r_);
	SphereCDP(const SphereCDP& other);
	SphereCDP();

	virtual ~SphereCDP() = default;
	
	SphereCDP(SphereCDP&& other) = delete;
	SphereCDP& operator=(const SphereCDP& other) = delete;
	SphereCDP& operator=(SphereCDP&& other) = delete;

	//return the radius of the sphere
	double get_radius() const
	{
		return s.radius;
	}

	//return the center of the sphere, expressed in local coordinates
	Point3d get_center() const
	{
		return s.pos;
	}

private:
	//keep track of the local-coordinates sphere used by this collision detection primitive
	Sphere s;
};
