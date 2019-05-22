#include "SphereCDP.h"
#include "CapsuleCDP.h"

SphereCDP::SphereCDP(Point3d& c_, double r_)
{
	s.pos = c_;
	s.radius = r_;
	type = sphere_cdp;
}

SphereCDP::SphereCDP(const SphereCDP& other) : CollisionDetectionPrimitive(other)
{
	s = other.s;
}

SphereCDP::SphereCDP() 
{
	type = sphere_cdp;
}