#include "CapsuleCDP.h"
#include "SphereCDP.h"

Capsule_cdp::Capsule_cdp(Point3d& a, Point3d& b, const double r)
{
	this->m_c.p1 = a;
	this->m_c.p2 = b;
	this->m_c.radius = r;
	type = capsule_cdp;
}

Capsule_cdp::Capsule_cdp(const Capsule_cdp& other) : CollisionDetectionPrimitive(other)
{
	m_c = other.m_c;
}

Capsule_cdp::Capsule_cdp()
{
	type = capsule_cdp;
}

