#include "PlaneCDP.h"
#include "SphereCDP.h"

Plane_cdp::Plane_cdp(const Vector3d& n, const Point3d& o)
{
	//make sure we have a unit vector;
	m_p.n = n;
	m_p.n.toUnit();
	m_p.p = o;

	type = plane_cdp;

	m_model = std::make_shared<Model>();
	m_model->set_position(o);
	m_model->load_from_file("../../data/models/plane.obj");
}
 
Plane_cdp::Plane_cdp(const Plane_cdp& other) : CollisionDetectionPrimitive(other)
{
	m_p = other.m_p;
}

Plane_cdp::Plane_cdp()
{
	type = plane_cdp;
}

