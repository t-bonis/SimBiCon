#include "BoxCDP.h"

BoxCDP::BoxCDP(Point3d& p1, Point3d& p2)
{
	m_p1 = p1;
	m_p2 = p2;
	type = box_cdp;
	m_model = std::make_shared<Model>();
	m_model->add_box(Vector3d(p1.x, p1.y, p1.z), Vector3d(p2.x, p2.y, p2.z));
}

BoxCDP::BoxCDP()
{
	type = box_cdp;
}


BoxCDP::BoxCDP(const BoxCDP& other) : CollisionDetectionPrimitive(other)
{
	m_p1 = other.m_p1;
	m_p2 = other.m_p2;
}