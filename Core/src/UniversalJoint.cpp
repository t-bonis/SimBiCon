#include "UniversalJoint.h"

#define ANGLE_A_CONSTRAINT		1
#define ANGLE_B_CONSTRAINT		2

UniversalJoint::UniversalJoint(const UniversalJoint& other) : Joint(other)
{
	rot_axe1 = other.rot_axe1;
	rot_axe2 = other.rot_axe2;

	minAngleA = other.minAngleA;
	maxAngleA = other.maxAngleA;

	minAngleB = other.minAngleB;
	maxAngleB = other.maxAngleB;
}

void UniversalJoint::read_axes(char* axes)
{
	if (sscanf_s(axes, "%lf %lf %lf %lf %lf %lf", &rot_axe1.x, &rot_axe1.y, &rot_axe1.z, &rot_axe2.x, &rot_axe2.y,
	             &rot_axe2.z) != 6)
		throw std::logic_error("Universal joints require two rotation axes to be provided as parameters!");

	rot_axe1.toUnit();
	rot_axe2.toUnit();
}

void UniversalJoint::read_joint_limits(char* limits)
{
	if (sscanf_s(limits, "%lf %lf %lf %lf", &minAngleA, &maxAngleA, &minAngleB, &maxAngleB) != 4)
		throw std::logic_error(
			"Universal joints require 4 joint limites (minAngleA, maxAngleA, minAngleB, maxAngleB)!");
	m_use_joint_limits = true;
}

void UniversalJoint::fix_angular_constraint_parent_to_child(const Quaternion& qRel)
{
	//to go from the child's coord frame to its parent, first rotate around the axis b, then around the axis a.
	Quaternion tmpQ1, tmpQ2;
	//compute two rotations, such that qRel = tmpQ1 * tmpQ2, and tmpQ2 is a rotation about the vector b (expressed in child coordinates)
	qRel.decomposeRotation(&tmpQ1, &tmpQ2, rot_axe2);

	//now make sure that tmpQ1 represents a rotation about axis a (expressed in parent coordinates)
	double angA = tmpQ1.get_rotation_angle(rot_axe1);
	Vector3d tmpV1 = tmpQ1.v;
	tmpV1.toUnit();
	double mod = tmpV1.dotProductWith(rot_axe1);
	if (mod < 0) mod = -mod;
	angA *= mod;
	m_child_arb->set_orientation(
		m_parent_arb->get_orientation() * Quaternion::get_rotation_quaternion(angA, rot_axe1) * tmpQ2);
}

Vector3d UniversalJoint::get_angles() const
{
	//This joint can only rotate about the vector a, that is stored in parent coordinates
	//or about vector b that is stored in child coordinates
	Vector3d n = rot_axe1.unit(); //normal to the first plan
	Point3d p = m_joint_pos_in_parent; //joint in parent coords
	//child and parent COMs in parent coords
	Point3d parentCOM(0, 0, 0);
	Point3d childCOM = m_parent_arb->get_local_coordinates(m_child_arb->get_cm_position());
	//projections onto the plane
	Point3d pparent = parentCOM - n * (parentCOM - p).dotProductWith(n);
	Point3d pchild = childCOM - n * (childCOM - p).dotProductWith(n);
	//unit vectors
	Vector3d jointParent = (pparent - p).toUnit();
	Vector3d jointChild = (pchild - p).toUnit();
	//oriented clockwise angle between the vectors (parent and child joint)
	double alpha = acos(jointParent.dotProductWith(jointChild));
	Vector3d cross = jointParent.cross_product_with(jointChild);
	if (n.dotProductWith(cross) < 0)
	{
		alpha = 2 * M_PI - alpha;
	}

	//normal to the second plan
	n = m_parent_arb->get_local_coordinates(m_child_arb->get_vector_world_coordinates(rot_axe2)).toUnit();
	//projections onto the plane
	pparent = parentCOM - n * (parentCOM - p).dotProductWith(n);
	pchild = childCOM - n * (childCOM - p).dotProductWith(n);
	//unit vectors
	jointParent = (pparent - p).toUnit();
	jointChild = (pchild - p).toUnit();
	//oriented clockwise angle between the vectors (parent and child joint)
	double beta = acos(jointParent.dotProductWith(jointChild));
	cross = jointParent.cross_product_with(jointChild);
	if (n.dotProductWith(cross) < 0)
	{
		beta = 2 * M_PI - beta;
	}

	return Vector3d(alpha, beta, 0);
}

void UniversalJoint::fix_angular_constraint_child_to_parent(const Quaternion& qRel)
{
}
