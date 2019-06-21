#include "BallInSocketJoint.h"

#define ANGLE_A_CONSTRAINT		1
#define ANGLE_B_CONSTRAINT		2


BallInSocketJoint::BallInSocketJoint(char* axes)
{
	read_axes(axes); 
}

BallInSocketJoint::BallInSocketJoint(const BallInSocketJoint& other) : Joint(other)
{
	swingAxis1 = other.swingAxis1;
	swingAxis2 = other.swingAxis2;

	minSwingAngle1 = other.minSwingAngle1;
	maxSwingAngle1 = other.maxSwingAngle1;
	minSwingAngle2 = other.minSwingAngle2;
	maxSwingAngle2 = other.maxSwingAngle2;
	minTwistAngle = other.minTwistAngle;
	maxTwistAngle = other.maxTwistAngle;
}

//	This method is used to fix the joint angular constraint to correct for drift. This is done by changing
//	the arb_orientation of the child.
void BallInSocketJoint::fix_angular_constraint_parent_to_child(const Quaternion& qRel)
{
}

//	This method is used to pass in information regarding the rotation axes. The string that is passed in is expected to have
//	been read from an input file.
void BallInSocketJoint::read_axes(char* axes)
{
	if (sscanf_s(axes, "%lf %lf %lf %lf %lf %lf %lf %lf %lf", &swingAxis1.x, &swingAxis1.y, &swingAxis1.z,
	             &swingAxis2.x, &swingAxis2.y, &swingAxis2.z, &twistAxis.x, &twistAxis.y, &twistAxis.z) != 9)
	{
		if (sscanf_s(axes, "%lf %lf %lf %lf %lf %lf", &swingAxis1.x, &swingAxis1.y, &swingAxis1.z, &twistAxis.x,
		             &twistAxis.y, &twistAxis.z) != 6)
		{
			throw std::logic_error("Ball in socket joints require two or three axis to be specified!");
		}
		swingAxis2 = swingAxis1.cross_product_with(twistAxis);
	}
	swingAxis1.toUnit();
	swingAxis2.toUnit();
	twistAxis.toUnit();
}

Vector3d BallInSocketJoint::get_angles() const
{
	//these two axes define the plane of vectors along which the rotations represent a swing - stored in parent coordinates
	//Vector3d swingAxis1, swingAxis2;
	//and this one is stored in child coordinates - this is the twist axis
	//Vector3d twistAxis;
	Vector3d n = swingAxis1.unit(); //normal to the first plane
	Point3d p = m_joint_pos_in_parent; //joint in parent coords
	//child and parent COMs in parent coords
	Point3d parentCOM(0, 0, 0);
	Point3d childCOM = m_parent_arb->get_local_coordinates(m_child_arb->get_cm_position());
	//projections onto the plane
	Point3d pparent = parentCOM - n * (parentCOM - p).dotProductWith(n);
	Point3d pchild = childCOM - n * (childCOM - p).dotProductWith(n);
	//unit vectors
	Vector3d joint_to_parent = (pparent - p).toUnit();
	Vector3d joint_to_child = (pchild - p).toUnit();
	//oriented clockwise angle between the vectors (parent and child joint)
	double alpha = acos(joint_to_parent.dotProductWith(joint_to_child));
	Vector3d cross = joint_to_parent.cross_product_with(joint_to_child);
	if (n.dotProductWith(cross) < 0)
	{
		alpha = 2 * M_PI - alpha;
	}

	n = swingAxis2.unit(); //normal to the second plane
	//projections onto the plane
	pparent = parentCOM - n * (parentCOM - p).dotProductWith(n);
	pchild = childCOM - n * (childCOM - p).dotProductWith(n);
	//unit vectors
	joint_to_parent = (pparent - p).toUnit();
	joint_to_child = (pchild - p).toUnit();
	//oriented clockwise angle between the vectors (parent and child joint)
	double beta = acos(joint_to_parent.dotProductWith(joint_to_child));
	cross = joint_to_parent.cross_product_with(joint_to_child);
	if (n.dotProductWith(cross) < 0)
	{
		beta = 2 * M_PI - beta;
	}

	n = m_parent_arb->get_local_coordinates(m_child_arb->get_vector_world_coordinates(twistAxis)).toUnit();
	//normal to the third plane
	//projections onto the plane
	pparent = parentCOM - n * (parentCOM - p).dotProductWith(n);
	pchild = childCOM - n * (childCOM - p).dotProductWith(n);
	//unit vectors
	joint_to_parent = (pparent - p).toUnit();
	joint_to_child = (pchild - p).toUnit();
	//oriented clockwise angle between the vectors (parent and child joint)
	double gamma = acos(joint_to_parent.dotProductWith(joint_to_child));
	cross = joint_to_parent.cross_product_with(joint_to_child);
	if (n.dotProductWith(cross) < 0)
	{
		gamma = 2 * M_PI - gamma;
	}
	return Vector3d(alpha, beta, gamma);
}

void BallInSocketJoint::fix_angular_constraint_child_to_parent(const Quaternion& qRel)
{
}

//	This method is used to pass information regarding the joint limits for a joint. The string that is passed in is expected to
//	have been read from an input file.
void BallInSocketJoint::read_joint_limits(char* limits)
{
	int n = sscanf_s(limits, "%lf %lf %lf %lf %lf %lf", &minSwingAngle1, &maxSwingAngle1, &minSwingAngle2,
	                 &maxSwingAngle2, &minTwistAngle, &maxTwistAngle);
	if (n != 6)
		throw std::logic_error(
			"Ball in socket joints require 6 joint limit parameters (min/max angle for the three axis)!");

	m_use_joint_limits = true;
}
