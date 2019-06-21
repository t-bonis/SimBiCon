#include "HingeJoint.h"

//----------------------------------------------------
// Destructor and constructor
//----------------------------------------------------

HingeJoint::HingeJoint(char* axes)
{
	read_axes(axes);
	minAngle = 0;
	maxAngle = 0;
}

HingeJoint::HingeJoint(const HingeJoint& other) : Joint(other)
{
	rot_axe = other.rot_axe;
	minAngle = other.minAngle;
	maxAngle = other.maxAngle;
}

//-----------------------------------------------------
// Construction
//-----------------------------------------------------

/**
	This method is used to pass in information regarding the rotation axes. The string that is passed in is expected to have
	been read from an input file.
*/
void HingeJoint::read_axes(char* axes)
{
	if (sscanf_s(axes, "%lf %lf %lf", &rot_axe.x, &rot_axe.y, &rot_axe.z) != 3)
		throw std::logic_error("Hinge joints require the rotation axis to be provided as a parameter!");

	rot_axe.toUnit();
}

/**
	This method is used to pass information regarding the joint limits for a joint. The string that is passed in is expected to
	have been read from an input file.
*/
void HingeJoint::read_joint_limits(char* limits)
{
	if (sscanf_s(limits, "%lf %lf", &minAngle, &maxAngle) != 2)
		throw std::logic_error("Two parameters are needed to specify joint limits for a hinge joint!");
	m_use_joint_limits = true;
}

//--------------------------------------
// set and get
//--------------------------------------

Vector3d HingeJoint::get_angles() const
{
	//This joint only allows relative motion about axis a - stored in parent coordinates
	Vector3d n = rot_axe.unit(); //normal to the plane
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
	double angle = acos(jointParent.dotProductWith(jointChild));
	Vector3d cross = jointParent.cross_product_with(jointChild);
	if (n.dotProductWith(cross) < 0)
	{
		angle = 2 * M_PI - angle;
	}
	return Vector3d(angle, 0, 0);
}

std::vector<std::vector<double>> HingeJoint::get_rotation_limits()
{
	return std::vector<std::vector<double>>({{minAngle, maxAngle}});
}


//----------------------------------------
// Calculation
//----------------------------------------

/**
	This method is used to fix the joint angular constraint to correct for drift. This is done by changing
		the arb_orientation of the child.
*/
void HingeJoint::fix_angular_constraint_parent_to_child(const Quaternion& qRel)
{
	//make sure that the relative rotation between the child and the parent is around the a axis
	Vector3d axis = qRel.getV().toUnit();

	//this is the rotation angle around the axis above, which may not be the rotation axis
	double rotAngle = 2 * safeACOS(qRel.getS());

	//get the rotation angle around the correct axis now (we are not in the world frame now)
	double ang = axis.dotProductWith(rot_axe) * rotAngle;

	//and compute the correct child arb_orientation
	m_child_arb->set_orientation(m_parent_arb->get_orientation() * Quaternion::get_rotation_quaternion(ang, rot_axe));
}

void HingeJoint::fix_angular_constraint_child_to_parent(const Quaternion& qRel)
{
	//make sure that the relative rotation between the parent and the child is around the a axis
	Vector3d axis = qRel.getV().toUnit();

	//this is the rotation angle around the axis above, which may not be the rotation axis
	double rotAngle = 2 * safeACOS(qRel.getS());

	//get the rotation angle around the correct axis now (we are not in the world frame now)
	double ang = axis.dotProductWith(rot_axe) * rotAngle;

	//and compute the correct parent arb_orientation
	m_parent_arb->set_orientation(m_child_arb->get_orientation() * Quaternion::get_rotation_quaternion(ang, rot_axe));
}
