#include "StiffJoint.h"

/**
	This method is used to fix the joint angular constraint to correct for drift. This is done by changing
	the orientation of the child.
*/
void StiffJoint::fix_angular_constraint_parent_to_child(const Quaternion& qRel)
{
	//not much to do here...
}

void StiffJoint::read_axes(char* axes)
{
}

/**
This method is used to pass information regarding the joint limits for a joint. The string that is passed in is expected to
have been read from an input file.
*/
void StiffJoint::read_joint_limits(char* limits)
{
}

Vector3d StiffJoint::get_angles() const
{
	return Vector3d(0, 0, 0);
}

void StiffJoint::fix_angular_constraint_child_to_parent(const Quaternion& qRel)
{
}
