#pragma once
#include "Joint.h"

/*======================================================================================================================================================================*
 * This class is used to implement a stiff joint - always computes the impulses that are needed to have zero relative angular velocity between the parent and the child *
 *======================================================================================================================================================================*/

class StiffJoint : public Joint
{
	friend class Ode_world;
	friend class Articulated_rigid_body; //muscles
public:
	StiffJoint() = default;
	~StiffJoint() = default;

	/**
		This method is used to fix the joint angular constraint to correct for drift. This is done by changing
		the orientation of the child.
	*/
	void fix_angular_constraint_parent_to_child(const Quaternion& qRel) override;

	void read_axes(char* axes) override;

	void read_joint_limits(char* limits) override;

	//Returns the type of the current joint
	type get_type() const override { return stiff; }

	Vector3d get_angles() const override;

	std::vector<Vector3d> get_rotation_axes_in_local_coords() const override
	{
		return std::vector<Vector3d>();
	}

	std::vector<Vector3d> get_rotation_axes_in_global_coords() const override
	{
		return std::vector<Vector3d>();
	}

	std::vector<std::vector<double>> get_rotation_limits() override
	{
		return std::vector<std::vector<double>>();
	}

	void fix_angular_constraint_child_to_parent(const Quaternion& qRel) override;
};
