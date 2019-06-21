#pragma once
#include "Joint.h"

/*======================================================================================================================================================================*
 * This class is used to implement a universal joint - angular impulses that allow only two degrees of freedom between the parent and the child must be computed.       *
 *======================================================================================================================================================================*/
class UniversalJoint : public Joint
{
public:
	UniversalJoint() = default;
	UniversalJoint(const UniversalJoint& other);
	
	~UniversalJoint() = default;

	UniversalJoint(UniversalJoint&& other) = delete;
	UniversalJoint& operator=(const UniversalJoint& other) = delete;
	UniversalJoint& operator=(UniversalJoint&& other) = delete;

	//This method is used to pass in information regarding the rotation axes. The string that is passed in is expected to have
	//been read from an input file.
	void read_axes(char* axes) override;

	
	//	This method is used to pass information regarding the joint limits for a joint. The string that is passed in is expected to
	//	have been read from an input file.
	void read_joint_limits(char* limits) override;

	//	This method is used to fix the joint angular constraint to correct for drift. This is done by changing
	//	the arb_orientation of the child.
	void fix_angular_constraint_parent_to_child(const Quaternion& qRel) override;

	//	Return the A rotation axis
	Vector3d getRotAxisA() { return rot_axe1; }

	//	Return the B rotation axis
	Vector3d getRotAxisB() { return rot_axe2; }

	std::vector<Vector3d> get_rotation_axes_in_local_coords() const override
	{
		return std::vector<Vector3d>({rot_axe1, rot_axe2});
	}

	std::vector<std::vector<double>> get_rotation_limits() override
	{
		return std::vector<std::vector<double>>({{minAngleA, maxAngleA}, {minAngleB, maxAngleB}});
	}

	//Returns the type of the current joint
	type get_type() const override { return universal; }

	void read_axis_from_struct(Joint_from_xml joint_input)
	{
	};

	// [muscles]
	Vector3d get_angles() const override;


	std::vector<Vector3d> get_rotation_axes_in_global_coords() const override
	{
		return std::vector<Vector3d>();
	}


	void fix_angular_constraint_child_to_parent(const Quaternion& qRel) override;
private:
	//This joint can only rotate about the vector a, that is stored in parent coordinates
	Vector3d rot_axe1;
	//or about vector b that is stored in child coordinates
	Vector3d rot_axe2;
	//and the min and max allowed angles (around a axis)
	double minAngleA{}, maxAngleA{};
	//and around the b axis
	double minAngleB{}, maxAngleB{};
};
