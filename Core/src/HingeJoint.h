#pragma once
#include "Joint.h"


/*======================================================================================================================================================================*
 * This class is used to implement a hinge joint - angular impulses that allow relative rotation between the parent and the child only around a given axis must be      *
 * computed.                                                                                                                                                            *
 *======================================================================================================================================================================*/
class HingeJoint : public Joint
{
public:
	HingeJoint(char* axes);
	HingeJoint() = default;
	HingeJoint(const HingeJoint& other);

	~HingeJoint() = default;

	HingeJoint(HingeJoint&& other) = delete;
	HingeJoint& operator=(const HingeJoint& other) = delete;
	HingeJoint& operator=(HingeJoint&& other) = delete;


	void read_axes(char* axes) override;
	void read_joint_limits(char* limits) override;

	Vector3d get_rot_axis_a() const { return rot_axe; }
	type get_type() const override { return hinge; }
	std::vector<std::vector<double>> get_rotation_limits() override;

	std::vector<Vector3d> get_rotation_axes_in_local_coords() const override
	{
		return std::vector<Vector3d>(1, rot_axe);
	}

	void fix_angular_constraint_parent_to_child(const Quaternion& qRel) override;
	void fix_angular_constraint_child_to_parent(const Quaternion& qRel) override;

	std::vector<Vector3d> get_rotation_axes_in_global_coords() const override
	{
		return std::vector<Vector3d>();
	}

	Vector3d get_angles() const override;
private:
	//This joint only allows relative motion about axis a - stored in parent coordinates
	Vector3d rot_axe{0, 0, 0};

	//keep track of the joint limits as well - min and max allowed angles around the rotation axis
	double minAngle{0};
	double maxAngle{0};
};
