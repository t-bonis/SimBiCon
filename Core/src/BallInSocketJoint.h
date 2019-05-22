#pragma once

#include "Joint.h"

//BallInSocket joints are free to rotate about any axis. However, in order to be able to apply joint limits, we will 
//identify some axis, about which we will place joint limits. In particular, we'll use a swing and twist decomposition
//of the relative orientations between the two bodies
class BallInSocketJoint : public Joint
{
public:
	BallInSocketJoint(char* axes);
	BallInSocketJoint(const BallInSocketJoint& other);
	BallInSocketJoint() = default;

	~BallInSocketJoint() = default;

	BallInSocketJoint(BallInSocketJoint&& other) = delete;
	BallInSocketJoint& operator=(const BallInSocketJoint& other) = delete;
	BallInSocketJoint& operator=(BallInSocketJoint&& other) = delete;

	//This method is used to fix the joint angular constraint to correct for drift. This is done by changing
	//the orientation of the child.
	void fix_angular_constraint_parent_to_child(const Quaternion& qRel) override;

	//This method is used to pass information regarding the joint limits for a joint. The string that is passed in is expected to
	//have been read from an input file.
	void read_joint_limits(char* limits) override;


	//This method is used to pass in information regarding the rotation axes. The string that is passed in is expected to have
	//been read from an input file.
	void read_axes(char* axes) override;

	//Returns the type of the current joint
	type get_type() const override { return ball_in_socket; }

	//[muscles]
	Vector3d get_angles() const override;

	std::vector<Vector3d> get_rotation_axes_in_local_coords() const override
	{
		return std::vector<Vector3d>({ swingAxis1, swingAxis2, twistAxis });
	}


	std::vector<std::vector<double>> get_rotation_limits() override
	{
		return std::vector<std::vector<double>>({
			{minSwingAngle1, maxSwingAngle1}, {minSwingAngle2, maxSwingAngle2}, {minTwistAngle, maxTwistAngle}
			});
	}

	std::vector<Vector3d> get_rotation_axes_in_global_coords() const override
	{
		return std::vector<Vector3d>(); //TODO : define
	}

	void fix_angular_constraint_child_to_parent(const Quaternion& qRel) override;
private:
	//these two axes define the plane of vectors along which the rotations represent a swing - stored in parent coordinates
	Vector3d swingAxis1, swingAxis2;
	//and this one is stored in child coordinates - this is the twist axis
	Vector3d twistAxis;
	//and the min and max allowed angles along the two swing axes (define an ellipsoid that can be offset if the min/max angles are not equal in magnitude)
	double minSwingAngle1{}, maxSwingAngle1{}, minSwingAngle2{}, maxSwingAngle2{};
	//and limits around the twist axis
	double minTwistAngle{}, maxTwistAngle{};
};
