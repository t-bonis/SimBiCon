#pragma once
#include "Joint.h"
#include "Coordinate.h"
#include "TransformAxis.h"

class Custom_joint : public Joint
{
public:
	Custom_joint() = default;
	Custom_joint(const Custom_joint& other);

	~Custom_joint() = default;

	Custom_joint(Custom_joint&& other) = delete;
	Custom_joint& operator=(const Custom_joint& other) = delete;
	Custom_joint& operator=(Custom_joint&& other) = delete;

	void fix_angular_constraint_parent_to_child(const Quaternion& qRel) override;
	void fix_angular_constraint_child_to_parent(const Quaternion& qRel);

	void read_axis_from_struct(Joint_from_xml joint_input);

	void load_coordinates_from_struct(Joint_from_xml joint_input);

	//return the angles of rotation around axes (axes are fixed in the parent arb) 
	Vector3d get_angles() const override;

	Coordinate* get_ptr_to_coords_by_name(const std::string& a_name);
	
	Joint::type get_type() const override;

	double get_translation(int indice) override;

	std::vector<Vector3d> get_rotation_axes_in_local_coords() const override;

	std::vector<Vector3d> get_rotation_axes_in_global_coords() const override;

	std::vector<std::vector<double>> get_rotation_limits() override;

private:
	//6 axis : 3 rotation and 3 translation 
	std::vector<TransformAxis> m_axis;

	std::vector<Coordinate> m_coordinates;
};
