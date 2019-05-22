#include "Custom_joint.h"
#include <ode/objects.h>


Custom_joint::Custom_joint(const Custom_joint& other) : Joint(other)
{
	m_axis = other.m_axis;
	m_coordinates = other.m_coordinates;
}

void Custom_joint::fix_angular_constraint_parent_to_child(const Quaternion& qRel)
{
}


void Custom_joint::fix_angular_constraint_child_to_parent(const Quaternion& qRel)
{
}


void Custom_joint::read_axis_from_struct(Joint_from_xml joint_input)
{
	TransformAxis tempAxe;
	for (auto& axe : joint_input.transAxis)
	{
		tempAxe.set_coordinate(get_ptr_to_coords_by_name(axe.coordinate_name));

		tempAxe.set_function(axe);

		tempAxe.set_axis(axe.axis[0], axe.axis[1], axe.axis[2]);
		m_axis.push_back(tempAxe);
	}
}

void Custom_joint::load_coordinates_from_struct(Joint_from_xml joint_input)
{
	Coordinate temp_coords;
	for (auto& coordinate : joint_input.coordinates)
	{
		temp_coords.name = coordinate.name;
		temp_coords.range = coordinate.range;
		temp_coords.coords_value = 0;
		m_coordinates.push_back(temp_coords);
		m_use_joint_limits = true;
	}
}

Vector3d Custom_joint::get_angles() const
{
	switch (get_type())
	{
	case stiff_os:
		return Vector3d(0, 0, 0);
	case hinge_os:
	{
		Vector3d angles{ 0,0,0 };
		Vector3d axis;
		// get orientation of child arb relative to parent arb
		const auto child_orientation = m_child_arb->get_orientation();
		const auto parent_orientation = m_parent_arb->get_orientation();
		auto diff = parent_orientation * child_orientation.get_inverse();
		const auto global_axis = get_rotation_axes_in_global_coords()[0];
		diff.get_axis_and_angle(axis, angles.x);
		const auto error = axis - global_axis;
		if (abs(error.x) > 0.9 || abs(error.y) > 0.9 || abs(error.z) > 0.9)
		{
			angles.x = -angles.x;
		}
		return angles;
	}
	case ball_in_socket_os:
	{
		Vector3d angles;
		// get orientation of child arb relative to parent arb
		const auto child_orientation = m_child_arb->get_orientation();
		const auto parent_orientation = m_parent_arb->get_orientation();
		auto diff = parent_orientation * child_orientation.get_inverse();
		diff.get_euler_angles(angles);
		return angles;
	}
	default:
		return Vector3d(0, 0, 0);
	}
}

Coordinate* Custom_joint::get_ptr_to_coords_by_name(const std::string& a_name)
{
	for (auto& coords : m_coordinates)
	{
		if (coords.name == a_name)
		{
			return &coords;
		}
	}
	return nullptr;
}

Joint::type Custom_joint::get_type() const
{
	const auto nb_free_rot = m_axis[0].is_free() + m_axis[1].is_free() + m_axis[2].is_free();
	const auto nb_free_trans = m_axis[3].is_free() + m_axis[4].is_free() + m_axis[5].is_free();
	if (nb_free_rot == 3 && nb_free_trans == 0)
	{
		return ball_in_socket_os;
	}
	if (nb_free_rot == 1 && nb_free_trans == 0)
	{
		return hinge_os;
	}
	if (nb_free_rot == 0 && nb_free_trans == 0)
	{
		return stiff_os;
	}
	throw std::logic_error("Joint type undefined");
}

double Custom_joint::get_translation(const int index)
{
	return m_axis[index + 3].get_value();
}

std::vector<Vector3d> Custom_joint::get_rotation_axes_in_local_coords() const
{
	std::vector<Vector3d> rotation_axis;
	for (int i = 0; i < 3; i++)
	{
		if (m_axis[i].is_free())
		{
			rotation_axis.push_back(m_axis[i].get_axis());
		}
	}
	return rotation_axis;
}

std::vector<Vector3d> Custom_joint::get_rotation_axes_in_global_coords() const
{
	std::vector<Vector3d> rotation_axes;
	for (int i = 0; i < 3; i++)
	{
		if (m_axis[i].is_free())
		{
			//Global rotation axis is the local axis rotated by parent body orientation
			auto axis = m_parent_arb->get_orientation().rotate(m_axis[i].get_axis());
			rotation_axes.push_back(axis);
		}
	}
	return rotation_axes;
}

std::vector<std::vector<double>> Custom_joint::get_rotation_limits()
{
	std::vector<std::vector<double>> output;
	for (int i = 0; i < 3; i++)
	{
		if (m_axis[i].is_free())
		{
			output.push_back(m_axis[i].get_coordinate()->range);
		}
	}
	return output;
}


