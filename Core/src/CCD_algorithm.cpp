#include "CCD_algorithm.h"
#include "Reduced_character_state.h"


CCD_algorithm::CCD_algorithm(const Articulated_figure& articulated_figure, Rigid_body* anchor, Rigid_body* end_effector)
{
	m_articulated_figure = std::make_shared<Articulated_figure>(articulated_figure);

	m_anchor_body = dynamic_cast<Articulated_rigid_body*>(m_articulated_figure->get_arb_by_name(anchor->get_name()));
	m_end_effector = dynamic_cast<Articulated_rigid_body*>(m_articulated_figure->get_arb_by_name(end_effector->get_name()));

	m_arbs_involved = m_articulated_figure->list_arbs_involved_form_anchor_to_target(*m_anchor_body, *m_end_effector);
	m_nb_arbs_involved = m_arbs_involved.size();

	m_nb_joints = m_nb_arbs_involved - 1;

	for (size_t i = 0; i < m_nb_joints; i++)
	{
		m_joints_involved.push_back(m_arbs_involved[i]->get_joint_with(*m_arbs_involved[i + 1]));
	}
}

void CCD_algorithm::set_end_joint_target_pos(const Point3d position, const Quaternion orientation)
{
	m_target_last_joint_pos = position;
	m_target_end_effector_orientation = orientation;
	m_target_set = true;
}

void CCD_algorithm::set_initial_state(std::vector<double> state)
{
	m_initial_state = state;
	auto temp_handler_initial = Reduced_character_state(state);
	auto temp_handler_line = Reduced_character_state(m_articulated_figure->get_state());

	for (auto& joint : m_joints_involved)
	{
		auto value = temp_handler_initial.get_joint_relative_orientation(joint->get_af_id());
		temp_handler_line.set_joint_relative_orientation(value, joint->get_af_id());
	}
	m_articulated_figure->set_state(temp_handler_line.get_state());
}

void CCD_algorithm::compute_result()
{
	if (!m_target_set)
	{
		throw std::logic_error("No target set");
	}
	std::vector<double> temp_state;

	auto n = m_nb_joints;
	auto last_joint_pos = m_joints_involved.back()->get_pos_in_global_coords();
	auto pos_error = (m_target_last_joint_pos - last_joint_pos).length();
	Quaternion orientation_error;
	auto orientation_diff = m_target_end_effector_orientation.get_inverse() * m_end_effector->get_orientation();
	Vector3d axis_diff;
	double angle_diff;
	orientation_diff.get_axis_and_angle(axis_diff, angle_diff);
	auto error = pos_error + angle_diff;

	while (!(error < m_error_threshold || m_iteration_count >= m_max_number_of_iteration))
	{
		if (m_nb_joints != 1)
		{

			if (n < 2)
			{
				n = m_nb_joints;
			}
			auto current_joint = m_joints_involved[n - 2];
			const auto relation = m_arbs_involved[n - 2]->get_relation_with(*current_joint);
			last_joint_pos = m_joints_involved.back()->get_pos_in_global_coords();
			auto current_joint_pos = current_joint->get_pos_in_global_coords();
			m_v_last = last_joint_pos - current_joint_pos;
			m_v_target = m_target_last_joint_pos - current_joint_pos;

			switch (current_joint->get_type()) {
			case Joint::stiff: break;
			case Joint::stiff_os: break;
			case Joint::hinge: break;
			case Joint::hinge_os:
			{
				m_rotation_axis = current_joint->get_rotation_axes_in_global_coords()[0].toUnit();

				// TODO : get two vector orthogonal vector in the plane of rotation
				Vector3d v1, v2;
				m_rotation_axis.get_orthogonal_vectors(&v1, &v2);

				v1 = v1.toUnit();
				v2 = v2.toUnit();

				// TODO : compute the projection of m_v on the plane	
				auto v_last_on_rot = v1 * (m_v_last.dotProductWith(v1) / pow(v1.length(), 2)) + v2 * (m_v_last.dotProductWith(v2) / pow(v2.length(), 2));
				auto v_target_on_rot = v1 * (m_v_target.dotProductWith(v1) / pow(v1.length(), 2)) + v2 * (m_v_target.dotProductWith(v2) / pow(v2.length(), 2));
				auto axis = v_last_on_rot.cross_product_with(v_target_on_rot).toUnit();

				// TODO : compute the angle of rotation 
				m_rotation_angle = v_target_on_rot.angle_with(v_last_on_rot);

				if (axis.dotProductWith(m_rotation_axis) > 0)
				{
					m_rotation_angle = -m_rotation_angle;
				}

				// TODO : check if angle of rotation is larger than the limit
				const auto limit_down = current_joint->get_rotation_limits()[0][0];
				const auto limit_up = current_joint->get_rotation_limits()[0][1];
				const auto current_angle = -current_joint->get_angles().x;
				if (m_rotation_angle + current_angle < limit_down)
				{
					m_rotation_angle = limit_down - current_angle;
				}
				else if (m_rotation_angle + current_angle > limit_up)
				{
					m_rotation_angle = limit_up - current_angle;
				}
				break;
			}
			case Joint::ball_in_socket:	break;
			case Joint::ball_in_socket_os:
			{
				m_rotation_axis = m_v_target.cross_product_with(m_v_last) / m_v_target.cross_product_with(m_v_last).length();

				m_rotation_angle = acos(m_v_target.dotProductWith(m_v_last) / (m_v_target.length()*m_v_last.length()));

				// TODO : Décomposer la rotation sur les trois axes x y z

				// TODO : trouver les angles de rotation sur les axes x y z

				// TODO : regarder si les angles dépasse les limites si ou, trouver la contrainte la plus forte et appliquer la 
				// ratio de réduction sur les trois angles
				break;
			}
			case Joint::universal: break;
			case Joint::custom: break;
			case Joint::undefined: break;
			default:;
			}

			// Position correction
			m_articulated_figure->get_state(temp_state);
			auto temp_handler = Reduced_character_state(temp_state);
			switch (relation)
			{
			case Articulated_rigid_body::parent:
			{
				Quaternion delta_orientation;
				delta_orientation.set_to_rotation_quaternion(-m_rotation_angle, m_rotation_axis.toUnit());
				auto previous_joint_relative_orientation_in_parent = temp_handler.get_joint_relative_orientation(current_joint->get_af_id());
				auto target_joint_relative_orientation_in_child = (previous_joint_relative_orientation_in_parent.get_inverse() * delta_orientation);
				auto target_joint_relative_orientation_in_parent = target_joint_relative_orientation_in_child.get_inverse();
				temp_handler.set_joint_relative_orientation(target_joint_relative_orientation_in_parent, current_joint->get_af_id());
				break;


			}
			case Articulated_rigid_body::child:
			{
				Quaternion delta_orientation;
				delta_orientation.set_to_rotation_quaternion(-m_rotation_angle, m_rotation_axis.toUnit());
				auto previous_joint_relative_orientation_in_parent = temp_handler.get_joint_relative_orientation(current_joint->get_af_id());
				auto target_joint_relative_orientation_in_parent = previous_joint_relative_orientation_in_parent * delta_orientation;
				temp_handler.set_joint_relative_orientation(target_joint_relative_orientation_in_parent, current_joint->get_af_id());
				break;
			}
			default:;
			}

			m_articulated_figure->set_arbs_lines_state(temp_handler.get_state(), m_arbs_involved);

			auto ideal_relative_orientation = m_arbs_involved[m_nb_arbs_involved - 2]->get_orientation().get_inverse() * m_target_end_effector_orientation;
			Quaternion target_relative_orientation;

			//Orientation correction
			switch (m_joints_involved.back()->get_type()) {
			case Joint::stiff: break;
			case Joint::stiff_os: break;
			case Joint::hinge: break;
			case Joint::hinge_os:
			{
				Vector3d axis;
				double angle;
				m_rotation_axis = m_joints_involved.back()->get_rotation_axes_in_global_coords()[0].toUnit();
				ideal_relative_orientation.get_axis_and_angle(axis, angle);
				target_relative_orientation = ideal_relative_orientation.decomposeRotation(m_rotation_axis);
				target_relative_orientation.get_axis_and_angle(axis, angle);
				break;
			}
			case Joint::ball_in_socket:	break;
			case Joint::ball_in_socket_os:
			{
				target_relative_orientation = ideal_relative_orientation;
				break;
			}
			case Joint::universal: break;
			case Joint::custom: break;
			case Joint::undefined: break;
			default:;
			}

			switch (relation)
			{
			case Articulated_rigid_body::parent:
			{
				auto target_joint_relative_orientation_in_parent = target_relative_orientation.get_inverse();
				temp_handler.set_joint_relative_orientation(target_joint_relative_orientation_in_parent, m_joints_involved.back()->get_af_id());
				m_articulated_figure->set_arbs_lines_state(temp_handler.get_state(), m_arbs_involved);
				break;
			}
			case Articulated_rigid_body::child:
			{
				auto target_joint_relative_orientation_in_parent = target_relative_orientation;
				temp_handler.set_joint_relative_orientation(target_joint_relative_orientation_in_parent, m_joints_involved.back()->get_af_id());
				m_articulated_figure->set_arbs_lines_state(temp_handler.get_state(), m_arbs_involved);
				break;
			}
			default:;
			}

			m_articulated_figure->get_state(temp_state);
			m_iteration_count += 1;
			n -= 1;
		}
		else
		{
			m_articulated_figure->get_state(temp_state);
			auto temp_handler = Reduced_character_state(temp_state);
			auto ideal_relative_orientation = m_end_effector->get_orientation().get_inverse() * m_target_end_effector_orientation;
			temp_handler.set_joint_relative_orientation(ideal_relative_orientation.get_inverse(), m_joints_involved.back()->get_af_id());
			m_articulated_figure->set_state(temp_handler.get_state());
			break;
		}
		orientation_diff = m_target_end_effector_orientation.get_inverse() * m_end_effector->get_orientation();
		orientation_diff.get_axis_and_angle(axis_diff, angle_diff);
		pos_error = (m_target_last_joint_pos - m_joints_involved.back()->get_pos_in_global_coords()).length();
		error = pos_error + angle_diff;
	}

	m_articulated_figure->get_state(temp_state);
	auto temp_handler_result = Reduced_character_state(m_initial_state);
	auto temp_handler_line = Reduced_character_state(temp_state);

	for (auto& joint : m_joints_involved)
	{
		auto value = temp_handler_line.get_joint_relative_orientation(joint->get_af_id());
		temp_handler_result.set_joint_relative_orientation(value, joint->get_af_id());
	}
	result = temp_handler_result.get_state();
}

std::vector<double> CCD_algorithm::get_result() const
{
	return result;
}
