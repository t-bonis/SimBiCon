#include "Gait_analyzer.h"
#include "Ode_world.h"
#include "SimBiCon_framework.h"

Gait_analyzer::Gait_analyzer(SimBiCon_framework& conF)
{
	m_con_f = &conF;

	init_size_of_vectors();
}

void Gait_analyzer::init_size_of_vectors()
{
	const auto articulated_figure = m_con_f->get_physical_world()->get_character();

	const auto nb_of_arbs = articulated_figure->get_arbs().size();
	pos.resize(nb_of_arbs);
	pos_reference.resize(nb_of_arbs);
	vel.resize(nb_of_arbs);
	vel_reference.resize(nb_of_arbs);
	arb_angular_velocity.resize(nb_of_arbs);
	arb_angular_velocity_reference.resize(nb_of_arbs);
	orientation.resize(nb_of_arbs);
	orientation_reference.resize(nb_of_arbs);


	const auto nb_of_joint = articulated_figure->get_joints().size();
	angles.resize(nb_of_joint);
	angles_reference.resize(nb_of_joint);
	angular_velocity.resize(nb_of_joint);
	angular_velocity_reference.resize(nb_of_joint);
	angular_acceleration.resize(nb_of_joint);
	angular_acceleration_reference.resize(nb_of_joint);
	torques1.resize(nb_of_joint);
	torques2.resize(nb_of_joint);
	forces1.resize(nb_of_joint);
	forces2.resize(nb_of_joint);


}

void Gait_analyzer::reset()
{
	pos.clear();
	pos_reference.clear();
	orientation.clear();
	orientation_reference.clear();
	arb_angular_velocity.clear();
	arb_angular_velocity_reference.clear();


	angles.clear();
	angles_reference.clear();
	angular_velocity.clear();
	angular_velocity_reference.clear();
	angular_acceleration.clear();
	angular_acceleration_reference.clear();
	torques1.clear();
	torques2.clear();
	forces1.clear();
	forces2.clear();
}

double Gait_analyzer::compute_angular_diff(std::array<size_t,2> interval)
{
	double global_diff = 0;
	for (size_t i = 0; i < angles.size(); i++)
	{
		double per_joint_diff = 0;
		if(interval[1] - interval[0] == 0 )
		{
			interval[0] = 0;
			interval[1] = angles[i].size();
		}
		for (size_t j = interval[0]; j < interval[1]; ++j)
		{
			switch (m_con_f->get_physical_world()->get_joint_by_id(i)->get_type())
			{
			case Joint::stiff: break;
			case Joint::stiff_os: break;
			case Joint::hinge: break;
			case Joint::hinge_os:
				per_joint_diff += pow(angles[i][j].x - angles_reference[i][j].x, 2);
				break;
			case Joint::ball_in_socket: break;
			case Joint::ball_in_socket_os:
			{
				double per_angle_diff = pow(angles[i][j].x - angles_reference[i][j].x, 2);
				per_angle_diff += pow(angles[i][j].y - angles_reference[i][j].y, 2);
				per_angle_diff += pow(angles[i][j].z - angles_reference[i][j].z, 2);
				per_joint_diff += per_angle_diff / 3.0;
				break;
			}
			case Joint::universal: break;
			case Joint::custom: break;
			case Joint::undefined: break;
			default:;
			}
		}
		per_joint_diff /= (interval[1] - interval[0]);
		global_diff += per_joint_diff;

	}
	global_diff /= angles.size();
	return global_diff;
}

double Gait_analyzer::compute_pelvis_pos_diff(std::array<size_t, 2> interval)
{
	double global_diff = 0;
	const size_t pelvis_id = m_con_f->get_controlled_character()->get_object_id("pelvis");
	if (interval[1] - interval[0] == 0)
	{
		interval[0] = 0;
		interval[1] = pos_reference[pelvis_id].size();
	}
	for (size_t i = interval[0]; i < interval[1]; ++i)
	{

		global_diff += (pos_reference[pelvis_id][i] - pos[pelvis_id][i]).length();

	}
	global_diff /= (interval[1] - interval[0]);
	return global_diff;
}

double Gait_analyzer::compute_effort(std::array<size_t, 2> interval)
{
	double effort = 0;
	for (auto& muscle_activations : activations)
	{
		if (interval[1] - interval[0] == 0)
		{
			interval[0] = 0;
			interval[1] = muscle_activations.size();
		}
		for (auto& i = interval[0]; i < interval[1]; ++i)
		{
			effort += pow(muscle_activations[i], 2);
		}
	}
	return effort;
}


double Gait_analyzer::compute_pelvis_orientation_diff(std::array<size_t, 2> interval)
{
	double sum_diff = 0;
	for (size_t i = 0; i < orientation[0].size(); i++)
	{
		sum_diff += distanceBetweenOrientations(orientation[0][i], orientation_reference[0][i]);
	}
	sum_diff /= orientation[0].size();
	return sum_diff;
}

double Gait_analyzer::compute_gain_sum() const
{
	auto sum = 0.0;
	for (auto& controller : m_con_f->get_controllers())
	{
		if (std::dynamic_pointer_cast<SimBiCon>(controller))
		{
			const auto simbicon = std::dynamic_pointer_cast<SimBiCon>(controller);
			for (auto& gain : simbicon->get_pose_controller()->get_all_gains())
			{
				sum += gain;
			}
		}
	}
	return sum;
}



void Gait_analyzer::update(const Subject_interface* observable)
{
	if (const auto character = dynamic_cast<const Character*>(observable)) // main
	{
		//orientation and position
		auto i = 0;
		for (const auto& arb : character->get_arbs())
		{
			pos[i].push_back(arb->get_cm_position());
			orientation[i].push_back(arb->get_orientation());
			vel[i].push_back(arb->get_cm_velocity());
			i++;
		}

		// joint angular velocities
		for (const auto& joint : character->get_joints())
		{
			Vector3d wRel;
			joint->compute_relative_angular_velocity(wRel);
			angular_velocity[joint->get_af_id()].push_back(wRel);
		}


		// arb angular velocities
		for (const auto& arb : character->get_arbs())
		{
			arb_angular_velocity[arb->get_af_id()].push_back(arb->get_angular_velocity());
		}

		for (const auto& joint : character->get_joints())
		{
			Vector3d wRel;
			joint->compute_relative_angular_acceleration(wRel);
			angular_acceleration[joint->get_af_id()].push_back(wRel);
		}

		// joints angles
		for (const auto& joint : character->get_joints())
		{
			angles[joint->get_af_id()].push_back(joint->get_angles());
		}

		//cop
		cop.push_back(character->get_cop());

		//com_vel
		com_vel.push_back(character->get_com_velocity());

		// nb_of_contact point
		size_t nb_left = 0;
		for (auto arb : character->get_left_side_articulated_rigid_bodies())
		{
			for (auto cp : arb->get_contact_points())
			{
				if (cp->rb1->get_name() == "ground" || cp->rb2->get_name() == "ground")
				{
					nb_left++;
				}
			}
		}

		l_foot_nb_contact.push_back(nb_left);

		size_t nb_right = 0;
		for (auto arb : character->get_right_side_articulated_rigid_bodies())
		{
			for (auto cp : arb->get_contact_points())
			{
				if (cp->rb1->get_name() == "ground" || cp->rb2->get_name() == "ground")
				{
					nb_right++;
				}
			}
		}

		r_foot_nb_contact.push_back(nb_right);

		//force on foot
		force_on_l_foot.push_back(character->get_force_from_ground(character->get_left_side_articulated_rigid_bodies()));
		force_on_r_foot.push_back(character->get_force_from_ground(character->get_right_side_articulated_rigid_bodies()));

		torque_on_l_foot.push_back(character->get_torque_from_ground(character->get_left_side_articulated_rigid_bodies()));
		torque_on_r_foot.push_back(character->get_torque_from_ground(character->get_right_side_articulated_rigid_bodies()));



		//torques and forces on articulations
		for (const auto& joint : character->get_joints())
		{
			const auto joint_feedback = dynamic_cast<Ode_world*>(character->get_physique_engine())->m_joints_feedback[
				joint->get_af_id()];

			forces1[joint->get_af_id()].emplace_back(joint_feedback.f1[0], joint_feedback.f1[1], joint_feedback.f1[2]);
			torques1[joint->get_af_id()].emplace_back(joint_feedback.t1[0], joint_feedback.t1[1], joint_feedback.t1[2]);
			forces2[joint->get_af_id()].emplace_back(joint_feedback.f2[0], joint_feedback.f2[1], joint_feedback.f2[2]);
			torques2[joint->get_af_id()].emplace_back(joint_feedback.t2[0], joint_feedback.t2[1], joint_feedback.t2[2]);
		}

	}
	else if (const auto articulated_figure = dynamic_cast<const Articulated_figure*>(observable)) //reference or desired
	{
		auto i = 0;
		for (const auto& arb : articulated_figure->get_arbs())
		{
			pos_reference[i].push_back(arb->get_cm_position());
			orientation_reference[i].push_back(arb->get_orientation());
			vel_reference[i].push_back(arb->get_cm_velocity());
			i++;
		}

		//com_vel
		com_vel_ref.push_back(articulated_figure->get_com_velocity());


		// joint angular velocities
		for (const auto& joint : articulated_figure->get_joints())
		{
			Vector3d wRel;
			joint->compute_relative_angular_velocity(wRel);
			angular_velocity_reference[joint->get_af_id()].push_back(wRel);
		}

		// arb angular velocities
		for (const auto& arb : articulated_figure->get_arbs())
		{
			arb_angular_velocity_reference[arb->get_af_id()].push_back(arb->get_angular_velocity());
		}


		for (const auto& joint : articulated_figure->get_joints())
		{
			Vector3d wwRel;
			joint->compute_relative_angular_acceleration(wwRel);
			angular_acceleration_reference[joint->get_af_id()].push_back(wwRel);
		}

		for (const auto& joint : articulated_figure->get_joints())
		{
			angles_reference[joint->get_af_id()].push_back(joint->get_angles());
		}
	}
}

size_t Gait_analyzer::get_nb_of_missing_frame() const
{
	if(target_simulation_length != size_t(-1))
	{
		return (target_simulation_length)-pos.back().size() + 1;
	}
	return 0;
}


void Gait_analyzer::print_values() const
{
	auto m_temp_string = std::stringstream();

	//m_temp_string << angles << std::endl;

	m_temp_string << angles_reference << std::endl;

	//m_temp_string << pos << std::endl;

	m_temp_string << pos_reference << std::endl;

	//m_temp_string << angular_velocity << std::endl;

	m_temp_string << angular_velocity_reference << std::endl;

	//m_temp_string << angular_acceleration << std::endl;

	m_temp_string << angular_acceleration_reference << std::endl;

	//m_temp_string << forces1 << std::endl;
	//m_temp_string << torques1 << std::endl;
	//m_temp_string << forces2 << std::endl;
	//m_temp_string << torques2 << std::endl;

	//m_temp_string << arb_angular_velocity << std::endl;

	BOOST_LOG_TRIVIAL(trace) << m_temp_string.str();

	BOOST_LOG_TRIVIAL(trace) << "end";
}

