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
	arb_position.resize(nb_of_arbs);
	arb_position_ref.resize(nb_of_arbs);
	arb_orientation.resize(nb_of_arbs);
	arb_orientation_ref.resize(nb_of_arbs);
	arb_linear_velocity.resize(nb_of_arbs);
	arb_linear_velocity_ref.resize(nb_of_arbs);
	arb_angular_velocity.resize(nb_of_arbs);
	arb_angular_velocity_ref.resize(nb_of_arbs);

	const auto nb_of_joint = articulated_figure->get_joints().size();
	joint_relative_orientation.resize(nb_of_joint);
	joint_relative_orientation_ref.resize(nb_of_joint);
	joint_relative_angular_velocity.resize(nb_of_joint);
	joint_relative_angular_velocity_ref.resize(nb_of_joint);
	torques1.resize(nb_of_joint);
	torques2.resize(nb_of_joint);
	forces1.resize(nb_of_joint);
	forces2.resize(nb_of_joint);


}

void Gait_analyzer::reset()
{
	arb_position.clear();
	arb_position_ref.clear();
	arb_orientation.clear();
	arb_orientation_ref.clear();
	arb_linear_velocity.clear();
	arb_linear_velocity.clear();
	arb_angular_velocity.clear();
	arb_angular_velocity_ref.clear();


	joint_relative_orientation.clear();
	joint_relative_orientation_ref.clear();
	joint_relative_angular_velocity.clear();
	joint_relative_angular_velocity_ref.clear();
	torques1.clear();
	torques2.clear();
	forces1.clear();
	forces2.clear();
}

double Gait_analyzer::compute_pose_control(std::array<size_t,2> interval)
{
	double global_diff = 0;
	for (size_t i = 0; i < joint_relative_orientation.size(); i++)
	{
		double per_joint_diff = 0;
		if(interval[1] - interval[0] == 0 )
		{
			interval[0] = 0;
			interval[1] = joint_relative_orientation[i].size();
		}
		for (size_t j = interval[0]; j < interval[1]; ++j)
		{
			Vector3d axis;
			double angle;

			(joint_relative_orientation[i][j] * joint_relative_orientation_ref[i][j].get_inverse()).get_axis_and_angle(axis, angle);

			per_joint_diff += angle + 0.1*pow((joint_relative_angular_velocity[i][j] - joint_relative_angular_velocity_ref[i][j]).length(),2);
		}
		per_joint_diff /= double(interval[1] - interval[0]);
		global_diff += per_joint_diff;
	}
	global_diff /= double(joint_relative_orientation.size());
	return global_diff;
}

double Gait_analyzer::compute_root_control(std::array<size_t, 2> interval)
{
	double global_diff = 0;
	const size_t pelvis_id = m_con_f->get_controlled_character()->get_object_id("pelvis");
	if (interval[1] - interval[0] == 0)
	{
		interval[0] = 0;
		interval[1] = arb_position_ref[pelvis_id].size();
	}
	for (size_t i = interval[0]; i < interval[1]; ++i)
	{

		Vector3d axis;
		double angle;

		(arb_orientation[pelvis_id][i] * arb_orientation_ref[pelvis_id][i].get_inverse()).get_axis_and_angle(axis, angle);

		global_diff += angle + 0.1*pow((arb_angular_velocity[pelvis_id][i] - arb_angular_velocity_ref[pelvis_id][i]).length(), 2);

	}
	global_diff /= double(interval[1] - interval[0]);
	return global_diff;
}

double Gait_analyzer::compute_end_effector_control(std::array<size_t, 2> interval)
{
	if (interval[1] - interval[0] == 0)
	{
		interval[0] = 0;
		interval[1] = arb_position_ref.back().size();
	}
	double global_diff = 0;
	std::vector<std::string> names = { "calcn_r", "calcn_l" };
	for (const auto& name : names)
	{
		auto id = m_con_f->get_controlled_character()->get_object_id(name);
		for (size_t i = interval[0]; i < interval[1]; ++i)
		{

			global_diff += (arb_position[id][i].y -arb_position_ref[id][i].y);
		}
		global_diff /= (interval[1] - interval[0]);
	}
	global_diff /= double(names.size());
	return global_diff;
}

double Gait_analyzer::compute_balance_control(std::array<size_t, 2> interval)
{
	if (interval[1] - interval[0] == 0)
	{
		interval[0] = 0;
		interval[1] = arb_position_ref.back().size();
	}
	double global_diff = 0;
	std::vector<std::string> names = { "calcn_r", "calcn_l" };

	for (const auto& name : names)
	{
		auto id = m_con_f->get_controlled_character()->get_object_id(name);
		for (size_t i = interval[0]; i < interval[1]; ++i)
		{
			auto r = com_pos[i] - Vector3d(arb_position[id][i]);
			r.y = 0;
			auto r_ref = com_pos_ref[i] - Vector3d(arb_position_ref[id][i]);
			r_ref.y = 0;
			global_diff += pow((r - r_ref).length(), 2);
		}
		global_diff /= double(interval[1] - interval[0]);
	}
	global_diff /= 1.8*double(names.size());
	double temp_diff = 0;
	for (size_t i = interval[0]; i < interval[1]; ++i)
	{
		temp_diff += 0.1*pow((com_vel[i] - com_vel_ref[i]).length(), 2);
	}
	temp_diff /= double(interval[1] - interval[0]);

	global_diff += temp_diff;

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

double Gait_analyzer::get_nb_frame()
{
	return double(arb_position_ref.back().size());
}


void Gait_analyzer::update(const Subject_interface* observable)
{
	if (const auto character = dynamic_cast<const Character*>(observable)) // main
	{
		//arb position, orientation, linear velocity and angular velocity
		for (const auto& arb : character->get_arbs())
		{
			arb_position[arb->get_af_id()].push_back(arb->get_cm_position());
			arb_orientation[arb->get_af_id()].push_back(arb->get_orientation());
			arb_linear_velocity[arb->get_af_id()].push_back(arb->get_cm_velocity());
			arb_angular_velocity[arb->get_af_id()].push_back(arb->get_angular_velocity());
		}

		// joint relative orientation and angular velocity
		for (const auto& joint : character->get_joints())
		{
			Quaternion qRel;
			joint->compute_relative_orientation(qRel);
			joint_relative_orientation[joint->get_af_id()].push_back(qRel);

			Vector3d wRel;
			joint->compute_relative_angular_velocity(wRel);
			joint_relative_angular_velocity[joint->get_af_id()].push_back(wRel);
		}

		//COM velocity and position
		com_vel.push_back(character->get_com_velocity());
		com_pos.push_back(character->get_com());

		//COP position
		cop.push_back(character->get_cop());

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
		//arb position, orientation, linear velocity and angular velocity
		for (const auto& arb : articulated_figure->get_arbs())
		{
			arb_position_ref[arb->get_af_id()].push_back(arb->get_cm_position());
			arb_orientation_ref[arb->get_af_id()].push_back(arb->get_orientation());
			arb_linear_velocity_ref[arb->get_af_id()].push_back(arb->get_cm_velocity());
			arb_angular_velocity_ref[arb->get_af_id()].push_back(arb->get_angular_velocity());
		}

		// joint relative orientation and angular velocity
		for (const auto& joint : articulated_figure->get_joints())
		{
			Quaternion qRel;
			joint->compute_relative_orientation(qRel);
			joint_relative_orientation_ref[joint->get_af_id()].push_back(qRel);

			Vector3d wRel;
			joint->compute_relative_angular_velocity(wRel);
			joint_relative_angular_velocity_ref[joint->get_af_id()].push_back(wRel);
		}

		//com_vel
		com_vel_ref.push_back(articulated_figure->get_com_velocity());
		com_pos_ref.push_back(articulated_figure->get_com());
	}
}

size_t Gait_analyzer::get_nb_of_missing_frame() const
{
	if(target_simulation_length != size_t(-1))
	{
		return (target_simulation_length)-arb_position.back().size() + 1;
	}
	return 0;
}


void Gait_analyzer::print_values() const
{
	auto m_temp_string = std::stringstream();

	//m_temp_string << angles << std::endl;

	//m_temp_string << joint_relative_orientation_ref << std::endl;

	m_temp_string << arb_position << std::endl;

	m_temp_string << arb_position_ref << std::endl;

	//m_temp_string << angular_velocity << std::endl;

	//m_temp_string << joint_relative_angular_velocity_ref << std::endl;

	//m_temp_string << angular_acceleration << std::endl;

	//m_temp_string << angular_acceleration_reference << std::endl;

	//m_temp_string << forces1 << std::endl;
	//m_temp_string << torques1 << std::endl;
	//m_temp_string << forces2 << std::endl;
	//m_temp_string << torques2 << std::endl;

	//m_temp_string << arb_angular_velocity << std::endl;

	BOOST_LOG_TRIVIAL(trace) << m_temp_string.str();

	BOOST_LOG_TRIVIAL(trace) << "end";
}

