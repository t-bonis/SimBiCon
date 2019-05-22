#include "Pelvis_pose_control.h"
#include "SimBiCon_framework.h"
#include "CCD_algorithm.h"

Pelvis_pose_control::Pelvis_pose_control(Character& character, SimBiCon_framework& control_framework) :
	Controller_interface(character, control_framework)
{
	m_reference_pos = m_framework->get_reference_handler().get();
}

void Pelvis_pose_control::pre_simulation_step_phase_1()
{
}

void Pelvis_pose_control::pre_simulation_step_phase_2()
{
	//test_function_modify_desired_pose();
	control_with_stance_leg();
	//control_with_torso_2();
}

void Pelvis_pose_control::pre_simulation_step_phase_3()
{
}

void Pelvis_pose_control::post_simulation_step()
{

}

void Pelvis_pose_control::control_with_stance_leg() const
{
	const auto simbicon = std::dynamic_pointer_cast<SimBiCon>(m_framework->get_controllers()[0]);

	auto algorithm = std::make_unique<CCD_algorithm>(*dynamic_cast<Articulated_figure*>(m_controlled_character),
		m_controlled_character->get_stance_foot(),
		m_controlled_character->get_arb_by_name("pelvis"));

	auto temp_handler = Reduced_character_state(simbicon->get_pose_controller()->desired_pose);
	m_reference_pos->set_stance(m_controlled_character->get_stance());
	const auto hip_stance_pos = m_reference_pos->get_stance_hip()->get_pos_in_global_coords();
	auto target_pos = Vector3d(hip_stance_pos) + Vector3d(0, 0, 0);
	const auto pelvis_orientation = m_reference_pos->get_arb_by_name("pelvis")->get_orientation();

	algorithm->set_end_joint_target_pos(target_pos, pelvis_orientation);
	algorithm->set_initial_state(simbicon->get_pose_controller()->desired_pose);
	algorithm->compute_result();
	simbicon->get_pose_controller()->desired_pose = algorithm->get_result();

}

void Pelvis_pose_control::control_with_torso() const
{
	const auto simbicon = std::dynamic_pointer_cast<SimBiCon>(m_framework->get_controllers()[0]);

	auto algorithm = std::make_unique<CCD_algorithm>(*dynamic_cast<Articulated_figure*>(m_controlled_character),
		m_controlled_character->get_arb_by_name("torso"),
		m_controlled_character->get_arb_by_name("pelvis"));

	const auto torso_pos = m_controlled_character->get_arb_by_name("torso")->get_parent_joint()->get_pos_in_global_coords();
	const auto torso_desired_pos = m_reference_pos->get_arb_by_name("torso")->get_parent_joint()->get_pos_in_global_coords();
	auto input_pos = Vector3d(torso_pos);
	const auto axis = Vector3d(torso_pos, torso_desired_pos).cross_product_with(SimGlobals::up);
	const auto angle = atan(50*Vector3d(torso_pos, torso_desired_pos).length());
	Quaternion delta_orientation;
	delta_orientation.set_to_rotation_quaternion(angle, axis);
	const auto pelvis_orientation = m_controlled_character->get_arb_by_name("pelvis")->get_orientation() * delta_orientation;

	algorithm->set_end_joint_target_pos(input_pos, pelvis_orientation);
	algorithm->set_initial_state(simbicon->get_pose_controller()->desired_pose);
	algorithm->compute_result();
	simbicon->get_pose_controller()->desired_pose = algorithm->get_result();
}


void Pelvis_pose_control::control_with_torso_2() const
{
	const auto simbicon = std::dynamic_pointer_cast<SimBiCon>(m_framework->get_controllers()[0]);

	auto algorithm = std::make_unique<CCD_algorithm>(*dynamic_cast<Articulated_figure*>(m_controlled_character),
		m_controlled_character->get_arb_by_name("torso"),
		m_controlled_character->get_arb_by_name("pelvis"));

	const auto torso_pos = m_controlled_character->get_arb_by_name("torso")->get_parent_joint()->get_pos_in_global_coords();
	const auto com = m_controlled_character->get_com();
	const auto cop = m_controlled_character->get_cop();
	
	if(cop.length() == 0)
	{
		return;
	}
	const auto cop_com = com - cop;

	const auto axis = cop_com.cross_product_with(SimGlobals::up);
	const auto angle = 10 * cop_com.angle_with(SimGlobals::up);
	Quaternion delta_orientation;
	delta_orientation.set_to_rotation_quaternion(angle, axis);
	const auto pelvis_orientation = m_controlled_character->get_arb_by_name("pelvis")->get_orientation() * delta_orientation;

	algorithm->set_end_joint_target_pos(torso_pos, pelvis_orientation);
	algorithm->set_initial_state(simbicon->get_pose_controller()->desired_pose);
	algorithm->compute_result();
	simbicon->get_pose_controller()->desired_pose = algorithm->get_result();
}

void Pelvis_pose_control::test_function_modify_desired_pose() const
{
	const auto simbicon = std::dynamic_pointer_cast<SimBiCon>(m_framework->get_controllers()[0]);

	auto swing_foot = m_controlled_character->get_swing_foot();

	auto algorithm = std::make_unique<CCD_algorithm>(*dynamic_cast<Articulated_figure*>(m_controlled_character),
		swing_foot,
		m_controlled_character->get_arb_by_name("pelvis"));

	const auto swing_foot_pos = m_controlled_character->get_swing_foot()->get_parent_joint()->get_pos_in_global_coords();
	const auto swing_hip_pos = m_controlled_character->get_swing_hip()->get_pos_in_global_coords();

	auto target_pos = Vector3d(0.0, 0.2, 0.2);

	auto input_pos = Vector3d(swing_hip_pos) - Vector3d(target_pos) ;
	const auto pelvis_orientation = m_controlled_character->get_arb_by_name("pelvis")->get_orientation();
	
	algorithm->set_end_joint_target_pos(input_pos, pelvis_orientation);
	algorithm->set_initial_state(simbicon->get_pose_controller()->desired_pose);
	algorithm->compute_result();
	simbicon->get_pose_controller()->desired_pose = algorithm->get_result();
}

void Pelvis_pose_control::test_function_modify_desired_pose_2() const
{
	const auto simbicon = std::dynamic_pointer_cast<SimBiCon>(m_framework->get_controllers()[0]);

	auto algorithm = std::make_unique<CCD_algorithm>(*dynamic_cast<Articulated_figure*>(m_controlled_character),
		m_controlled_character->get_arb_by_name("torso"),
		m_controlled_character->get_arb_by_name("pelvis"));

	const auto torso_pos = m_controlled_character->get_arb_by_name("torso")->get_parent_joint()->get_pos_in_global_coords();

	auto input_pos = Vector3d(torso_pos);
	Quaternion delta_orientation;
	delta_orientation.set_to_rotation_quaternion(-1, Vector3d(1, 0, 0));
	const auto pelvis_orientation = m_controlled_character->get_arb_by_name("pelvis")->get_orientation() * delta_orientation;

	algorithm->set_end_joint_target_pos(input_pos, pelvis_orientation);
	algorithm->set_initial_state(simbicon->get_pose_controller()->desired_pose);
	algorithm->compute_result();
	simbicon->get_pose_controller()->desired_pose = algorithm->get_result();
}


void Pelvis_pose_control::test_function_pd_control() const
{
	Articulated_rigid_body* pelvis = m_controlled_character->get_arb_by_name("pelvis");

	std::cout << pelvis->get_cm_position() << std::endl;
	std::cout << pelvis->get_orientation() << std::endl;
	const auto force_to_apply = compute_pd_force(pelvis->get_cm_position(), Point3d(0.0, 1.10, 0), pelvis->get_cm_velocity(), Vector3d(0, 0, 0));
	const auto torque_to_apply = compute_pd_torque(pelvis->get_orientation(), Quaternion(1, 0, 0, 0), pelvis->get_angular_velocity(), Vector3d(0, 0, 0));


	const auto foot_r = m_controlled_character->get_arb_by_name("calcn_r");
	const auto foot_l = m_controlled_character->get_arb_by_name("calcn_l");

	m_controlled_character->apply_virtual_force(*pelvis, *foot_r, force_to_apply);
	m_controlled_character->apply_virtual_torque(*pelvis, *foot_r, torque_to_apply);

	m_controlled_character->apply_virtual_force(*pelvis, *foot_l, force_to_apply);
	m_controlled_character->apply_virtual_torque(*pelvis, *foot_l, torque_to_apply);
}

void Pelvis_pose_control::add_pelvis_pose_correction() const
{
	Articulated_rigid_body* pelvis = m_controlled_character->get_arb_by_name("pelvis");

	Articulated_rigid_body* pelvis_d = m_reference_pos->get_arb_by_name("pelvis");

	const auto force_to_apply = compute_pd_force(pelvis->get_cm_position(), pelvis_d->get_cm_position(), pelvis->get_cm_velocity(), pelvis_d->get_cm_velocity());

	const auto torque_to_apply = compute_pd_torque(pelvis->get_orientation(), pelvis_d->get_orientation(), pelvis->get_angular_velocity(), pelvis_d->get_angular_velocity());

	switch (m_controlled_character->get_stance()) {
	case Character::right:
	{
		const auto foot_r = m_controlled_character->get_arb_by_name("calcn_r");
		if (foot_r->get_contact_points().size() > 2)
		{
			m_controlled_character->apply_virtual_force(*pelvis, *foot_r, force_to_apply);
			m_controlled_character->apply_virtual_torque(*pelvis, *foot_r, torque_to_apply);
		}
		else
		{
			const auto tibia_r = m_controlled_character->get_arb_by_name("tibia_r");
			m_controlled_character->apply_virtual_force(*pelvis, *tibia_r, force_to_apply);
			m_controlled_character->apply_virtual_torque(*pelvis, *tibia_r, torque_to_apply);
		}
		break;
	}
	case Character::left:
	{
		const auto foot_l = m_controlled_character->get_arb_by_name("calcn_l");
		if (foot_l->get_contact_points().size() > 2)
		{
			m_controlled_character->apply_virtual_force(*pelvis, *foot_l, force_to_apply);
			m_controlled_character->apply_virtual_torque(*pelvis, *foot_l, torque_to_apply);
		}
		else
		{
			const auto tibia_r = m_controlled_character->get_arb_by_name("tibia_r");
			m_controlled_character->apply_virtual_force(*pelvis, *tibia_r, force_to_apply);
			m_controlled_character->apply_virtual_torque(*pelvis, *tibia_r, torque_to_apply);
		}

		break;
	}
	case Character::both:
	case Character::none:
	default:;
	}

}

Vector3d Pelvis_pose_control::compute_pd_force(const Point3d& pRel, const Point3d& pRelD, const Vector3d& vRel,
	const Vector3d& vRelD) const
{
	return (pRel - pRelD)*m_kp_f + (vRel - vRelD)*m_kd_f;
}

Vector3d Pelvis_pose_control::compute_pd_torque(const Quaternion& oRel, const Quaternion& oRelD, const Vector3d& vaRel,
	const Vector3d& vaRelD) const
{
	auto axis = Vector3d(0, 0, 0);
	auto angle = 0.0;
	(oRel * oRelD.get_inverse()).get_axis_and_angle(axis, angle);
	return axis * angle*m_kp_t + (vaRel - vaRelD)*m_kd_t;

}


Controller_interface::type Pelvis_pose_control::get_type()
{
	return pelvis_pose_controller;
}
