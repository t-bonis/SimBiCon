#include "Balance_control.h"
#include "SimBiCon.h"
#include "Ode_world.h"

Balance_control::Balance_control(Character& character, SimBiCon_framework& control_framework)
: Controller_interface(character, control_framework)
{
	std::cout << "Balance_controller initialized" << std::endl;
}


void Balance_control::pre_simulation_step_phase_1()
{
	add_gravity_compensation();
}

void Balance_control::pre_simulation_step_phase_2()
{
}

void Balance_control::pre_simulation_step_phase_3()
{

}

void Balance_control::post_simulation_step()
{
}

void Balance_control::add_gravity_compensation() const
{
	const auto pelvis = m_controlled_character->get_arb_by_name("pelvis");

	std::vector<Articulated_rigid_body*> targeted_arb;

	//List arbs others than stance leg and pelvis
	for(auto& arb : m_controlled_character->get_arbs())
	{
		if (arb->get_name() == "pelvis")
		{
			continue;
		}
		else
		{
			bool find = false;
			for(auto& stance_arb : m_controlled_character->get_stance_arbs())
			{
				if(arb->get_name() == stance_arb->get_name())
				{
					find = true;
					break;
				}
			}
			if(!find)
			{
				targeted_arb.push_back(arb.get());
			}
		}
	}

	// Appply F = -m*g
	for(auto& arb : targeted_arb)
	{
		m_controlled_character->apply_virtual_force(*arb, *pelvis, SimGlobals::up * (-arb->get_mass()));
	}
}

void Balance_control::correct_cop_position() const
{

}

Vector3d Balance_control::compute_pd_force(const Point3d& pRel, const Point3d& pRelD, const Vector3d& vRel,
	const Vector3d& vRelD)
{
	double const kp = 500;

	double const kd = 30;

	const auto force = (pRelD - pRel)*kp + (vRelD - vRel)*kd;

	return force;
}

//This method is used to compute the PD torque that aligns two frames.
Vector3d Balance_control::compute_pd_torque(const Quaternion& qRel, const Quaternion& qRelD, const Vector3d& wRel,
	const Vector3d& wRelD)
{
	double const kp = 500;

	double const kd = 30;

	Quaternion qErr = qRel.get_inverse() * qRelD;
	Vector3d axis;
	double angle;

	qErr.get_axis_and_angle(axis, angle);

	Vector3d torque = axis * angle * (-kp) + (wRelD - wRel) * kd;

	return torque;
}

Controller_interface::type Balance_control::get_type()
{
	return balance_controller;
}

