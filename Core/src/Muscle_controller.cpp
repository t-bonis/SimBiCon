#include "Muscle_controller.h"
#include "SimGlobals.h"

Muscle_controller::Muscle_controller(Character& character, SimBiCon_framework& control_framework) :
Controller_interface(character, control_framework)
{
	for (auto& muscle : m_controlled_character->get_muscles())
	{
		size_t s = muscle->get_neuronal_delay() * SimGlobals::timeFrequency;
		if (s == 0) s = 1;
		muscle->previousActivations.set_size(s);
	}
}

void Muscle_controller::reset_state() const
{
	for (auto& muscle : m_controlled_character->get_muscles())
	{
		muscle->previousActivations.reset();
		muscle->set_activation(0.0);
	}
}

void Muscle_controller::pre_simulation_step_phase_1()
{
	convert_desired_torques_to_muscle_torques_2();

}

void Muscle_controller::pre_simulation_step_phase_2()
{
}

void Muscle_controller::pre_simulation_step_phase_3()
{

}

void Muscle_controller::convert_desired_torques_to_muscle_torques()
{
	//compute the desired muscle activation for each muscle accordingly [Geijtenbeek2013]
	//and the corresponding neuronal excitation
	//and the resulting activation that will be applied
	for (auto& muscle : m_controlled_character->get_muscles())
	{
		const auto desired_force = compute_desired_force(*muscle);
		compute_desired_activation(*muscle, desired_force);
		const auto u = compute_desired_excitation(*muscle);

		muscle->update_activation(u, SimGlobals::dt);
		//update muscle's segments, length, contraction velocity, moment arms, force and torque
		//given the activation
		muscle->update_fields();
	}

	//let each muscle set its torque for all of its joints
	for (auto& muscle : m_controlled_character->get_muscles())
	{
		auto i = 0;
		for (auto& joint : muscle->get_joints())
		{
			//find the global index of the joint in the character
			joint->add_muscle_torque(muscle->get_torques_from_mtu(i));
			i++;
		}
	}
}

void Muscle_controller::convert_desired_torques_to_muscle_torques_2()
{
	//compute the desired muscle activation for each muscle accordingly [Geijtenbeek2013]
	//and the corresponding neuronal excitation
	//and the resulting activation that will be applied
	for (auto& muscle : m_controlled_character->get_muscles())
	{
		const auto desired_force = compute_desired_force(*muscle);
		compute_desired_activation(*muscle, desired_force);
		muscle->desired_activation = compute_desired_excitation(*muscle);

	}

	//for (auto& joint : m_controlled_character->get_joints())
	//{
	//	const auto desired_torque = joint->get_torque();
	//	for(auto& muscle :joint->get_muscles())
	//	{
	//		auto const moment_arm = muscle->get_moment_arm(joint->get_index_in_character());
	//		auto const moment_arm_magnitude = moment_arm.length();
	//		auto const moment_arm_norm = moment_arm / moment_arm_magnitude;
	//		auto const muscle_effect = desired_torque.dotProductWith(moment_arm_norm);
	//		if(muscle_effect < 0 && muscle->desired_activation > 0)
	//		{
	//			//qDebug() << muscle_effect << muscle->desired_activation << QString::fromStdString(muscle->get_name());
	//		}
	//	}
	//}

	for (auto& muscle : m_controlled_character->get_muscles())
	{
		muscle->update_activation(muscle->desired_activation, SimGlobals::dt);
		//update muscle's segments, length, contraction velocity, moment arms, force and torque
		//given the activation
		muscle->update_fields();
	}

	//let each muscle set its torque for all of its joints
	for (auto& muscle : m_controlled_character->get_muscles())
	{
		auto i = 0;
		for (auto& joint : muscle->get_joints())
		{
			//find the global index of the joint in the character
			joint->add_muscle_torque(muscle->get_torques_from_mtu(i));
			i++;
		}
	}
}

double Muscle_controller::compute_desired_force(Muscle& muscle)
{
	const auto joint_overlapped = muscle.get_joints().size(); //number of joints covered by this muscle
	if (joint_overlapped == 0) return 0;
	double sum = 0;
	double momentArmMagnitude = 0;
	//compute the average of the positive desired torques
	for (auto i = 0; i < joint_overlapped; i++)
	{
		const double desired_torque = compute_torque_to_apply(muscle, i, momentArmMagnitude);
		//if (desired_torque >= 0)
		sum += desired_torque / momentArmMagnitude;
	}

	//unless there aren't any positive torques, return the average
	double res = sum / joint_overlapped;
	if(res < 0)
	{
		res = 0;
	}
	return res;
}

double Muscle_controller::compute_torque_to_apply(Muscle& muscle, size_t jointId, double& momentArmMagnitude)
{
	//get the joint by its given local index (to the muscle)
	auto joint = muscle.get_joint(jointId);
	const Vector3d desired_torque = joint->get_torque();
	//use the formula [Geijtenbeek2013]: tau = (r / ||r||) . T
	Vector3d momentArm = muscle.get_moment_arm(joint->get_af_id());
	momentArmMagnitude = momentArm.length();
	momentArm = momentArm / momentArmMagnitude;
	return (momentArm).dotProductWith(desired_torque);
}


void Muscle_controller::compute_desired_activation(Muscle& muscle, const double desired_force)
{
	auto a = desired_force / muscle.get_max_iso_force();
	if (a > 1) { a = 1; }
	muscle.previousActivations.add(a);
}

double Muscle_controller::compute_desired_excitation(Muscle& muscle)
{
	if (muscle.previousActivations.is_empty())
	{
		return muscle.get_mtu()->initExcitation;
	}
	return muscle.previousActivations.get(); //get a previous activation value according to the muscle's neuronal delay
}



void Muscle_controller::update_muscles() const
{
	for (auto& muscle : m_controlled_character->get_muscles())
	{
		muscle->update_fields();
	}
}

void Muscle_controller::post_simulation_step()
{

}
