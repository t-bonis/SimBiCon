#pragma once
#include "Controller_interface.h"

// A muscular biped controller is an extension of a simbicon controller where the joint torques are computed based on a muscular model
// at each time step, we compute a desired torque (from the current to the target positions), the desired muscle contraction forces, 
// the desired activations and neuronal excitations, then apply the excitation and resulting activations, forces, and finally torques,
// which are put into an array muscleTorques
class Muscle_controller : public Controller_interface
{
public:
	explicit Muscle_controller(Character& character, SimBiCon_framework& control_framework);

	~Muscle_controller() = default;
		
	Muscle_controller() = delete;
	Muscle_controller(const Muscle_controller& other) = delete;
	Muscle_controller(Muscle_controller&& other)  = delete;
	Muscle_controller& operator=(const Muscle_controller& other) = delete;
	Muscle_controller& operator=(Muscle_controller&& other)  = delete;


	void reset_state() const;

	//update all the character's muscles
	void update_muscles() const;

	void pre_simulation_step_phase_1() override;

	void pre_simulation_step_phase_2() override;

	void pre_simulation_step_phase_3() override;

	void post_simulation_step() override;

	//fill the array muscleTorques : the torques converted from the muscle forces (and setDataFromPosition them to / replace them in the existing "torques" array)
	void convert_desired_torques_to_muscle_torques();
	void convert_desired_torques_to_muscle_torques_2();

	//compute the desired scalar torque for the given muscle (global character index) and the given joint (index local to the muscle)
	//also provide the corresponding momentArmMagnitude
	static double compute_torque_to_apply(Muscle& muscle, size_t jointId, double& momentArmMagnitude);

	//compute the desired scalar force for the given muscle, 
	//by computing an average of the desired torques of the joints it covers [Geijtenbeek2013]
	static double compute_desired_force(Muscle& muscle);

	//compute the desired muscle activation for the given muscle, 
	//using the desired force and the max isometric force [Geijtenbeek2013]
	static void compute_desired_activation(Muscle& muscle, double desired_force);

	//compute the desired muscle excitation for the given muscle, 
	//using the desired activation [Geijtenbeek2013]
	static double compute_desired_excitation(Muscle& muscle);

	Controller_interface::type get_type() override
	{
		return muscular_controller;
	}

};
