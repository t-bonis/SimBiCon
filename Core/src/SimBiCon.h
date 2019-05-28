#pragma once
#include "Trajectory.h"
#include "Fsm_state.h"
#include "Controller_interface.h"
#include "Pose_controller.h"


//A simbicon controller is a fancy PoseController. The root (i.e. pelvis or torso), as well as the two hips are controlled
//relative to a semi-global coordinate frame (it needs only have the y-axis pointing up), but all other joints are controlled
//exactly like in a normal PoseController. The SimBiCon operates on a biped, which means that we can make special
//assumptions about it: it must have two joints, lHip and rHip, connecting the root to the left and right upper-legs,
//and it must also have two feet (lFoot and rFoot) as rigid bodies in the articulated linkage.
class SimBiCon : public Controller_interface
{
public:
	struct State
	{
		double phi;
		size_t fsm_state_index;
	};

public:


	SimBiCon(char* filename, Articulated_figure& character, SimBiCon_framework& control_framework);
	SimBiCon(const SimBiCon& other, Articulated_figure& af, SimBiCon_framework& control_framework);

	~SimBiCon() = default;

	SimBiCon(SimBiCon&& other) = delete;
	SimBiCon& operator=(const SimBiCon& other) = delete;
	SimBiCon& operator=(SimBiCon&& other) = delete;
	
	void update_desired_pos() const;

	void load_from_file(char* f_name);

	//First we will compute the pose (i.e. relative orientations), using the joint trajectories for the current state
	//Second we will compute the PD torques that are needed to drive the links towards their orientations
	void pre_simulation_step_phase_1() override;

	void pre_simulation_step_phase_2() override;

	void pre_simulation_step_phase_3() override;

	void compute_desired_pose();
	void compute_future_desired_pose();

	void post_simulation_step() override;

	void is_body_touching_ground() const;

	void get_state(SimBiCon::State& simbicon_state) const;
	void set_state(const SimBiCon::State& simbicon_state);

	std::shared_ptr<Pose_controller> get_pose_controller() const
	{
		return m_pose_controller;
	}

	void set_pose_controller(const std::shared_ptr<Pose_controller>& pose_controller)
	{
		m_pose_controller = pose_controller;
	}

	double get_phase() const
	{
		return m_phi;
	}

	double get_current_state_time() const
	{
		return get_fsm_state(m_current_fsm_state_id)->get_state_duration()*m_phi;
	}

	size_t get_fsm_state_id() const
	{
		return this->m_current_fsm_state_id;
	}

	//Returns null if the index is out of range
	std::shared_ptr<Fsm_state> get_fsm_state(size_t id) const;

	std::vector<std::shared_ptr<Fsm_state>> get_fsm_states() const
	{
		return m_controller_fsm_states;
	}

	type get_type() override
	{
		return simbicon;
	}

	double get_starting_time() const
	{
		return m_starting_time;
	}

	void write_state(std::string name);

protected:
	//This method is used to advance the controller in time. It takes in a list of the contact points, since they might be
	//used to determine when to transition to a new state. This method returns -1 if the controller does not advance to a new state,
	//or the index of the state that it transitions to otherwise.
	void advance_in_time(double dt);

	//This method is used to set the current FSM state of the controller to that of the index that
	//is passed in.
	void set_fsm_state_to(size_t index);

	//This method should be called when the controller transitions to this state.
	void transition_to_state(size_t state_index);

	//This method is used to resolve the names (map them to their index) of the joints
	void resolve_joints(Fsm_state& state) const;

	void resolve_joints_open_sim(Fsm_state& state) const;

	//This method is used to set the stance
	void set_stance(Character::stance new_stance) const;

	//This method is used to return a pointer to a rigid body, based on its name (SWING_XXX or STANCE_XXX)
	Rigid_body* get_rb_by_symbolic_name(char* sName) const;

protected:
	//this is a collection of the states that are used in the controller
	std::vector<std::shared_ptr<Fsm_state>> m_controller_fsm_states;

	//the root is not directly controlled by any joint, so we will store its Kp, Kd and maxTorque separated here.
	//while the pose controller does not use these values, other types of controllers may need this information
	std::shared_ptr<Pose_controller> m_pose_controller;

	//this is the index of the controller that is currently active
	size_t m_current_fsm_state_id;

	//the phase parameter, phi must have values between 0 and 1, and it indicates the progress through the current state.
	double m_phi;

	double m_starting_time;

};
