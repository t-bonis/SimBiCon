#pragma once
#include "Utils/src/Utils.h"
#include "SimGlobals.h"
#include "Trajectory.h"
#include "Controller_interface.h"

enum class con_utils;

//This helper class is used to hold information regarding one component of a state trajectory. This includes (mainly): the base trajectory, 
//a data member that specifies the feedback law to be used, and the axis about which it represents a rotation, 
class Trajectory_component
{
public:
	Trajectory_component() = default;
	Trajectory_component(const Trajectory_component& other) = default;

	~Trajectory_component() = default;

	Trajectory_component(Trajectory_component&& other)  = delete;
	Trajectory_component& operator=(const Trajectory_component& other) = delete;
	Trajectory_component& operator=(Trajectory_component&& other)  = delete;

	//this method is used to evaluate the trajectory at a given point phi, knowing the stance of the character, 
	Quaternion evaluate_trajectory_component(size_t stance, double phi)
	{
		double base_angle = offset;
		if (base_trj.get_knot_count() > 0)
			base_angle += base_trj.evaluate_catmull_rom(phi);

		if (stance == Character::left && reverse_angle_on_left_stance)
			base_angle = -base_angle;
		if (stance == Character::right && reverse_angle_on_right_stance)
			base_angle = -base_angle;

		return Quaternion::get_rotation_quaternion(base_angle, rotation_axis);
	}

	void read_trajectory_component(FILE* f);
	bool is_implicit();
	void write_base_trajectory(FILE* f);

	friend std::ofstream& operator<< (std::ofstream& in, Trajectory_component& trajectory_component);

	//this is the array of basis functions that specify the trajectories for the sagittal plane.
	Trajectory1D base_trj;
	//if this variable is set to true, then when the stance of the character is the left side, the 
	//static target provided by this trajectory should be negated
	bool reverse_angle_on_left_stance{false};
	//if this variable is set to true, then when the stance of the character is the right side, the 
	//static target provided by this trajectory should be negated
	bool reverse_angle_on_right_stance{false};
	//this is the rotation axis that the angles obtained from the trajectory represent rotations about
	Vector3d rotation_axis{};

	//this is the base value for the trajectory
	double offset{0};


};


//This helper class is used to hold information regarding one trajectory. This includes: a sequence of components, 
//the index of the joint that this trajectory applies to, the coordinate frame in which the final arb_orientation is expressed, etc.
class Joint_trajectory
{
public:
	Joint_trajectory() = default;
	Joint_trajectory(std::string& joint_name, FILE* f)
	{
		left_stance_index = right_stance_index = -1;
		this->joint_name = joint_name;
		read_trajectory(f);
	}

	Joint_trajectory(const Joint_trajectory& other);

	~Joint_trajectory() = default;

	Joint_trajectory(Joint_trajectory&& other) = delete;
	Joint_trajectory& operator=(const Joint_trajectory& other) = delete;
	Joint_trajectory& operator=(Joint_trajectory&& other) = delete;


	//this method is used to evaluate the trajectory at a given point phi, knowing the stance of the character, 
	Quaternion evaluate_trajectory(size_t stance, double phi)
	{
		Quaternion q(1, 0, 0, 0);

		for (auto& component : components)
			q = q * component->evaluate_trajectory_component(stance, phi);

		return q;
	}

	//this method returns the joint index that this trajectory applies to, unless this applies to the root, in which case it returns -1.
	size_t get_joint_index(size_t stance) const
	{
		if (stance == Character::left)
		{
			return left_stance_index;
		}
		else
		{
			return right_stance_index;
		}
	}
	//This method is used to write a trajectory to a file
	void read_trajectory(FILE* f);

	//This method is used to write a trajectory to a file
	friend std::ofstream& operator<< (std::ofstream& in, const Joint_trajectory& joint_trajectory);

	Trajectory_component* get_component(Vector3d axis, bool allow_opposite = true);


public :
	std::vector<std::shared_ptr<Trajectory_component>> components;

	//if the biped that is controlled is in a left-sided stance, then this is the index of the joint that
	//the trajectory is used to control - it is assumed that if this is -1, then the trajectory applies
	//to the torso, and not to a joint
	size_t left_stance_index;

	//and this is the index of the joint that the trajectory is associated to if the biped is in a
	//right-side stance
	size_t right_stance_index;

	std::string joint_name{};
};


//A simbicon controller is a finite state machine. Transition between states happen on foot contact, time out, user interaction, etc.
//Each machine state holds the trajectories for all the joints that are controlled. 
class Fsm_state
{
public:
	Fsm_state() = delete;

	Fsm_state(FILE* f, size_t offset, Controller_interface& controller)
	{
		m_controller = &controller;
		read_state(f, offset);
	}
	Fsm_state(const Fsm_state& other, Controller_interface& controller);


	~Fsm_state() = default;

	Fsm_state(const Fsm_state& other) = delete;
	Fsm_state(Fsm_state&& other)  = delete;
	Fsm_state& operator=(const Fsm_state& other) = delete;
	Fsm_state& operator=(Fsm_state&& other)  = delete;

	//this method is used to determine the new stance, based on the information in this state and the old stance
	Character::stance get_state_stance(const Character::stance old_stance) const
	{
		if (m_keep_stance)
			return old_stance;
		if (!m_reverse_stance)
			return m_state_stance;
		if (old_stance == Character::left)
			return Character::right;
		return Character::left;
	}

	double get_state_duration() const { return m_state_duration; }

	size_t get_next_state_index() const { return m_next_state_index; }

	size_t get_trajectory_count() const { return m_trajectories.size(); }

	std::vector<std::shared_ptr<Joint_trajectory>> get_trajectories() const
	{
		return m_trajectories;
	}

	std::shared_ptr<Joint_trajectory> get_trajectory(const uint idx)
	{
		if (idx >= m_trajectories.size()) return nullptr;
		return m_trajectories[idx];
	}

	std::shared_ptr<Joint_trajectory> get_trajectory(const char* name)
	{
		for (auto& trajectory : m_trajectories)
			if (strcmp(trajectory->joint_name.c_str(), name) == 0)
				return trajectory;
		return nullptr;
	}

	//	This method is used to determine if the current state in the controller FSM needs to be transitioned from.
	bool need_transition() const;

	std::string get_description() const { return m_description; }

	void read_state(FILE* f, size_t offset);

	friend std::ofstream& operator<< (std::ofstream& in, const Fsm_state& state);
	
	//	This method is used to read the knots of a 1D trajectory from the file, where they are specified one (knot) on a line
	//	The trajectory is considered complete when a line starting with endingLineType is encountered
	static void read_trajectory1D(FILE* f, Trajectory1D& result, con_utils endingLineType);

	static void write_trajectory1D(FILE* f, Trajectory1D& result, con_utils startingLineType, con_utils endingLineType);

private:
	//this is the array of trajectories, one for each joint that is controlled
	std::vector<std::shared_ptr<Joint_trajectory>> m_trajectories;

	Controller_interface* m_controller;
	//this is a description of this state, for debugging purposes
	std::string m_description{"Uninitialized state"};
	//this is the number of the state that we should transition to in the controller's finite state machine
	size_t m_next_state_index{ size_t(-1) };

	//this is the amount of time that it is expected the biped will spend in this state
	double m_state_duration{0};

	//upon a transition to a new FSM state, it is assumed that the stance of the character either will be given stance, it will be reverseed , or keept the same.
	//if a state is designed for a certain stance, it is given by this variable
	//for generic states, this variable is used to determine if the stance should be reversed (as opposed to set to left or right), or stay the same.
	bool m_reverse_stance{false};
	//and if this is the same, then upon entering this FSM state, the stance will remain the same
	bool m_keep_stance{false};

	//if both keepStance and reverseStance are set to false, then this is the state that the character is assumed to take
	Character::stance m_state_stance{ Character::none};

	bool m_transition_on_foot_contact{true};
	//if we are to allow a transition on foot contact, we need to take care of the possibility that it
	//will occur early. In this case, we may still want to switch. If phi is at least this, then it is assumed
	//that we can transition;
	double m_min_phi_before_transition_on_foot_contact{0.9};
	//also, in order to make sure that we don't transition too early, we expect a minimum force applied on the swing foot before
	double m_min_swing_foot_force_for_contact{20};
};
