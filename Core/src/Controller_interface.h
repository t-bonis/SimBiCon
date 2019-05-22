#pragma once
#include "Character.h"

class SimBiCon_framework;
// 	This class is used to provide a generic interface to a controller. A controller acts on a character - it computes torques that are
//	applied to the joints of the character. The details of how the torques are computed are left up to the classes that extend this one.
class Controller_interface
{
public:
	enum type { undefined, muscular_controller, simbicon, pelvis_pose_controller, balance_controller, swing_foot_controller, velocity_controller
	};

public :
	Controller_interface() = delete;
	Controller_interface(Articulated_figure& character, SimBiCon_framework& framework);
	
	virtual ~Controller_interface() = default;

	Controller_interface(Controller_interface&& other)  = delete;
	Controller_interface& operator=(const Controller_interface& other) = delete;
	Controller_interface& operator=(Controller_interface&& other)  = delete;

	virtual void pre_simulation_step_phase_1() = 0;

	virtual void pre_simulation_step_phase_2() = 0;

	virtual void pre_simulation_step_phase_3() = 0;

	virtual void post_simulation_step() = 0;
	
	virtual type get_type() = 0;

	//############################
	// Getter
	//############################

	Character* get_controlled_character() const
	{
		return m_controlled_character;
	}

	//############################
	// Setter
	//############################


	void set_controlled_character(Character* controlled_character)
	{
		m_controlled_character = controlled_character;
	}

protected:
	Character* m_controlled_character;

	SimBiCon_framework* m_framework;
};
