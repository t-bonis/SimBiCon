#include "SimBiCon.h"
#include "ConUtils.h"
#include "Reduced_character_state.h"
#include "SimBiCon_framework.h"
#include <fstream>
#include <iomanip>   


SimBiCon::SimBiCon(char* filename, Articulated_figure& character, SimBiCon_framework& control_framework) :
	Controller_interface(character, control_framework)
{
	load_from_file(filename);
}

SimBiCon::SimBiCon(const SimBiCon& other, Articulated_figure& af, SimBiCon_framework& control_framework)
	:Controller_interface(af, control_framework)
{
	m_phi = other.m_phi;

	m_pose_controller = std::make_shared<Pose_controller>(*other.m_pose_controller, *this);

	for (const auto& other_fsm_state : other.m_controller_fsm_states)
	{
		m_controller_fsm_states.push_back(std::make_shared<Fsm_state>(*other_fsm_state, *this));
		//now we have to resolve all the joint names (i.e. figure out which joints they apply to).
		resolve_joints_open_sim(*m_controller_fsm_states.back()); //TODO: find a solution
	}

	m_controlled_character->set_default_state();

	for (auto& muscle : m_controlled_character->get_muscles())
	{
		muscle->update_fields();
	}

	m_starting_time = other.m_starting_time;

	set_fsm_state_to(other.m_current_fsm_state_id);

	set_stance(other.m_controlled_character->get_stance());
}

void SimBiCon::pre_simulation_step_phase_1()
{
	advance_in_time(SimGlobals::dt);

	compute_desired_pose();
	//compute_future_desired_pose();
}

void SimBiCon::pre_simulation_step_phase_2()
{

}

void SimBiCon::pre_simulation_step_phase_3()
{
	update_desired_pos();
	m_pose_controller->compute_torques();
}


void SimBiCon::post_simulation_step()
{

	is_body_touching_ground();
}

void SimBiCon::update_desired_pos() const
{
	auto rs = Reduced_character_state(m_pose_controller->desired_pose);

	rs.set_root_position(Vector3d(-1, 1, get_controlled_character()->get_arb_by_name("pelvis")->get_cm_position().z));
	rs.set_root_orientation(Quaternion(1, 0, 0, 0));

	if (m_framework)
	{
		m_framework->get_desired_pose_handler()->set_state(rs.get_state());
		for (auto& arb : m_framework->get_desired_pose_handler()->get_arbs())
		{
			arb->notify();
		}
	}
	else
	{
		std::cout << "Could not update desired pose" << std::endl;
	}


}

void SimBiCon::transition_to_state(const size_t state_index)
{
	set_fsm_state_to(state_index);
	set_stance(m_controller_fsm_states[m_current_fsm_state_id]->get_state_stance(m_controlled_character->get_stance()));
	this->m_phi = 0;
}

void SimBiCon::set_fsm_state_to(const size_t index)
{
	if (index < 0 || uint(index) >= m_controller_fsm_states.size())
	{
		m_current_fsm_state_id = 0;
		return;
	}
	m_current_fsm_state_id = index;
}

void SimBiCon::set_stance(Character::stance new_stance) const
{
	m_controlled_character->set_stance(new_stance);
}

void SimBiCon::get_state(SimBiCon::State& simbicon_state) const
{
	simbicon_state.phi = this->m_phi;
	simbicon_state.fsm_state_index = this->m_current_fsm_state_id;
}

void SimBiCon::set_state(const SimBiCon::State& simbicon_state)
{
	this->m_phi = simbicon_state.phi;
	this->set_fsm_state_to(simbicon_state.fsm_state_index);
}

Rigid_body* SimBiCon::get_rb_by_symbolic_name(char* sName) const
{
	char resolvedName[101];
	//deal with the SWING/STANCE_XXX' case
	if (strncmp(sName, "SWING_", strlen("SWING_")) == 0)
	{
		strcpy_s(resolvedName, sName + strlen("SWING_"));
		if (m_controlled_character->get_stance() == Character::left)
			resolvedName[0] = 'r';
		else
			resolvedName[0] = 'l';
	}
	else if (strncmp(sName, "STANCE_", strlen("STANCE_")) == 0)
	{
		strcpy_s(resolvedName, sName + strlen("STANCE_"));
		if (m_controlled_character->get_stance() == Character::left)
			resolvedName[0] = 'l';
		else
			resolvedName[0] = 'r';
	}
	else
		strcpy_s(resolvedName, sName);
	const auto result = m_controlled_character->get_arb_by_name(resolvedName);

	if (result == nullptr)
		throw std::logic_error("Could not find RB " + std::string(resolvedName) + " " + std::string(sName));

	return result;
}


void SimBiCon::compute_desired_pose()
{
	Reduced_character_state pose_rs(std::move(m_pose_controller->desired_pose));

	//always start from a neutral desired pose, and build from there...
	for (size_t i = 0; i < m_controlled_character->get_joints().size(); i++)
	{
		pose_rs.set_joint_relative_orientation(Quaternion(1, 0, 0, 0), i);
		pose_rs.set_joint_relative_ang_velocity(Vector3d(), i);
	}

	const auto cur_state = m_controller_fsm_states[m_current_fsm_state_id];
	const auto phi = MIN(m_phi, 1);
	const auto phi_future = MIN(m_phi + SimGlobals::dt / cur_state->get_state_duration(), 1);

	for (auto& trajectory : cur_state->get_trajectories())
	{
		const auto j_index = trajectory->get_joint_index(m_controlled_character->get_stance());

		auto new_orientation = trajectory->evaluate_trajectory(m_controlled_character->get_stance(), phi);
		auto new_orientation_future = trajectory->evaluate_trajectory(m_controlled_character->get_stance(), phi_future);

		//compute the target velocity
		Quaternion qDiff = new_orientation_future * new_orientation.get_conjugate();
		Vector3d ang_vel = Vector3d(0, 0, 0);
		if (new_orientation_future != new_orientation) {
			ang_vel = qDiff.v / (qDiff.v.length())*(safeACOS(qDiff.s) * 2 / SimGlobals::dt);
		}

		//if the index is -1, it must mean it's the root's trajectory. Otherwise we should give an error
		if (j_index == size_t(-1))
		{
			pose_rs.set_root_orientation(new_orientation);
			pose_rs.set_root_velocity(ang_vel);
		}
		else
		{
			pose_rs.set_joint_relative_orientation(new_orientation, j_index);
			pose_rs.set_joint_relative_ang_velocity(ang_vel, j_index);
		}
	}
	m_pose_controller->desired_pose = *pose_rs.get_state_ptr();
}

void SimBiCon::compute_future_desired_pose()
{
	Reduced_character_state pose_rs(std::move(m_pose_controller->desired_pose_dt));

	//always start from a neutral desired pose, and build from there...
	for (size_t i = 0; i < m_controlled_character->get_joints().size(); i++)
	{
		pose_rs.set_joint_relative_orientation(Quaternion(1, 0, 0, 0), i);
		pose_rs.set_joint_relative_ang_velocity(Vector3d(), i);
	}

	const auto cur_state = m_controller_fsm_states[m_current_fsm_state_id];
	const auto phi = MIN(m_phi + SimGlobals::dt, 1);
	const auto phi_future = MIN(m_phi + 2 * SimGlobals::dt / cur_state->get_state_duration(), 1);

	for (auto& trajectory : cur_state->get_trajectories())
	{
		const auto j_index = trajectory->get_joint_index(m_controlled_character->get_stance());

		auto new_orientation = trajectory->evaluate_trajectory(m_controlled_character->get_stance(), phi);
		auto new_orientation_future = trajectory->evaluate_trajectory(m_controlled_character->get_stance(), phi_future);

		//compute the target velocity
		Quaternion qDiff = new_orientation_future * new_orientation.get_conjugate();
		Vector3d ang_vel = Vector3d(0, 0, 0);
		if (new_orientation_future != new_orientation) {
			ang_vel = qDiff.v / (qDiff.v.length())*(safeACOS(qDiff.s) * 2 / SimGlobals::dt);
		}

		//if the index is -1, it must mean it's the root's trajectory. Otherwise we should give an error
		if (j_index == size_t(-1))
		{
			pose_rs.set_root_orientation(new_orientation);
			pose_rs.set_root_velocity(ang_vel);
		}
		else
		{
			pose_rs.set_joint_relative_orientation(new_orientation, j_index);
			pose_rs.set_joint_relative_ang_velocity(ang_vel, j_index);
		}
	}
	m_pose_controller->desired_pose_dt = *pose_rs.get_state_ptr();
}

void SimBiCon::write_state(std::string name)
{
	std::ofstream ofs;

	ofs.open(name);
	auto i = 0;
	for (auto& state : m_controller_fsm_states)
	{
		ofs << "Con_OpenSim_State " << i << std::endl;
		ofs << *state;
		++i;
	}
}

void SimBiCon::advance_in_time(const double dt)
{
	if (m_current_fsm_state_id >= m_controller_fsm_states.size())
	{
		throw std::logic_error("State out of range");
	}

	//advance the phase of the controller
	m_phi += dt / m_controller_fsm_states[m_current_fsm_state_id]->get_state_duration();
	//see if we have to transition to the next state in the FSM, and do it if so...
	if (m_controller_fsm_states[m_current_fsm_state_id]->need_transition())
	{
		const auto new_state_index = m_controller_fsm_states[m_current_fsm_state_id]->get_next_state_index();
		transition_to_state(new_state_index);
	}
}

void SimBiCon::is_body_touching_ground() const
{
	//see if anything else other than the feet touch the ground...
	for (auto& cf : m_controlled_character->get_contact_points())
	{
		if (cf->rb1->is_articulated() && cf->rb2->is_articulated())
		{
			auto arb1 = dynamic_cast<Articulated_rigid_body*>(cf->rb1);
			auto arb2 = dynamic_cast<Articulated_rigid_body*>(cf->rb2);
			if((m_controlled_character->is_left_arb(arb1) && m_controlled_character->is_right_arb(arb2))
			|| (m_controlled_character->is_left_arb(arb2) && m_controlled_character->is_right_arb(arb1)))
			{
				m_controlled_character->set_legs_touched(true);
			}
			continue;
		}

		if (cf->rb1->get_name() == "pelvis" || cf->rb2->get_name() == "pelvis")
		{
			m_controlled_character->set_body_touched_the_ground(true);
		}

		if (cf->rb1->get_name() == "torso" || cf->rb2->get_name() == "torso")
		{
			m_controlled_character->set_body_touched_the_ground(true);
		}
	}
}


void SimBiCon::resolve_joints(Fsm_state& state) const
{
	char tmp_name[101];
	for (auto& jt : state.get_trajectories())
	{
		//deal with the 'root' special case
		if (strcmp(jt->joint_name.c_str(), "root") == 0)
		{
			jt->left_stance_index = jt->right_stance_index = -1;
			continue;
		}
		//deal with the SWING_XXX' case
		if (strncmp(jt->joint_name.c_str(), "SWING_", strlen("SWING_")) == 0)
		{
			std::strcpy(tmp_name, jt->joint_name.c_str() + strlen("SWING"));
			tmp_name[0] = 'r';
			jt->left_stance_index = m_controlled_character->get_object_id(tmp_name);
			tmp_name[0] = 'l';
			jt->right_stance_index = m_controlled_character->get_object_id(tmp_name);
			continue;
		}
		//deal with the STANCE_XXX' case
		if (strncmp(jt->joint_name.c_str(), "STANCE_", strlen("STANCE_")) == 0)
		{
			std::strcpy(tmp_name, jt->joint_name.c_str() + strlen("STANCE"));
			tmp_name[0] = 'l';
			jt->left_stance_index = m_controlled_character->get_object_id(tmp_name);
			tmp_name[0] = 'r';
			jt->right_stance_index = m_controlled_character->get_object_id(tmp_name);
			continue;
		}
		//if we get here, it means it is just the name...
		jt->left_stance_index = m_controlled_character->get_object_id(jt->joint_name);
		jt->right_stance_index = jt->left_stance_index;
	}
}


void SimBiCon::resolve_joints_open_sim(Fsm_state& state) const
{
	char tmp_name[100];
	for (auto& jt : state.get_trajectories())
	{
		//deal with the 'root' special case
		if (strcmp(jt->joint_name.c_str(), "root") == 0)
		{
			jt->left_stance_index = jt->right_stance_index = -1;
			continue;
		}
		//deal with the SWING_XXX' case
		if (strncmp(jt->joint_name.c_str(), "SWING_", strlen("SWING_")) == 0)
		{
			std::strcpy(tmp_name, jt->joint_name.c_str() + strlen("SWING_"));
			strcat(tmp_name, "_r");
			jt->left_stance_index = m_controlled_character->get_object_id(tmp_name);
			if (jt->left_stance_index == size_t(-1))
				throw std::logic_error("Cannot find joint " + std::string(tmp_name));
			tmp_name[strlen(tmp_name) - 1] = 'l';
			jt->right_stance_index = m_controlled_character->get_object_id(tmp_name);
			if (jt->right_stance_index == size_t(-1))
				throw std::logic_error("Cannot find joint " + std::string(tmp_name));
			continue;
		}
		//deal with the STANCE_XXX' case
		if (strncmp(jt->joint_name.c_str(), "STANCE_", strlen("STANCE_")) == 0)
		{
			std::strcpy(tmp_name, jt->joint_name.c_str() + strlen("STANCE_"));
			strcat(tmp_name, "_l");
			jt->left_stance_index = m_controlled_character->get_object_id(tmp_name);
			if (jt->left_stance_index == size_t(-1))
				throw std::logic_error("Cannot find joint " + std::string(tmp_name));
			tmp_name[strlen(tmp_name) - 1] = 'r';
			jt->right_stance_index = m_controlled_character->get_object_id(tmp_name);
			if (jt->right_stance_index == size_t(-1))
				throw std::logic_error("Cannot find joint " + std::string(tmp_name));
			continue;
		}
		//if we get here, it means it is just the name...
		jt->left_stance_index = m_controlled_character->get_object_id(jt->joint_name);
		if (jt->left_stance_index == size_t(-1))
			throw std::logic_error("Cannot find joint " + std::string(jt->joint_name));
		jt->right_stance_index = jt->left_stance_index;
	}
}

void SimBiCon::load_from_file(char* f_name)
{
	if (f_name == nullptr)
		throw std::logic_error("NULL file name provided.");
	FILE* f;
	const auto err = fopen_s(&f, f_name, "r");
	if (err != 0 || f == nullptr)
		throw std::logic_error("Could not open file " + std::string(f_name));

	//to be able to load multiple controllers from multiple files,
	//we will use this offset to make sure that the state numbers
	//mentioned in each input file are updated correctly
	const auto state_offset = this->m_controller_fsm_states.size();
	size_t temp_state_nr = -1;

	//have a temporary buffer used to read the file line by line...
	char buffer[200];
	//this is where it happens.

	while (!feof(f))
	{
		//get a line from the file...
		fgets(buffer, 200, f);
		if (feof(f))
			break;
		if (strlen(buffer) > 195)
			throw std::logic_error("The input file contains a line that is longer than ~200 characters - not allowed");
		char* line = lTrim(buffer);
		const auto line_type = get_con_line_type(line);
		switch (line_type)
		{
		case con_utils::con_pd_gains_start:
			m_pose_controller = std::make_shared<Pose_controller>(f, *this);
			break;
		case con_utils::con_state_start:
		{
			sscanf_s(line, "%llu", &temp_state_nr);
			if (temp_state_nr != state_offset + this->m_controller_fsm_states.size())
				throw std::logic_error("Incorrect state offset specified: " + std::to_string(temp_state_nr));
			m_controller_fsm_states.push_back(std::make_shared<Fsm_state>(f, state_offset, *this));
			resolve_joints(*m_controller_fsm_states.back());
			break;
		}
		case con_utils::con_state_start_open_sim:
		{
			sscanf_s(line, "%llu", &temp_state_nr);
			if (temp_state_nr != state_offset + this->m_controller_fsm_states.size())
				throw std::logic_error("Incorrect state offset specified: " + std::to_string(temp_state_nr));
			m_controller_fsm_states.push_back(std::make_shared<Fsm_state>(f, state_offset, *this));
			//now we have to resolve all the joint names (i.e. figure out which joints they apply to).
			resolve_joints_open_sim(*m_controller_fsm_states.back());
			break;
		}
		case con_utils::starting_time_in_ref:
		{
			double starting_time = 0;
			sscanf_s(line, "%lf", &starting_time);
			m_starting_time = starting_time;
			break;
		}
		case con_utils::starting_phi:
		{
			sscanf_s(line, "%lf", &m_phi);
			break;
		}
		case con_utils::con_character_state:
			m_controlled_character->load_reduced_state_from_file(trim(line));
			break;
		case con_utils::con_character_state_open_sim:
		{
			auto state = std::vector<double>();
			m_controlled_character->read_reduced_state_from_file(trim(line), state);
			m_controlled_character->set_state(state);

			//update the muscle geometry based on the new character position
			for (auto& muscle : m_controlled_character->get_muscles())
			{
				muscle->update_fields();
			}
			break;
		}
		case con_utils::con_start_at_state:
			if (sscanf_s(line, "%llu", &temp_state_nr) != 1)
				throw std::logic_error("A starting state must be specified!");
			set_fsm_state_to(temp_state_nr);
			set_stance(m_controller_fsm_states[m_current_fsm_state_id]->get_state_stance(m_controlled_character->get_stance()));
			break;
		case con_utils::con_starting_stance:
			if (strncmp(trim(line), "left", 4) == 0)
			{
				set_stance(Character::left);
			}
			else if (strncmp(trim(line), "right", 5) == 0)
			{
				set_stance(Character::right);
			}
			else
				throw std::logic_error(
					R"(When using the 'reverseTargetOnStance' keyword, 'left' or 'right' must be specified!)");
			break;
		case con_utils::comment:
		case con_utils::not_important:
			break;
		default:
			throw std::logic_error("Incorrect SIMBICON input file: " + std::string(buffer) + " - unexpected line.");
		}
	}
	fclose(f);

	m_controlled_character->set_default_state();
}

std::shared_ptr<Fsm_state> SimBiCon::get_fsm_state(const size_t id) const
{
	if (id >= m_controller_fsm_states.size()) return nullptr;
	return m_controller_fsm_states[id];
}