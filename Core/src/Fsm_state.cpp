#include "Fsm_state.h"
#include "ConUtils.h"
#include "SimBiCon.h"
#include <iostream>
#include <fstream>

void Trajectory_component::write_base_trajectory(FILE* f)
{
	if (f == nullptr)
		return;

	fprintf(f, "\t\t\t%s\n", get_con_line_string(con_utils::con_base_trajectory_start));

	for (auto i = 0; i < base_trj.get_knot_count(); ++i)
	{
		fprintf(f, "\t\t\t\t%lf %lf\n", base_trj.getKnotPosition(i), base_trj.getKnotValue(i));
	}

	fprintf(f, "\t\t\t%s\n", get_con_line_string(con_utils::con_base_trajectory_end));
}

//void Trajectory_component::write_trajectory_component(FILE* f)
//{
//	if (f == nullptr)
//		return;
//
//	fprintf(f, "\t\t%s\n", get_con_line_string(con_utils::con_traj_component));
//
//	fprintf(f, "\t\t\t%s %lf %lf %lf\n", get_con_line_string(con_utils::con_rotation_axis),
//	        rotation_axis.x, rotation_axis.y, rotation_axis.z);
//
//	if (reverse_angle_on_left_stance)
//		fprintf(f, "\t\t\t%s left\n", get_con_line_string(con_utils::con_reverse_angle_on_stance));
//	else if (reverse_angle_on_right_stance)
//		fprintf(f, "\t\t\t%s right\n", get_con_line_string(con_utils::con_reverse_angle_on_stance));
//
//	write_base_trajectory(f);
//
//	fprintf(f, "\t\t%s\n", get_con_line_string(con_utils::con_traj_component_end));
//}



void Trajectory_component::read_trajectory_component(FILE* f)
{
	if (f == nullptr)
		throw std::logic_error("File pointer is NULL - cannot read gain coefficients!!");

	//have a temporary buffer used to read the file line by line...
	char buffer[200];

	//this is where it happens.
	while (!feof(f))
	{
		//get a line from the file...
		fgets(buffer, 200, f);
		if (strlen(buffer) > 195)
			throw std::logic_error("The input file contains a line that is longer than ~200 characters - not allowed");
		char* line = lTrim(buffer);
		con_utils lineType = get_con_line_type(line);
		switch (lineType)
		{
		case con_utils::con_traj_component_end:
			return;
		case con_utils::comment:
			break;
		case con_utils::con_rotation_axis:
			if (sscanf_s(line, "%lf %lf %lf", &this->rotation_axis.x, &this->rotation_axis.y,
			             &this->rotation_axis.z) != 3)
				throw std::logic_error("The axis for a trajectory is specified by three parameters!");
			this->rotation_axis.toUnit();
			break;
		case con_utils::con_base_trajectory_start:
			//read in the base trajectory
			Fsm_state::read_trajectory1D(f, base_trj, con_utils::con_base_trajectory_end);
			break;
		case con_utils::con_reverse_angle_on_stance:
			if (strncmp(trim(line), "left", 4) == 0)
				reverse_angle_on_left_stance = true;
			else if (strncmp(trim(line), "right", 5) == 0)
				reverse_angle_on_right_stance = true;
			else
				throw std::logic_error(
					R"(When using the 'startingStance' keyword, 'left' or 'right' must be specified!)");
			break;
		case con_utils::not_important:
			//tprintf("Ignoring input line: \'%s\'\n", line); 
			break;
		default:
			throw std::logic_error("Incorrect SIMBICON input file: " + std::string(buffer) + " - unexpected line.");
		}
	}
	throw std::logic_error("Incorrect SIMBICON input file: No '/trajectory' found ");
}

bool Trajectory_component::is_implicit()
{
	//if (!ref_trajectories.empty()) {
	//	return false;
	//} No reference trajectory map

	if (base_trj.get_knot_count() != 1) {
		return false;
	}

	if (base_trj.getKnotValue(0) != 0.0f || base_trj.getKnotPosition(0) != 0.5f) {
		return false;
	}

	return true;
}

Joint_trajectory::Joint_trajectory(const Joint_trajectory& other)
{
	
	for(const auto& other_component : other.components)
	{
		components.push_back(std::make_shared<Trajectory_component>(*other_component));
	}

	left_stance_index = other.left_stance_index;
	right_stance_index = other.right_stance_index;
	joint_name = other.joint_name;
}

//void Joint_trajectory::write_trajectory(FILE* f)
//{
//	if (f == nullptr)
//		return;
//
//	fprintf(f, "\t%s %s\n", get_con_line_string(con_utils::con_trajectory_start), joint_name.c_str());
//
//	for (auto& component : components)
//	{
//		fprintf(f, "\n");
//		component->write_trajectory_component(f);
//	}
//
//	fprintf(f, "\t%s\n", get_con_line_string(con_utils::con_trajectory_end));
//}

Trajectory_component* Joint_trajectory::get_component(Vector3d axis, bool allow_opposite)
{
	for (int j = 0; j < components.size(); ++j) {
		auto cur_comp = components[j];

		if (cur_comp->rotation_axis == axis) {
			return cur_comp.get();
		}

		if (allow_opposite) {
			if (cur_comp->rotation_axis == (axis*(-1))) {
				return cur_comp.get();
			}
		}

	}
	return nullptr;
}

void Joint_trajectory::read_trajectory(FILE* f)
{
	if (f == nullptr)
		throw std::logic_error("File pointer is NULL - cannot read gain coefficients!!");

	//have a temporary buffer used to read the file line by line...
	char buffer[200];

	//this is where it happens.
	while (!feof(f))
	{
		//get a line from the file...
		fgets(buffer, 200, f);
		if (strlen(buffer) > 195)
			throw std::logic_error("The input file contains a line that is longer than ~200 characters - not allowed");
		char* line = lTrim(buffer);
		con_utils lineType = get_con_line_type(line);
		switch (lineType)
		{
		case con_utils::con_trajectory_end:
			return;
		case con_utils::comment:
			break;
		case con_utils::con_traj_component:
			{
				//read in the base trajectory
				auto new_component = std::make_shared<Trajectory_component>();
				new_component->read_trajectory_component(f);
				components.push_back(new_component);
				break;
			}
		case con_utils::not_important:
			break;
		default:
			throw std::logic_error("Incorrect SIMBICON input file: " + std::string(buffer) + "- unexpected line.");
		}
	}
	throw std::logic_error("Incorrect SIMBICON input file: No \'/trajectory\' found ");
}

bool Fsm_state::need_transition() const
{
	const auto phi = dynamic_cast<SimBiCon*>(m_controller)->get_phase();
	//if it is a foot contact based transition
	if (m_transition_on_foot_contact)
	{
		const auto swing_arbs = m_controller->get_controlled_character()->get_swing_arbs();
		const auto swing_foot_vertical_force = fabs(m_controller->get_controlled_character()->get_force_from_ground(swing_arbs).dotProductWith(SimGlobals::up));


		//transition if we have a meaningful foot contact, and if it does not happen too early on...
		return (phi > m_min_phi_before_transition_on_foot_contact && swing_foot_vertical_force >
			m_min_swing_foot_force_for_contact) || phi >= 1;
	}

	// else transition is time based
	return phi >= 1;
}

Fsm_state::Fsm_state(const Fsm_state& other, Controller_interface& controller)
{
	m_controller = &controller;

	for(const auto& other_trajectory : other.m_trajectories)
	{
		m_trajectories.push_back(std::make_shared<Joint_trajectory>(*other_trajectory));
	}

	m_description = other.m_description;
	m_next_state_index = other.m_next_state_index;
	m_state_duration = other.m_state_duration;
	m_reverse_stance = other.m_reverse_stance;
	m_keep_stance = other.m_keep_stance;
	m_state_stance = other.m_state_stance;
	m_transition_on_foot_contact = other.m_transition_on_foot_contact;
	m_min_phi_before_transition_on_foot_contact = other.m_min_phi_before_transition_on_foot_contact;
	m_min_swing_foot_force_for_contact = other.m_min_swing_foot_force_for_contact;
}

void Fsm_state::read_state(FILE* f, size_t offset)
{
	if (f == nullptr)
		throw std::logic_error("File pointer is nullptr - cannot read gain coefficients!!");

	//have a temporary buffer used to read the file line by line...
	char buffer[200];


	//this is where it happens.
	while (!feof(f))
	{
		//get a line from the file...
		fgets(buffer, 200, f);
		if (strlen(buffer) > 195)
			throw std::logic_error("The input file contains a line that is longer than ~200 characters - not allowed");
		char* line = lTrim(buffer);
		const con_utils line_type = get_con_line_type(line);
		switch (line_type)
		{
		case con_utils::con_state_end:
			//we're done...
			return;
		case con_utils::con_next_state:
			if (sscanf(line, "%zu", &this->m_next_state_index) != 1)
				throw std::logic_error("An index must be specified when using the \'nextState\' keyword");
			this->m_next_state_index += offset;
			break;
		case con_utils::con_state_description:
			m_description =  trim(line);
			break;
		case con_utils::con_state_time:
			if (sscanf_s(line, "%lf", &m_state_duration) != 1)
				throw std::logic_error("The time that is expected to be spent in this state needs to be provided.");
			break;
		case con_utils::con_state_stance:
			m_reverse_stance = false;
			m_keep_stance = false;
			if (strncmp(trim(line), "left", 4) == 0)
				m_state_stance = Character::left;
			else if (strncmp(trim(line), "right", 5) == 0)
				m_state_stance = Character::right;
			else if (strncmp(trim(line), "reverse", 7) == 0)
				m_reverse_stance = true;
			else if (strncmp(trim(line), "same", 4) == 0)
				m_keep_stance = true;
			else
				throw std::logic_error(
					R"(When using the 'stateStance' keyword, 'left', 'right' or 'reverse' must be specified.)");
			break;
		case con_utils::con_transition_on:
			m_transition_on_foot_contact = false;
			if (strncmp(trim(line), "footDown", 8) == 0)
				m_transition_on_foot_contact = true;
			else if (strncmp(trim(line), "timeUp", 6) == 0)
				//nothings to do, since this is the default
				;
			else
				throw std::logic_error(
					R"(When using the 'transitionOn' keyword, 'footDown' or 'timeUp' must be specified.)");
			break;
		case con_utils::con_trajectory_start:
			{
				//create a new trajectory, and read its information from the file
				std::string joint_name = trim(line);
				m_trajectories.push_back(std::make_shared<Joint_trajectory>(joint_name, f));
				break;
			}
		case con_utils::comment:
		case con_utils::not_important:
			break;


		default:
			throw std::logic_error("Incorrect SIMBICON input file: " + std::string(buffer) + " - unexpected line.");
		}
	}
	throw std::logic_error("Incorrect SIMBICON input file: No /State found");
}

//void Fsm_state::write_state(FILE* f, int index)
//{
//	if (f == nullptr)
//		return;
//
//	fprintf(f, "%s %d\n", get_con_line_string(con_utils::con_state_start), index);
//
//	fprintf(f, "\t%s %s\n", get_con_line_string(con_utils::con_state_description), m_description.c_str());
//	fprintf(f, "\t%s %d\n", get_con_line_string(con_utils::con_next_state), (int)m_next_state_index);
//	fprintf(f, "\t%s %s\n", get_con_line_string(con_utils::con_transition_on),
//	        m_transition_on_foot_contact ? "footDown" : "timeUp");
//
//	if (m_reverse_stance)
//		fprintf(f, "\t%s reverse\n", get_con_line_string(con_utils::con_state_stance));
//	else if (m_keep_stance)
//		fprintf(f, "\t%s same\n", get_con_line_string(con_utils::con_state_stance));
//	else if (m_state_stance == Character::left)
//		fprintf(f, "\t%s left\n", get_con_line_string(con_utils::con_state_stance));
//	else if (m_state_stance == Character::right)
//		fprintf(f, "\t%s right\n", get_con_line_string(con_utils::con_state_stance));
//
//	fprintf(f, "\t%s %lf\n", get_con_line_string(con_utils::con_state_time), m_state_duration);
//
//	fprintf(f, "\n");
//
//	for (auto& m_trajectorie : m_trajectories)
//	{
//		fprintf(f, "\n");
//		m_trajectorie->write_trajectory(f);
//	}
//
//	fprintf(f, "%s\n", get_con_line_string(con_utils::con_state_end));
//}


void Fsm_state::read_trajectory1D(FILE* f, Trajectory1D& result, con_utils endingLineType)
{
	if (f == nullptr)
		throw std::logic_error("File pointer is NULL - cannot read gain coefficients!!");

	//have a temporary buffer used to read the file line by line...
	char buffer[200];
	double temp1, temp2;

	//this is where it happens.
	while (!feof(f))
	{
		//get a line from the file...
		fgets(buffer, 200, f);
		if (strlen(buffer) > 195)
			throw std::logic_error("The input file contains a line that is longer than ~200 characters - not allowed");
		char* line = lTrim(buffer);
		con_utils lineType = get_con_line_type(line);
		if (lineType == endingLineType)
			//we're done...
			return;

		switch (lineType)
		{
		case con_utils::comment:
			break;
		case con_utils::not_important:
			//we expect pairs of numbers, one pair on each row, so see if we have a valid pair
			if (sscanf_s(line, "%lf %lf", &temp1, &temp2) == 2)
				result.addKnot(temp1, temp2);
			//tprintf("Ignoring input line: \'%s\'\n", line); 
			break;
		default:
			throw std::logic_error("Incorrect SIMBICON input file: " + std::string(buffer) + " - unexpected line.");
		}
	}
	throw std::logic_error("Incorrect SIMBICON input file: StateTrajectory not closed ");
}

void Fsm_state::write_trajectory1D(FILE* f, Trajectory1D& result, con_utils startingLineType,
                                       con_utils endingLineType)
{
	if (f == nullptr)
		return;

	fprintf(f, "\t%s\n", get_con_line_string(startingLineType));

	for (int i = 0; i < result.get_knot_count(); ++i)
	{
		fprintf(f, "\t\t%lf %lf\n", result.getKnotPosition(i), result.getKnotValue(i));
	}

	fprintf(f, "\t%s\n", get_con_line_string(endingLineType));
}

std::ofstream & operator<<(std::ofstream & in, Trajectory_component& trajectory_component)
{
	in << "component" << std::endl;
	in << "rotationAxis " << trajectory_component.rotation_axis << std::endl;
	in << "baseTrajectory" << std::endl;
	for (size_t i = 0; i < trajectory_component.base_trj.get_knot_count() ; ++i)
	{
		in << std::fixed << std::setprecision(6);
		in << "\t" << trajectory_component.base_trj.getKnotPosition(i) << " ";
		in << trajectory_component.base_trj.getKnotValue(i) << std::endl;
	}
	in << "/baseTrajectory" << std::endl;
	in << "/component" << std::endl;
	return in;
}

std::ofstream & operator<<(std::ofstream & in, const Joint_trajectory& joint_trajectory)
{
	in << "trajectory " << joint_trajectory.joint_name << std::endl;
	for (const auto& comp : joint_trajectory.components)
	{
		in << *comp;
	}
	in << "/trajectory\n" << std::endl;
	return in;
}

std::ofstream & operator<<(std::ofstream & in, const Fsm_state& state)
{
	in << "description " << state.get_description() << std::endl;
	in << "nextState " << state.get_next_state_index() << std::endl;
	in << "stateStance " << state.m_state_stance << std::endl;
	if (state.m_transition_on_foot_contact)
	{
		in << "transitionOn footDown" << std::endl;
	}
	else
	{
		in << "transitionOn timeUp" << std::endl;

	}
	in << "time " << state.get_state_duration() << "\n" << std::endl;
	for (const auto& traj : state.m_trajectories)
	{
		in << *traj;
	}
	return in;
}