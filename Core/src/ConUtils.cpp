#include "ConUtils.h"
#include <cstring>

struct Con_keyword
{
	char keyword[40];
	con_utils value;
};

static Con_keyword con_keywords[] = {
	{"#", con_utils::comment},
	{"PDParams", con_utils::con_pd_gains_start},
	{"/PDParams", con_utils::con_pd_gains_end},
	{"ConState", con_utils::con_state_start},
	{"Con_OpenSim_State", con_utils::con_state_start_open_sim},
	{"/ConState", con_utils::con_state_end},
	{"nextState", con_utils::con_next_state},
	{"description", con_utils::con_state_description},
	{"transitionOn", con_utils::con_transition_on},
	{"stateStance", con_utils::con_state_stance},
	{"startingStance", con_utils::con_starting_stance},
	{"startAtState", con_utils::con_start_at_state},
	{"loadCharacterState", con_utils::con_character_state},
	{"load_open_sim_CharacterState", con_utils::con_character_state_open_sim},
	{"time", con_utils::con_state_time},
	{"trajectory", con_utils::con_trajectory_start},
	{"/trajectory", con_utils::con_trajectory_end},
	{"baseTrajectory", con_utils::con_base_trajectory_start},
	{"/baseTrajectory", con_utils::con_base_trajectory_end},
	{"rotationAxis", con_utils::con_rotation_axis},
	{"reverseTargetAngleOnStance", con_utils::con_reverse_angle_on_stance},
	{"loadXMLFile", con_utils::load_af_from_xml},
	{"loadRBFile", con_utils::load_rb_file},
	{"loadSimBiCon", con_utils::load_simbicon},
	{"loadRefTrajectory", con_utils::load_ref_trajectory_file},
	{"load_pelvis_pose_controller", con_utils::load_pelvis_pose_controller},
	{"component", con_utils::con_traj_component},
	{"/component", con_utils::con_traj_component_end},
	{"Kd", con_utils::con_kd},
	{"Kp", con_utils::con_kp},
	{"translationAxis", con_utils::con_transition_axis},
	{"load_swing_foot_controller", con_utils::load_swing_foot_controller},
	{"starting_phi", con_utils::starting_phi},
	{"starting_time_in_ref", con_utils::starting_time_in_ref},
};


con_utils get_con_line_type(char* & buffer)
{
	if (buffer[0] == '\0')
		return con_utils::comment;

	for (auto& keyword : con_keywords)
	{
		if (strncmp(buffer, keyword.keyword, strlen(keyword.keyword)) == 0)
		{
			buffer += strlen(keyword.keyword);
			return keyword.value;
		}
	}

	return con_utils::not_important;
}

const char* get_con_line_string(const con_utils line_type)
{
	for (auto& keyword : con_keywords)
	{
		if (keyword.value == line_type)
		{
			return keyword.keyword;
		}
	}

	return "ERROR! Unknown lineType";
}
