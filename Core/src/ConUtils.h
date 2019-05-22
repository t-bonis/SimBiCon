#pragma once

enum class con_utils
{
	not_important = 1, 
	comment,
	con_pd_gains_start,
	con_pd_gains_end,
	con_state_start,
	con_state_start_open_sim,
	con_state_end,
	con_next_state,
	con_state_description,
	con_transition_on,
	con_state_stance,
	con_starting_stance,
	con_start_at_state,
	con_character_state_open_sim,
	con_character_state,
	con_state_time,
	con_trajectory_start,
	con_trajectory_end,
	con_reverse_angle_on_stance,
	con_rotation_axis,
	con_base_trajectory_start,
	con_base_trajectory_end,
	load_rb_file,
	load_af_from_xml,
	load_simbicon,
	con_traj_component,
	con_traj_component_end,
	load_ref_trajectory_file,
	load_pelvis_pose_controller,
	con_kd,
	con_kp,
	con_transition_axis,
	load_swing_foot_controller,
	starting_phi,
	starting_time_in_ref,
};


//	This method is used to determine the type of a line that was used in the input file for a rigid body.
//	It is assumed that there are no white spaces at the beginning of the string that is passed in. the pointer buffer
//	will be updated to point at the first character after the keyword.
con_utils get_con_line_type(char* & buffer);

//	This method is used to determine the string corresponding to a specific line keyword
const char* get_con_line_string(con_utils line_type);
