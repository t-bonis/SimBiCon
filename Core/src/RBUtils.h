#pragma once

enum class rb_utils
{
	rb_not_important,
	rb_rb,
	rb_end_rb,
	rb_arb,
	rb_mesh_name,
	rb_mass,
	rb_moi,
	rb_color,
	rb_root,
	rb_character,
	rb_child,
	rb_parent,
	rb_end_articulated_figure,
	rb_name,
	rb_joint,
	rb_end_joint,
	rb_ppos,
	rb_cpos,
	rb_sphere,
	rb_capsule,
	rb_plane,
	rb_locked,
	rb_position,
	rb_orientation,
	rb_velocity,
	rb_angular_velocity,
	rb_friction_coeff,
	rb_restitution_coeff,
	rb_min_bdg_sphere,
	rb_joint_type_hinge,
	rb_joint_limits,
	rb_joint_type_universal,
	rb_joint_type_ball_in_socket,
	rb_box,
	rb_ode_ground_coeff,
	rb_planar,
	rb_soft_body,
	rb_joint_type_stiff_joint,
	rb_muscle,
	rb_end_muscle,
	rb_max_force,
	rb_o_length,
	rb_s_length,
	rb_delay,
};

//This method is used to determine the type of a line that was used in the input file for a rigid body.
//It is assumed that there are no white spaces at the beginning of the string that is passed in. the pointer buffer
//will be updated to point at the first character after the keyword.
rb_utils getRBLineType(char* & buffer);
