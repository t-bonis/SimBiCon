#include "RBUtils.h"
#include <cstring>


struct Rb_keyword
{
	char keyword[25];
	rb_utils value;
};

static Rb_keyword rb_keywords[] = {
	{"RigidBody", rb_utils::rb_rb},
	{"/End", rb_utils::rb_end_rb},
	{"A_RigidBody", rb_utils::rb_arb},
	{"mesh", rb_utils::rb_mesh_name},
	{"mass", rb_utils::rb_mass},
	{"moi", rb_utils::rb_moi},
	{"colour", rb_utils::rb_color},
	{"root", rb_utils::rb_root},
	{"ArticulatedFigure", rb_utils::rb_character},
	{"child", rb_utils::rb_child},
	{"parent", rb_utils::rb_parent},
	{"/ArticulatedFigure", rb_utils::rb_end_articulated_figure},
	{"name", rb_utils::rb_name},
	{"Joint", rb_utils::rb_joint},
	{"/Joint", rb_utils::rb_end_joint},
	{"jointPPos", rb_utils::rb_ppos},
	{"jointCPos", rb_utils::rb_cpos},
	{"CDP_Sphere", rb_utils::rb_sphere},
	{"CDP_Capsule", rb_utils::rb_capsule},
	{"CDP_Plane", rb_utils::rb_plane},
	{"static", rb_utils::rb_locked},
	{"position", rb_utils::rb_position},
	{"orientation", rb_utils::rb_orientation},
	{"velocity", rb_utils::rb_velocity},
	{"angularVelocity", rb_utils::rb_angular_velocity},
	{"frictionCoefficient", rb_utils::rb_friction_coeff},
	{"restitutionCoefficient", rb_utils::rb_restitution_coeff},
	{"minBoundingSphere", rb_utils::rb_min_bdg_sphere},
	{"hingeJoint", rb_utils::rb_joint_type_hinge},
	{"jointLimits", rb_utils::rb_joint_limits},
	{"universalJoint", rb_utils::rb_joint_type_universal},
	{"ballInSocketJoint", rb_utils::rb_joint_type_ball_in_socket},
	{"stiffJoint", rb_utils::rb_joint_type_stiff_joint},
	{"CDP_Box", rb_utils::rb_box},
	{"planar", rb_utils::rb_planar},
	{"ODEGroundParameters", rb_utils::rb_ode_ground_coeff},
	{"softBody", rb_utils::rb_soft_body},
	{"Muscle", rb_utils::rb_muscle},
	{"/Muscle", rb_utils::rb_end_muscle},
	{"maxForce", rb_utils::rb_max_force},
	{"optimalLength", rb_utils::rb_o_length},
	{"slackLength", rb_utils::rb_s_length},
	{"delay", rb_utils::rb_delay}
};

rb_utils getRBLineType(char*& buffer)
{
	for (auto& rb_keyword : rb_keywords)
	{
		if (strncmp(buffer, rb_keyword.keyword, strlen(rb_keyword.keyword)) == 0)
		{
			buffer += strlen(rb_keyword.keyword);
			return rb_keyword.value;
		}
	}

	return rb_utils::rb_not_important;
}

