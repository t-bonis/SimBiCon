#include "Character.h"
#include "Reduced_character_state.h"
#include "Ode_world.h"
#include "SimGlobals.h"
#include "SimBiCon.h"
#include "RBUtils.h"
#include "UniversalJoint.h"
#include "HingeJoint.h"
#include "BallInSocketJoint.h"
#include "Custom_joint.h"
#include "Force_utilitary.h"


Character::Character(FILE* f, Abstract_rb_engine& physique_engine)
{
	m_physique_engine = &physique_engine;

	if (f == nullptr)
		throw std::logic_error("Invalid file pointer.");
	//have a temporary buffer used to read the file line by line...
	char buffer[200];
	char temp_name[100];
	std::shared_ptr<Joint> temp_joint;
	std::shared_ptr<Muscle> muscle;
	m_muscles.clear();
	auto fixed = false;

	//this is where it happens.
	while (!feof(f))
	{
		//get a line from the file...
		fgets(buffer, 200, f);
		if (strlen(buffer) > 195)
			throw std::logic_error("The input file contains a line that is longer than ~200 characters - not allowed");
		auto line = lTrim(buffer);
		const auto line_type = getRBLineType(line);
		switch (line_type)
		{
		case rb_utils::rb_root:
			sscanf(line, "%s", temp_name);
			m_root = dynamic_cast<Articulated_rigid_body*>(m_physique_engine->get_rb_by_name(temp_name).get());
			if (m_root == nullptr)
				throw std::logic_error("The articulated rigid body  " + std::string(temp_name) + " cannot be found!");
			break;
		case rb_utils::rb_joint_type_universal:
			temp_joint = std::make_shared<UniversalJoint>();
			temp_joint->read_axes(line);
			temp_joint->load_from_file(f, physique_engine);
			temp_joint->get_child_arb()->set_af_parent(*this);
			temp_joint->get_parent_arb()->set_af_parent(*this);
			break;
		case rb_utils::rb_joint_type_hinge:
			temp_joint = std::make_shared<HingeJoint>(line);
			temp_joint->load_from_file(f, physique_engine);
			temp_joint->get_child_arb()->set_af_parent(*this);
			temp_joint->get_parent_arb()->set_af_parent(*this);
			break;
		case rb_utils::rb_joint_type_ball_in_socket:
			temp_joint = std::make_shared<BallInSocketJoint>(line);
			temp_joint->load_from_file(f, physique_engine);
			temp_joint->get_child_arb()->set_af_parent(*this);
			temp_joint->get_parent_arb()->set_af_parent(*this);
			break;
		case rb_utils::rb_joint_type_stiff_joint:
			temp_joint = std::make_shared<StiffJoint>();
			temp_joint->load_from_file(f, physique_engine);
			temp_joint->get_child_arb()->set_af_parent(*this);
			temp_joint->get_parent_arb()->set_af_parent(*this);
			break;
		case rb_utils::rb_muscle:
			//before initializing the first muscle, make sure that all the joints have been initialized,
			//so that the rigid bodies are all placed properly in the world frame
			if (!fixed)
			{
				this->fix_joint_constraints_parent_to_child(false, true);
				fixed = true;
			}
			muscle = std::make_shared<Muscle>();
			muscle->load_from_file(f, physique_engine);
			m_muscles.push_back(muscle);
			muscle->set_af_id(m_muscles.size() - 1);
			break;


		case rb_utils::rb_end_articulated_figure:

			//make sure that the root does not have a parent, otherwise we'll end up with loops in the articulated figure
			if (m_root->get_parent_joint() != nullptr)
				throw std::logic_error("The root of the articulated figure is not allowed to have a parent!");
			fill_easy_access_pointers();
			if (!fixed)
			{
				this->fix_joint_constraints_parent_to_child(false, true);
			}
			return;

		case rb_utils::rb_not_important:
			break;
		default:
			throw std::logic_error(
				"Incorrect articulated body input file: " + std::string(buffer) + " - unexpected line. AF");
		}
	}
	throw std::logic_error("Incorrect articulated body input file! No /ArticulatedFigure found AF");
}

Character::Character(const Global& input, Abstract_rb_engine& physique_engine)
{
	m_physique_engine = &physique_engine;

	const auto first_rb = m_physique_engine->get_rb_by_name(input.rigidBodies[0].name);

	if (const auto root = std::dynamic_pointer_cast<Articulated_rigid_body>(first_rb))
	{
		set_name(input.modelName);
		set_root(*root);

		for (auto& joint : input.joints)
		{
			load_joint_from_struct(joint);
		}

		for (auto& muscle : input.muscles)
		{
			load_muscle_from_struct(muscle);
		}

		fill_easy_access_pointers();
		compute_articulated_figure_mass();
	}
	else
	{
		throw std::logic_error("No valid root found");
	}
}

Character::Character(const Articulated_figure& other, Abstract_rb_engine& physique_engine)
{
	m_physique_engine = &physique_engine;

	const auto first_rb = m_physique_engine->get_rb_by_name(other.get_root()->get_name());

	if (const auto root = std::dynamic_pointer_cast<Articulated_rigid_body>(first_rb))
	{
		set_name(other.get_name());
		set_root(*root);

		for (auto& other_joint : other.get_joints())
		{
			load_joint_from_other(other_joint);
		}

		for (auto& other_muscle : other.get_muscles())
		{
			load_muscle_from_other(other_muscle);
		}

		fill_easy_access_pointers();
		compute_articulated_figure_mass();
	}
	else
	{
		throw std::logic_error("No valid root found");
	}
}

void Character::fill_easy_access_pointers()
{
	if (m_root)
	{
		m_articulated_rigid_bodies.push_back(std::dynamic_pointer_cast<Articulated_rigid_body>(m_physique_engine->get_rb_by_name(m_root->get_name())));
		m_root->set_af_id(m_articulated_rigid_bodies.size() - 1);
		auto children_arbs = m_root->get_children_arbs(true);
		for (const auto& arb : children_arbs)
		{
			m_articulated_rigid_bodies.push_back(std::dynamic_pointer_cast<Articulated_rigid_body>(m_physique_engine->get_rb_by_name(arb->get_name())));
			arb->set_af_id(m_articulated_rigid_bodies.size() - 1);
		}
	}
	else
	{
		throw std::logic_error("No root defined, impossible to fill 'm_articulated_rigid_bodies'");
	}

	for (auto& arb : m_articulated_rigid_bodies)
	{
		if (arb->get_name() == "toes_l")
		{
			m_left_toes = arb.get();
		}
		if (arb->get_name() == "toes_r")
		{
			m_right_toes = arb.get();
		}
		if (arb->get_name() == "calcn_l")
		{
			m_left_foot = arb.get();
		}
		if (arb->get_name() == "calcn_r")
		{
			m_right_foot = arb.get();
		}
		if (arb->get_name().back() == 'l')
		{
			m_left_side_articulated_rigid_bodies.push_back(arb.get());
		}
		else if (arb->get_name().back() == 'r')
		{
			m_right_side_articulated_rigid_bodies.push_back(arb.get());
		}
	}

	for (auto& joint : m_joints)
	{
		if (joint->get_name() == "hip_l")
		{
			m_left_hip = joint.get();
		}
		if (joint->get_name() == "hip_r")
		{
			m_right_hip = joint.get();
		}
	}



}

void Character::add_contact_points(Contact_point& contact_point)
{
	m_contact_points.push_back(&contact_point);
}


void Character::load_joint_from_struct(const Joint_from_xml& a_joint)
{
	enum opensim_joint_type { custom, pin, weld };

	switch (a_joint.type)
	{
	case custom:
	{
		auto temp_joint = std::make_shared<Custom_joint>();
		m_joints.push_back(temp_joint);
		temp_joint->set_af_id(m_joints.size() - 1);

		temp_joint->load_coordinates_from_struct(a_joint);
		temp_joint->read_axis_from_struct(a_joint);

		temp_joint->set_name(a_joint.name);

		temp_joint->set_child_arb(*dynamic_cast<Articulated_rigid_body*>(m_physique_engine->get_rb_by_name(
			a_joint.child_body_name).get()));
		temp_joint->set_joint_position_in_child(-a_joint.child_RB_pos[0], -a_joint.child_RB_pos[1],
			-a_joint.child_RB_pos[2]);
		temp_joint.get()->get_child_arb()->set_parent_joint(*temp_joint);
		temp_joint->get_child_arb()->set_af_parent(*this);


		temp_joint->set_parent_arb(*dynamic_cast<Articulated_rigid_body*>(m_physique_engine->get_rb_by_name(
			a_joint.parent_body_name).get()
			));
		temp_joint->set_joint_position_in_parent(-a_joint.parent_RB_Pos[0], -a_joint.parent_RB_Pos[1],
			-a_joint.parent_RB_Pos[2]);
		temp_joint->get_parent_arb()->add_child_joint(*temp_joint);
		temp_joint->get_parent_arb()->set_af_parent(*this);
		break;
	}
	default:
		throw std::logic_error("Joint type unknown");
	}
}

void Character::load_joint_from_other(const std::shared_ptr<Joint>& other)
{
	switch (other->get_type()) {
	case Joint::stiff_os:
	case Joint::hinge_os:
	case Joint::ball_in_socket_os:
	{
		auto temp_joint = std::make_shared<Custom_joint>(*std::dynamic_pointer_cast<Custom_joint>(other));

		temp_joint->set_child_arb(*std::dynamic_pointer_cast<Articulated_rigid_body>(m_physique_engine->get_rb_by_name(
			other->get_child_arb()->get_name())));
		temp_joint->get_child_arb()->set_parent_joint(*temp_joint);
		temp_joint->get_child_arb()->set_af_parent(*this);


		temp_joint->set_parent_arb(*std::dynamic_pointer_cast<Articulated_rigid_body>(m_physique_engine->get_rb_by_name(
			other->get_parent_arb()->get_name())));
		temp_joint->get_parent_arb()->add_child_joint(*temp_joint);
		temp_joint->get_parent_arb()->set_af_parent(*this);

		m_joints.push_back(temp_joint);
		temp_joint->set_af_id(m_joints.size() - 1);
		break;
	}
	default:;
	}
}

void Character::load_muscle_from_struct(const Muscle_from_xml& input)
{
	auto temp_muscle = std::make_shared<Muscle>();

	temp_muscle->m_name = input.name;

	for (auto& attachment_point : input.viaPoints)
	{
		std::shared_ptr<Attachment_point> tempViaPoint = std::make_shared<Attachment_point>();
		tempViaPoint->set_body(
			*dynamic_cast<Articulated_rigid_body*>(m_physique_engine->get_rb_by_name(attachment_point.refBody).get()));
		tempViaPoint->setLocalOffsetPoint(Point3d(attachment_point.position[0], attachment_point.position[1],
			attachment_point.position[2]));
		temp_muscle->add_via_point(tempViaPoint);
	}

	temp_muscle->set_mtu(std::make_shared<MuscleTendonUnit>(input.maxForce, input.optimalLength, input.slackLength));

	for (auto& via_point : temp_muscle->get_via_points())
	{
		via_point->get_body()->link_muscle(*temp_muscle);
	}
	temp_muscle->init_fields();
	m_muscles.push_back(temp_muscle);
	temp_muscle->set_af_id(m_muscles.size() - 1);
}


void Character::load_muscle_from_other(const std::shared_ptr<Muscle>& other)
{
	auto temp_muscle = std::make_shared<Muscle>(*other, *m_physique_engine);


	//for (auto& attachment_point : input.viaPoints)
	//{
	//	std::shared_ptr<Attachment_point> tempViaPoint = std::make_shared<Attachment_point>();
	//	tempViaPoint->set_body(
	//		*dynamic_cast<Articulated_rigid_body*>(m_physique_engine->get_rb_by_name(attachment_point.refBody).get()));
	//	tempViaPoint->setLocalOffsetPoint(Point3d(attachment_point.position[0], attachment_point.position[1],
	//		attachment_point.position[2]));
	//	temp_muscle->add_via_point(tempViaPoint);
	//}

	//temp_muscle->set_mtu(std::make_shared<MuscleTendonUnit>(input.maxForce, input.optimalLength, input.slackLength));

	for (auto& via_point : temp_muscle->get_via_points())
	{
		via_point->get_body()->link_muscle(*temp_muscle);
	}
	temp_muscle->init_fields();


	m_muscles.push_back(temp_muscle);
	temp_muscle->set_af_id(m_muscles.size() - 1);
}


Vector3d Character::get_cop() const
{
	Vector3d output{ 0,0,0 };
	double sum_force{ 0 };
	for (auto& contact_point : m_contact_points)
	{
		if (contact_point->rb1->get_name() == "ground")
		{
			const auto pressure = (-contact_point->f).dotProductWith(SimGlobals::up);
			output += Vector3d(contact_point->cp)*pressure;
			sum_force += pressure;
		}
		else if (contact_point->rb2->get_name() == "ground")
		{
			const auto pressure = contact_point->f.dotProductWith(SimGlobals::up);
			output += Vector3d(contact_point->cp)*pressure;
			sum_force += pressure;
		}
	}
	if (sum_force)
	{
		output /= sum_force;
	}

	return output;
}

void Character::apply_virtual_force(Articulated_rigid_body& target, Articulated_rigid_body& anchor, const Vector3d& a_force) const
{
	using namespace boost::numeric::ublas;
	matrix<double> force(3, 1, 0);
	force(0, 0) = a_force.x;
	force(1, 0) = a_force.y;
	force(2, 0) = a_force.z;

	auto arbs_involved = list_arbs_involved_form_anchor_to_target(anchor, target);

	const auto jacobian = compute_jacobian_pos(arbs_involved);
	const auto resulting_torque = prod(trans(force), jacobian);

	int j = 0;
	for (unsigned int i = 0; i < arbs_involved.size() - 1; i++)
	{
		auto joint = arbs_involved[i]->get_joint_with(*arbs_involved[i + 1]);
		Vector3d temp_additional_torque;
		for (const auto& axis : joint->get_rotation_axes_in_local_coords())
		{
			temp_additional_torque += Vector3d(resulting_torque(0, j) * axis.x, resulting_torque(0, j) * axis.y, resulting_torque(0, j) * axis.z);
			j++;
		}
		//ensure to apply the torque in the right direction
		if (arbs_involved[i]->is_parent_of(*arbs_involved[i + 1]))
		{
			temp_additional_torque = joint->get_child_arb()->get_vector_world_coordinates(-temp_additional_torque);
		}
		else
		{
			temp_additional_torque = joint->get_parent_arb()->get_vector_world_coordinates(temp_additional_torque);
		}
		joint->add_torque(temp_additional_torque);
	}
}

void Character::apply_virtual_torque(Articulated_rigid_body& target, Articulated_rigid_body& anchor, const Vector3d& a_torque) const
{
	using namespace boost::numeric::ublas;
	matrix<double> torque(3, 1, 0);
	torque(0, 0) = a_torque.x;
	torque(1, 0) = a_torque.y;
	torque(2, 0) = a_torque.z;
	auto arbs_involved = list_arbs_involved_form_anchor_to_target(anchor, target);

	const auto jacobian = compute_jacobian_angles(arbs_involved);
	const auto resulting_torque = prod(trans(torque), jacobian);

	int j = 0;
	for (unsigned int i = 0; i < arbs_involved.size() - 1; i++)
	{
		auto joint = arbs_involved[i]->get_joint_with(*arbs_involved[i + 1]);
		Vector3d temp_additional_torque;
		for (const auto& axis : joint->get_rotation_axes_in_local_coords())
		{
			temp_additional_torque += Vector3d(resulting_torque(0, j) * axis.x, resulting_torque(0, j) * axis.y, resulting_torque(0, j) * axis.z);
			j++;
		}
		//ensure to apply the torque in the right direction
		if (arbs_involved[i]->is_parent_of(*arbs_involved[i + 1]))
		{
			temp_additional_torque = joint->get_child_arb()->get_vector_world_coordinates(-temp_additional_torque);
		}
		else
		{
			temp_additional_torque = joint->get_parent_arb()->get_vector_world_coordinates(temp_additional_torque);
		}
		joint->add_torque(temp_additional_torque);
	}
}

void Character::set_full_state(const State& state)
{
	m_body_touched_the_ground = state.body_touched_the_ground;
	m_stance = state.stance;
	for (auto& muscle : m_muscles)
	{
		muscle->update_fields();
		muscle->previousActivations.reset();
	}
}

void Character::get_full_state(State& state) const
{
	state.body_touched_the_ground = m_body_touched_the_ground;
	state.stance = m_stance;
}

boost::numeric::ublas::matrix<double> Character::compute_jacobian_pos(std::vector<Articulated_rigid_body*>& arbs_involved) const
{
	using namespace boost::numeric::ublas;
	matrix<double> jacobian(3, 1, 0);
	auto j = 0;
	for (size_t i = 0; i < arbs_involved.size() - 1; i++)
	{
		//axis of rotation of joint with next in list 
		auto joint = arbs_involved[i]->get_joint_with(*arbs_involved[i + 1]);

		//rotation axis is defined in parent coord
		for (auto& rotation_axis : joint->get_rotation_axes_in_global_coords())
		{
			auto u_rotation_axis = rotation_axis.unit();
			auto relative_pos = (arbs_involved.back()->get_cm_position() - joint->get_pos_in_global_coords());
			const auto result = u_rotation_axis.cross_product_with(relative_pos);
			jacobian.resize(3, j + 1);
			jacobian.insert_element(0, j, result.x);
			jacobian.insert_element(1, j, result.y);
			jacobian.insert_element(2, j, result.z);
			j++;
		}
	}
	return jacobian;
}

boost::numeric::ublas::matrix<double> Character::compute_jacobian_angles(std::vector<Articulated_rigid_body*>& arbs_involved) const
{
	using namespace boost::numeric::ublas;
	matrix<double> jacobian(3, 1, 0);
	auto j = 0;
	for (size_t i = 0; i < arbs_involved.size() - 1; i++)
	{
		//axis of rotation of joint with next in list 
		auto joint = arbs_involved[i]->get_joint_with(*arbs_involved[i + 1]);

		//rotation axis is defined in parent coord
		for (auto& rotation_axis : joint->get_rotation_axes_in_global_coords())
		{
			auto u_rotation_axis = rotation_axis.unit();
			jacobian.resize(3, j + 1);
			jacobian.insert_element(0, j, u_rotation_axis.x);
			jacobian.insert_element(1, j, u_rotation_axis.y);
			jacobian.insert_element(2, j, u_rotation_axis.z);
			j++;
		}
	}
	return jacobian;
}


std::vector<Articulated_rigid_body*> Character::get_anchors()
{
	//gather all rigid body in contact with ground
	std::vector<Articulated_rigid_body*> temp_holder, output;
	for (auto& contact_point : m_contact_points)
	{
		if (contact_point->rb1->get_name() == "ground")
		{
			temp_holder.push_back(reinterpret_cast<Articulated_rigid_body*>(contact_point->rb2));
		}
		else if (contact_point->rb2->get_name() == "ground")
		{
			temp_holder.push_back(reinterpret_cast<Articulated_rigid_body*>(contact_point->rb1));
		}
	}


	//filter the list of rigid body in contact with the ground
	for (auto& arb_1 : temp_holder)
	{
		//check if arb_1 has a parent who touch the ground
		bool has_parent_touching_ground = false;
		for (auto& arb_2 : temp_holder)
		{
			if (arb_2->get_parent_joint())
			{
				if (arb_1 == arb_2->get_parent_joint()->get_parent_arb())
				{
					has_parent_touching_ground = true;
					break;
				}
			}
		}
		if (has_parent_touching_ground)
		{
			continue;
		}

		//check if arb_1 is already in the list
		bool is_already_in = false;
		for (auto& arb_3 : output)
		{
			if (arb_3 == arb_1)
			{
				is_already_in = true;
				break;
			}
		}

		if (!is_already_in)
		{
			output.push_back(arb_1);
		}
	}
	return output;
}

//This method returns the net force on the body rb, acting from the ground
Vector3d Character::get_force_on(Rigid_body* rb) const
{
	Vector3d f_net = Vector3d();
	for (auto& cf : get_contact_points())
	{
		if (cf->rb1 == rb)
			f_net += cf->f;
		if (cf->rb2 == rb)
			f_net -= cf->f;
	}
	return f_net;
}

Vector3d Character::get_force_from_ground(Rigid_body* rb) const
{
	Vector3d f_net = Vector3d();
	for (auto& cf : get_contact_points())
	{
		if (cf->rb1 == rb && cf->rb2->get_name() == "ground")
			f_net += cf->f;
		if (cf->rb2 == rb && cf->rb1->get_name() == "ground")
			f_net -= cf->f;
	}
	return f_net;
}

Vector3d Character::get_torque_from_ground(Rigid_body* rb) const
{
	Vector3d t_net = Vector3d();
	for (auto& cf : get_contact_points())
	{
		if (cf->rb1 == rb && cf->rb2->get_name() == "ground")
			t_net += cf->t1;
		if (cf->rb2 == rb && cf->rb1->get_name() == "ground")
			t_net -= cf->t1;
	}
	return t_net;
}

//This method returns the net force on the side (0 = right and 1 = left)
Vector3d Character::get_force_on(std::vector<Articulated_rigid_body*> bodies) const
{
	Vector3d f_net;
	//we will also look at all children of the foot that is passed in (to take care of toes).
	for (auto& body : bodies)
	{
		f_net += get_force_on(body);
	}
	return f_net;
}

Vector3d Character::get_force_from_ground(std::vector<Articulated_rigid_body*> bodies) const
{
	Vector3d f_net;

	for (auto& body : bodies)
	{
		f_net += get_force_from_ground(body);
	}
	return f_net;
}

Vector3d Character::get_torque_from_ground(std::vector<Articulated_rigid_body*> bodies) const
{
	Vector3d t_net;

	for (auto& body : bodies)
	{
		t_net += get_torque_from_ground(body);
	}
	return t_net;
}

std::vector<Articulated_rigid_body*> Character::get_swing_arbs() const
{

	switch (m_stance) {
	case right:
		return m_left_side_articulated_rigid_bodies;
	case left:
		return m_right_side_articulated_rigid_bodies;
	case both:
	case none:
	default:
		return std::vector<Articulated_rigid_body*>();
	}
}

bool Character::is_right_arb(Articulated_rigid_body* a_arb) const
{
	for( auto& arb : m_right_side_articulated_rigid_bodies)
	{
		if(arb == a_arb)
		{
			return true;
		}
	}
	return false;

}

bool Character::is_left_arb(Articulated_rigid_body* a_arb) const
{
	for( auto& arb : m_left_side_articulated_rigid_bodies)
	{
		if(arb == a_arb)
		{
			return true;
		}
	}
	return false;

}

std::vector<Articulated_rigid_body*> Character::get_stance_arbs() const
{
	switch (m_stance) {
	case right:
		return m_right_side_articulated_rigid_bodies;
	case left:
		return m_left_side_articulated_rigid_bodies;
	case both:
	case none:
	default:
		return std::vector<Articulated_rigid_body*>();
	}
}

Vector3d Character::get_swing_foot_loc() const
{
	std::vector<Articulated_rigid_body*> iterate;
	switch (m_stance) {
	case right:
		iterate = m_right_side_articulated_rigid_bodies;
		break;
	case left:
		iterate = m_left_side_articulated_rigid_bodies;
		break;
	case both:
	case none:
	default:
		iterate = std::vector<Articulated_rigid_body*>();
		break;
	}

	return Vector3d(iterate[3]->get_cm_position());
}

void Character::clear_contact_points()
{
	
	m_contact_points.clear();
	dynamic_cast<Ode_world*>(m_physique_engine)->clear_contact_forces();
}

void Character::get_leg_contact_influence(std::vector<double> &vect_influence)
{
	std::vector<Vector3d> vect_force;
	vect_force.resize(2);

	vect_force[0] = get_force_from_ground(get_left_side_articulated_rigid_bodies());
	vect_force[1] = get_force_from_ground(get_right_side_articulated_rigid_bodies());

	double total_force = 0;
	for (int i = 0; i < vect_force.size(); ++i) {
		total_force += vect_force[i].y;
	}

	//now we get the influence
	vect_influence.resize(2, 0.0);
	if (total_force > 0) {
		for (int i = 0; i < vect_force.size(); ++i) {
			vect_influence[i] = vect_force[i].y / total_force;
		}
	}
}

void Character::get_force_info_on_foot(Rigid_body *rb, Force_struct &heelForce, Force_struct &frontFeetForce, Force_struct &toeForce)
{
	heelForce.F = Vector3d(0, 0, 0);
	frontFeetForce.F = Vector3d(0, 0, 0);
	toeForce.F = Vector3d(0, 0, 0);

	Rigid_body* swing_foot = get_joint_by_name("lAnkle")->get_child_arb();
	if (get_stance() == right) {
		swing_foot = get_joint_by_name("rAnkle")->get_child_arb();
	}

	if (rb == swing_foot) {
		heelForce.F.y += std::abs(_force_swing_foot[0].y);
		heelForce.F.y += std::abs(_force_swing_foot[1].y);
		frontFeetForce.F.y += std::abs(_force_swing_foot[2].y);
		frontFeetForce.F.y += std::abs(_force_swing_foot[3].y);
		toeForce.F.y = std::abs(_force_swing_toes.y);
	}
	else {
		heelForce.F.y += std::abs(_force_stance_foot[0].y);
		heelForce.F.y += std::abs(_force_stance_foot[1].y);
		frontFeetForce.F.y += std::abs(_force_stance_foot[2].y);
		frontFeetForce.F.y += std::abs(_force_stance_foot[3].y);
		toeForce.F.y = std::abs(_force_stance_toes.y);
	}
}

bool Character::continue_simulation()
{
	if(is_body_touched_the_ground())
	{
		return false;
	}
	if(is_legs_touched())
	{
		return false;
	}
	return true;
}