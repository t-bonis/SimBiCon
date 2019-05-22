#include "Articulated_figure.h"
#include "RBUtils.h"
#include "Utils/src/Utils.h"
#include "Core/src/Reduced_character_state.h"
#include "Core/src/SimGlobals.h"
#include "Custom_joint.h"
#include "BallInSocketJoint.h"
#include "HingeJoint.h"
#include "UniversalJoint.h"

Articulated_figure::Articulated_figure(const Articulated_figure& other)
{
	//Create new articulated_rigid_body based on rigid_body properties
	//without links
	for (auto& arb : other.get_arbs())
	{
		m_articulated_rigid_bodies.push_back(std::make_shared<Articulated_rigid_body>(*arb));
	}

	//Set root
	m_root = dynamic_cast<Articulated_rigid_body*>(get_arb_by_name(other.get_root()->get_name()));

	//Set left and right sides
	for (auto& arb : m_articulated_rigid_bodies)
	{
		if (arb->get_name()[0] == 'l')
		{
			m_left_side_articulated_rigid_bodies.push_back(arb.get());
		}
		else if (arb->get_name()[0] == 'r')
		{
			m_right_side_articulated_rigid_bodies.push_back(arb.get());
		}
	}

	//Create the joints based on existing joints and set link between ARBs and joints
	for (auto& joint : other.m_joints)
	{
		switch (joint->get_type())
		{
		case Joint::ball_in_socket_os:
		case Joint::hinge_os:
		case Joint::stiff_os:
		{
			auto tempJoint = std::make_shared<Custom_joint>(*dynamic_cast<Custom_joint*>(joint.get()));
			m_joints.push_back(tempJoint);
			tempJoint->set_af_id(m_joints.size() - 1);

			tempJoint->set_child_arb(*get_arb_by_name(joint->get_child_arb()->get_name()));
			tempJoint->get_child_arb()->set_af_parent(*this);
			tempJoint->get_child_arb()->set_parent_joint(*tempJoint);

			tempJoint->set_parent_arb(*get_arb_by_name(joint->get_parent_arb()->get_name()));
			tempJoint->get_parent_arb()->set_af_parent(*this);
			tempJoint->get_parent_arb()->add_child_joint(*tempJoint);

			break;
		}
		case Joint::ball_in_socket:
		{
			auto tempJoint = std::make_shared<BallInSocketJoint>(*dynamic_cast<BallInSocketJoint*>(joint.get()));
			m_joints.push_back(tempJoint);
			for (auto& a_ARB : m_articulated_rigid_bodies)
			{
				if (a_ARB->get_name() == joint->get_child_arb()->get_name())
				{
					tempJoint->set_child_arb(*a_ARB);
					break;
				}
			}
			tempJoint->get_child_arb()->set_af_parent(*this);
			tempJoint->get_child_arb()->set_parent_joint(*tempJoint);

			for (auto& a_ARB : m_articulated_rigid_bodies)
			{
				if (a_ARB->get_name() == joint->get_parent_arb()->get_name())
				{
					tempJoint->set_parent_arb(*a_ARB);
					break;
				}
			}
			tempJoint->get_parent_arb()->set_af_parent(*this);
			tempJoint->get_parent_arb()->add_child_joint(*tempJoint);

			break;
		}
		case Joint::hinge:
		{
			auto tempJoint = std::make_shared<HingeJoint>(*dynamic_cast<HingeJoint*>(joint.get()));
			m_joints.push_back(tempJoint);
			for (auto& a_ARB : m_articulated_rigid_bodies)
			{
				if (a_ARB->get_name() == joint->get_child_arb()->get_name())
				{
					tempJoint->set_child_arb(*a_ARB);
					break;
				}
			}
			tempJoint->get_child_arb()->set_af_parent(*this);
			tempJoint->get_child_arb()->set_parent_joint(*tempJoint);

			for (auto& a_ARB : m_articulated_rigid_bodies)
			{
				if (a_ARB->get_name() == joint->get_parent_arb()->get_name())
				{
					tempJoint->set_parent_arb(*a_ARB);
					break;
				}
			}
			tempJoint->get_parent_arb()->set_af_parent(*this);
			tempJoint->get_parent_arb()->add_child_joint(*tempJoint);

			break;
		}
		case Joint::universal:
		{
			auto tempJoint = std::make_shared<UniversalJoint>(*dynamic_cast<UniversalJoint*>(joint.get()));
			m_joints.push_back(tempJoint);
			for (auto& a_ARB : m_articulated_rigid_bodies)
			{
				if (a_ARB->get_name() == joint->get_child_arb()->get_name())
				{
					tempJoint->set_child_arb(*a_ARB);
					break;
				}
			}
			tempJoint->get_child_arb()->set_af_parent(*this);
			tempJoint->get_child_arb()->set_parent_joint(*tempJoint);

			for (auto& a_ARB : m_articulated_rigid_bodies)
			{
				if (a_ARB->get_name() == joint->get_parent_arb()->get_name())
				{
					tempJoint->set_parent_arb(*a_ARB);
					break;
				}
			}
			tempJoint->get_parent_arb()->set_af_parent(*this);
			tempJoint->get_parent_arb()->add_child_joint(*tempJoint);
			break;
		}

		default:
			throw std::logic_error("Joint type unknown");
		}
	}

	for (auto& other_muscle : other.get_muscles())
	{
		auto muscle = std::make_shared<Muscle>();

		muscle->clone_properties(*other_muscle);

		for (auto& attachment_point : other_muscle->get_via_points())
		{
			auto tempViaPoint = std::make_shared<Attachment_point>();
			for (auto& a_ARB : m_articulated_rigid_bodies)
			{
				if (a_ARB->get_name() == attachment_point->get_body()->get_name())
				{
					tempViaPoint->set_body(*a_ARB);
					break;
				}
			}
			tempViaPoint->setLocalOffsetPoint(*attachment_point->get_local_offset_point_ptr());
			muscle->add_via_point(tempViaPoint);
		}

		muscle->set_mtu(std::make_shared<MuscleTendonUnit>(*other_muscle->get_mtu()));

		for (auto& via_point : muscle->get_via_points())
		{
			via_point->get_body()->link_muscle(*muscle);
		}
		muscle->init_fields();
		m_muscles.push_back(muscle);
		muscle->set_af_id(m_muscles.size() - 1);
	}

	m_mass = other.m_mass;

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

Articulated_figure::~Articulated_figure()
{
}

std::shared_ptr<Muscle> Articulated_figure::get_muscle_by_name(const std::string& a_name)
{
	for (auto& muscle : m_muscles)
	{
		if (muscle->m_name == a_name)
		{
			return muscle;
		}
	}
	return nullptr;
}

void Articulated_figure::fix_joint_constraints_parent_to_child(bool fixOrientations, bool fixVelocities) const
{
	if (!m_root)
	{
		return;
	}

	for (auto& m_child_joint : m_root->get_children_joints())
	{
		m_child_joint->fix_joint_constraints_parent_to_child(fixOrientations, fixVelocities, true);
	}
}


std::shared_ptr<Joint> Articulated_figure::get_joint_by_name(const std::string& a_name) const
{
	for (auto& joint : m_joints)
	{
		if (joint->get_name() == a_name)
		{
			return joint;
		}
	}
	throw std::logic_error("Joint unknown !");
}

size_t Articulated_figure::get_object_id(const std::string& a_name)
{
	for (auto& joint : m_joints)
	{
		if (joint->get_name() == a_name)
		{
			return joint->get_af_id();
		}
	}
	for (auto& arb : m_articulated_rigid_bodies)
	{
		if (arb->get_name() == a_name)
		{
			return arb->get_af_id();
		}
	}
	throw std::logic_error("Object unknown !");
}

Articulated_rigid_body* Articulated_figure::get_arb_by_name(const std::string& a_name)
{
	for (auto& arb : m_articulated_rigid_bodies)
	{
		if (a_name == arb->get_name())
			return arb.get();
	}
	throw std::logic_error("Articulated rigid body '" + a_name + "' unknown !");
}

std::shared_ptr<Joint> Articulated_figure::get_joint_by_id(const size_t i) const
{
	for (auto& joint : m_joints)
	{
		if (joint->get_af_id() == i)
			return joint;
	}
	return nullptr;
}

Quaternion Articulated_figure::get_heading() const
{
	return m_root->get_orientation().decomposeRotation(SimGlobals::up);
}

//This method is used to compute the total mass of the articulated figure.
void Articulated_figure::compute_articulated_figure_mass()
{
	double curMass = m_root->get_mass();
	double totalMass = curMass;

	for (auto& joint : m_joints)
	{
		curMass = joint->get_child_arb()->get_mass();
		totalMass += curMass;
	}

	m_mass = totalMass;
}

double Articulated_figure::get_mass() const
{
	return m_mass;
}

std::string Articulated_figure::get_name() const
{
	return m_name;
}

void Articulated_figure::set_root(Articulated_rigid_body& a_root)
{
	m_root = &a_root;
}

std::vector<std::shared_ptr<Articulated_rigid_body>> Articulated_figure::get_arbs() const
{
	return m_articulated_rigid_bodies;
}


std::vector<double> Articulated_figure::get_state()
{
	auto state = std::vector<double>();
	get_state(state);
	return state;
}

//this method is used to read the reduced state of the character from the file, into the array passed in as a parameter. The
//state of the character is not modified
void Articulated_figure::read_reduced_state_from_file(const std::string& f_name, std::vector<double>& state) const
{
	FILE* fp = std::fopen(f_name.c_str(), "r");

	double temp1, temp2, temp3, temp4;

	char line[100];

	//Position
	readValidLine(line, fp);
	sscanf_s(line, "%lf %lf %lf", &temp1, &temp2, &temp3);
	state.push_back(temp1);
	state.push_back(temp2);
	state.push_back(temp3);
	//Orientation
	readValidLine(line, fp);
	sscanf_s(line, "%lf %lf %lf %lf", &temp1, &temp2, &temp3, &temp4);
	state.push_back(temp1);
	state.push_back(temp2);
	state.push_back(temp3);
	state.push_back(temp4);
	//Velocity
	readValidLine(line, fp);
	sscanf_s(line, "%lf %lf %lf", &temp1, &temp2, &temp3);
	state.push_back(temp1);
	state.push_back(temp2);
	state.push_back(temp3);
	//AngularVelocity
	readValidLine(line, fp);
	sscanf_s(line, "%lf %lf %lf", &temp1, &temp2, &temp3);
	state.push_back(temp1);
	state.push_back(temp2);
	state.push_back(temp3);

	for (uint i = 0; i < m_joints.size(); i++)
	{
		//Relative Orientation
		readValidLine(line, fp);
		sscanf_s(line, "%lf %lf %lf %lf", &temp1, &temp2, &temp3, &temp4);
		state.push_back(temp1);
		state.push_back(temp2);
		state.push_back(temp3);
		state.push_back(temp4);
		//Relative Angular Velocity
		readValidLine(line, fp);
		sscanf_s(line, "%lf %lf %lf", &temp1, &temp2, &temp3);
		state.push_back(temp1);
		state.push_back(temp2);
		state.push_back(temp3);
	}
	fclose(fp);
}


void Articulated_figure::get_state(std::vector<double>& state)
{
	state.clear();
	//we'll push the root's state information - ugly code....
	state.push_back(m_root->get_cm_position().x);
	state.push_back(m_root->get_cm_position().y);
	state.push_back(m_root->get_cm_position().z);

	state.push_back(m_root->get_orientation().s);
	state.push_back(m_root->get_orientation().v.x);
	state.push_back(m_root->get_orientation().v.y);
	state.push_back(m_root->get_orientation().v.z);

	state.push_back(m_root->get_cm_velocity().x);
	state.push_back(m_root->get_cm_velocity().y);
	state.push_back(m_root->get_cm_velocity().z);

	state.push_back(m_root->get_angular_velocity().x);
	state.push_back(m_root->get_angular_velocity().y);
	state.push_back(m_root->get_angular_velocity().z);

	//now each joint introduces one more rigid body, so we'll only record its state relative to its parent.
	//we are assuming here that each joint is resolved!!!
	Quaternion qRel, qRel_2;
	Vector3d wRel;

	for (auto joint : m_joints)
	{
		joint->compute_relative_orientation(qRel);

		state.push_back(qRel.s);
		state.push_back(qRel.v.x);
		state.push_back(qRel.v.y);
		state.push_back(qRel.v.z);

		joint->compute_relative_angular_velocity(wRel);
		state.push_back(wRel.x);
		state.push_back(wRel.y);
		state.push_back(wRel.z);
	}

	for (auto& muscle : m_muscles)
	{
		double a = muscle->previousActivations.get_newest();
		state.push_back(a);
		state.push_back(a);
	}
}

void Articulated_figure::set_state(const std::vector<double>& state, int start)
{
	Reduced_character_state rs(state, start);

	//kinda ugly code....
	m_root->set_cm_position(rs.get_root_position());
	m_root->set_orientation(rs.get_root_orientation());
	m_root->set_cm_velocity(rs.get_root_velocity());
	m_root->set_angular_velocity(rs.get_root_angular_velocity());

	Vector3d r;
	Vector3d d;
	Vector3d vRel;

	for (uint j = 0; j < m_joints.size(); j++)
	{
		Quaternion qRel = rs.get_joint_relative_orientation(j);
		Vector3d wRel = rs.get_joint_relative_ang_velocity(j);
		//transform the relative angular velocity to world coordinates
		wRel = m_joints[j]->get_parent_arb()->get_vector_world_coordinates(wRel);

		//now that we have this information, we need to restore the state of the rigid body.
		//set the proper orientation
		m_joints[j]->get_child_arb()->set_orientation(m_joints[j]->get_parent_arb()->get_orientation() * qRel);

		//and the proper angular velocity
		m_joints[j]->get_child_arb()->
			set_angular_velocity(m_joints[j]->get_parent_arb()->get_angular_velocity() + wRel);

		//and now set the linear position and velocity
		m_joints[j]->fix_joint_constraints_parent_to_child(false, true, false);
	}
}

void Articulated_figure::set_default_state()
{
	Reduced_character_state rs;

	rs.set_root_orientation(Quaternion(1, 0, 0, 0));
	rs.set_root_angular_velocity(Vector3d());
	rs.set_root_position(Vector3d());
	rs.set_root_velocity(Vector3d());

	for (uint j = 0; j < m_joints.size(); j++)
	{
		rs.set_joint_relative_orientation(Quaternion(1, 0, 0, 0), j);
		rs.set_joint_relative_ang_velocity(Vector3d(), j);
	}

	set_state(rs.get_state(), 0);
}

void Articulated_figure::set_arbs_lines_state(const std::vector<double>& state, std::vector<Articulated_rigid_body*> arbs_involved)
{

	Reduced_character_state rs(state, 0);

	//Position and orientation of fixed_arb will not change
	//Then update position of other bodies based on joints angle

	Vector3d r;
	Vector3d d;
	Vector3d vRel;
	const auto nb_arb_involved = arbs_involved.size();

	if(nb_arb_involved <= 1)
	{
		return;
	}

	for (size_t i = 0; i < nb_arb_involved - 1; i++)
	{
		const auto joint = arbs_involved[i]->get_joint_with(*arbs_involved[i + 1]);
		const auto relation = arbs_involved[i]->get_relation_with(*joint);
		Quaternion qRel = rs.get_joint_relative_orientation(joint->get_af_id());
		auto previous_arb_orientation = arbs_involved[i]->get_orientation();

		switch (relation)
		{
		case Articulated_rigid_body::parent:
		{
			auto temp = previous_arb_orientation * qRel.get_inverse();
			arbs_involved[i + 1]->set_orientation(temp);
			auto rc = arbs_involved[i]->get_vector_world_coordinates(Vector3d(joint->get_joint_position_in_child(), Point3d(0, 0, 0)));
			auto rp = arbs_involved[i + 1]->get_vector_world_coordinates(Vector3d(joint->get_joint_position_in_parent(), Point3d(0, 0, 0)));
			arbs_involved[i + 1]->set_cm_position(arbs_involved[i]->get_cm_position() + (rp - rc));

			break;
		}
		case Articulated_rigid_body::child:
		{
			auto temp = previous_arb_orientation * qRel;
			arbs_involved[i + 1]->set_orientation(temp);
			auto rc = arbs_involved[i + 1]->get_vector_world_coordinates(Vector3d(joint->get_joint_position_in_child(), Point3d(0, 0, 0)));
			auto rp = arbs_involved[i]->get_vector_world_coordinates(Vector3d(joint->get_joint_position_in_parent(), Point3d(0, 0, 0)));
			arbs_involved[i + 1]->set_cm_position(arbs_involved[i]->get_cm_position() + (rc - rp));
			break;
		}
		default:;
		}
	}
}


size_t Articulated_figure::get_state_dimension() const
{
	//13 for the root, and 7 for every other body (and each body is introduced by a joint).
	return 13 + 7 * m_joints.size();
}


Vector3d Articulated_figure::get_com() const
{
	Vector3d com = Vector3d(m_root->get_cm_position()) * m_root->get_mass();
	double curMass = m_root->get_mass();
	double totalMass = curMass;
	for (const auto& joint : m_joints)
	{
		curMass = joint->get_child_arb()->get_mass();
		totalMass += curMass;
		com.add_scaled_vector(joint->get_child_arb()->get_cm_position(), curMass);
	}

	com /= totalMass;

	return com;
}



Vector3d Articulated_figure::get_com_velocity() const
{
	Vector3d com_vel = Vector3d(m_root->get_cm_velocity()) * m_root->get_mass();
	double curMass = m_root->get_mass();
	double totalMass = curMass;
	for (const auto& joint : m_joints)
	{
		curMass = joint->get_child_arb()->get_mass();
		totalMass += curMass;
		com_vel.add_scaled_vector(joint->get_child_arb()->get_cm_velocity(), curMass);
	}

	com_vel /= totalMass;

	return com_vel;
}



std::vector<Articulated_rigid_body*> Articulated_figure::list_arbs_involved_form_anchor_to_target(
	Articulated_rigid_body& anchor, Articulated_rigid_body& target) const
{
	std::vector<Articulated_rigid_body*> output1, output2;
	output1.push_back(&anchor);

	//First search from anchor to root 
	bool find = false;
	for (auto& rigid_body : anchor.get_parent_arbs())
	{
		output1.push_back(rigid_body);
		if (rigid_body == &target)
		{
			find = true;
			break;
		}
	}

	if (find)
	{
		return output1;
	}

	// If not find, then search from target to anchor
	output2.push_back(&target);
	for (auto& rigid_body : target.get_parent_arbs())
	{
		output2.push_back(rigid_body);
		if (rigid_body == &anchor)
		{
			find = true;
			break;
		}
	}
	if (find)
	{
		std::reverse(std::begin(output2), std::end(output2));
		return output2;
	}



	//if not find in the last one return anchor to root + (root) to target
	output1.pop_back();
	auto  output3 = output1;
	output3.insert(output3.end(), output2.begin(), output2.end());

	return output3;

}

//This method is used to read the reduced state of the character from the file
void Articulated_figure::load_reduced_state_from_file(const std::string& fName)
{
	auto state = std::vector<double>();
	read_reduced_state_from_file(fName, state);

	set_state(state);

	//update the muscle geometry based on the new character position
	for (auto& muscle : m_muscles)
	{
		muscle->update_fields();
	}
}

void Articulated_figure::load_open_sim_reduced_state_from_file(const std::string& fName)
{
	auto state = std::vector<double>();
	read_reduced_state_from_file(fName, state);

	set_state(state);

	//update the muscle geometry based on the new character position
	for (auto& muscle : m_muscles)
	{
		muscle->update_fields();
	}
}

//this method is used to write the reduced state of the character to a file
void Articulated_figure::save_reduced_state_to_file(char* fName, std::vector<double>& state)
{
	if (fName == nullptr)
		throw std::logic_error("cannot write to a file whose name is NULL!");

	FILE* fp;
	const auto err = fopen_s(&fp, fName, "w");
	if (err != 0 || fp == nullptr)
		throw std::logic_error("cannot open the file " + std::string(fName) + " for writing...");
	//header
	fprintf(fp, "# order is:# Position\n# Orientation\n# Velocity\n# AngularVelocity\n\n");
	fprintf(fp, "# for each joint:\n# Relative Orientation\n# Relative Angular Velocity\n\n");
	fprintf(fp, "# for each muscle:\n# Initial activation\n# Initial excitation\n#----------------\n\n");
	//root
	fprintf(fp, "%lf %lf %lf\n", state[0], state[1], state[2]);
	fprintf(fp, "%lf %lf %lf %lf\n", state[3], state[4], state[5], state[6]);
	fprintf(fp, "%lf %lf %lf\n", state[7], state[8], state[9]);
	fprintf(fp, "%lf %lf %lf\n\n", state[10], state[11], state[12]);

	//joints
	size_t offset = 13;
	uint nb = 7;
	for (uint i = 0; i < m_joints.size(); i++)
	{
		fprintf(fp, "# %s\n", m_joints[i]->get_name().c_str());
		fprintf(fp, "%lf %lf %lf %lf\n", state[offset + nb * i + 0], state[offset + nb * i + 1],
			state[offset + nb * i + 2], state[offset + nb * i + 3]);
		fprintf(fp, "%lf %lf %lf\n", state[offset + nb * i + 4], state[offset + nb * i + 5],
			state[offset + nb * i + 6]);
		fprintf(fp, "\n");
	}

	//muscles
	offset += m_joints.size() * nb;
	nb = 2;
	int i = 0;
	for (auto& muscle : m_muscles)
	{
		fprintf(fp, "# %s\n", muscle->m_name.c_str());
		fprintf(fp, "%lf\n", state[offset + nb * i + 0]);
		fprintf(fp, "%lf\n", state[offset + nb * i + 1]);
		fprintf(fp, "\n");
		i++;
	}

	fclose(fp);
}

//this method is used to write the reduced state of the character to the file
void Articulated_figure::save_reduced_state_to_file(char* fName)
{
	auto state = std::vector<double>();
	get_state(state);

	save_reduced_state_to_file(fName, state);
}

//write the details of the character's muscles to a rbs file
void Articulated_figure::save_muscles_to_file(char* fName)
{
	if (fName == nullptr)
		throw std::logic_error("cannot write to a file whose name is NULL!");

	FILE* fp;
	const auto err = fopen_s(&fp, fName, "w");
	if (err != 0 || fp == nullptr)
		throw std::logic_error("cannot open the file " + std::string(fName) + " for writing...");
	//header
	fprintf(fp, "##muscles\n");

	//muscles
	for (auto& muscle : m_muscles)
	{
		fprintf(fp, "Muscle\n");
		fprintf(fp, "\tname %s \n", muscle->get_name().c_str());
		fprintf(fp, "\tmaxForce %f\n", muscle->get_max_iso_force());
		fprintf(fp, "\toptimalLength %lf\n", muscle->get_optimal_fiber_length());
		fprintf(fp, "\tslackLength %lf\n", muscle->get_tendon_slack_length());
		fprintf(fp, "\tdelay %f\n", muscle->get_neuronal_delay());
		for (auto& via_point : muscle->get_via_points())
		{
			fprintf(fp, "\tA_RigidBody %s\n", via_point->get_body()->get_name().c_str());
			const auto p = via_point->get_local_offset_point_ptr();
			fprintf(fp, "\tposition %f %f %f\n", p->x, p->y, p->z);
		}

		fprintf(fp, "/Muscle\n\n");

		fclose(fp);
	}
}

