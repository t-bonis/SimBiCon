#include "AbstractRBEngine.h"
#include "Ode_world.h"
#include "RBUtils.h"
#include "Utils/src/Utils.h"

void Abstract_rb_engine::load_af_from_struct(const Global& input)
{
	load_rbs_from_struct(input);

	m_character = std::make_shared<Character>(input, *this);
}

void Abstract_rb_engine::load_af_from_other(const Ode_world& other)
{
	m_character = std::make_shared<Character>(*std::dynamic_pointer_cast<Character>(other.get_character()), *this);

}

void Abstract_rb_engine::load_rbs_from_struct(const Global& input)
{
	for (auto& rigid_body : input.rigidBodies)
	{
		auto temp_rigid_body = std::make_shared<Articulated_rigid_body>();
		temp_rigid_body->load_rb_from_struct(rigid_body);
		m_rigid_bodies.push_back(temp_rigid_body);
	}
}

void Abstract_rb_engine::load_rbs_from_other(const Ode_world& other)
{
	for (auto& rigid_body : other.get_rigid_bodies())
	{
		std::shared_ptr<Rigid_body> temp_rigid_body;
		if (rigid_body->is_articulated())
		{
			temp_rigid_body = std::make_shared<Articulated_rigid_body>(*std::dynamic_pointer_cast<Articulated_rigid_body>(rigid_body));
		}
		else
		{
			temp_rigid_body = std::make_shared<Rigid_body>(*rigid_body);
		}
		m_rigid_bodies.push_back(temp_rigid_body);
	}
}

std::vector<Contact_point*> Abstract_rb_engine::get_contact_forces()
{
	std::vector<Contact_point*> output;
	for (auto& contact_point : m_contact_points)
	{
		output.push_back(contact_point.get());
	}
	return output;
}

void Abstract_rb_engine::clear_contact_forces()
{	
	m_contact_points.clear();
}

//This method reads a list of rigid bodies from the specified file.
void Abstract_rb_engine::load_rbs_from_file(char* f_name)
{
	if (f_name == nullptr)
		std::logic_error("NULL file name provided.");
	FILE* f;
	const auto err = fopen_s(&f, f_name, "r");
	if (err != 0 || f == nullptr)
		throw std::logic_error("Could not open file: " + std::string(f_name));

	//have a temporary buffer used to read the file line by line...
	char buffer[200];
	//this is where it happens.
	while (!feof(f))
	{
		//get a line from the file...
		fgets(buffer, 200, f);
		if (strlen(buffer) > 195)
			std::logic_error("The input file contains a line that is longer than ~200 characters - not allowed");
		auto line = lTrim(buffer);
		const auto line_type = getRBLineType(line);
		switch (line_type)
		{
		case rb_utils::rb_rb:
		{
			//create a new rigid body and have it load its own info...
			auto newBody = std::make_shared<Rigid_body>();
			newBody->load_from_file(f);
			m_rigid_bodies.push_back(newBody);
			break;
		}
		case rb_utils::rb_arb:
		{
			//create a new articulated rigid body and have it load its own info...
			auto newBody = std::make_shared<Articulated_rigid_body>();
			newBody->load_from_file(f);
			m_rigid_bodies.push_back(newBody);
			break;
		}
		case rb_utils::rb_character:
		{
			//we have an articulated figure to worry about...
			m_character = std::make_shared<Character>(f, *this);
			break;
		}
		case rb_utils::rb_not_important:
			if (strlen(line) != 0 && line[0] != '#')
				throw std::logic_error("Ignoring input line: " + std::string(line));
			break;
		default:
			throw std::logic_error(
				"Incorrect rigid body input : " + std::string(buffer) + " - unexpected line.");
		}
	}
}

//This method is used to get the state of all the rigid body in this collection.
void Abstract_rb_engine::get_state(std::vector<double>& state)
{
	for (auto& rigid_body : m_rigid_bodies)
	{
		state.push_back(rigid_body->get_cm_position().x);
		state.push_back(rigid_body->get_cm_position().y);
		state.push_back(rigid_body->get_cm_position().z);

		state.push_back(rigid_body->get_orientation().s);
		state.push_back(rigid_body->get_orientation().v.x);
		state.push_back(rigid_body->get_orientation().v.y);
		state.push_back(rigid_body->get_orientation().v.z);

		state.push_back(rigid_body->get_cm_velocity().x);
		state.push_back(rigid_body->get_cm_velocity().y);
		state.push_back(rigid_body->get_cm_velocity().z);

		state.push_back(rigid_body->get_angular_velocity().x);
		state.push_back(rigid_body->get_angular_velocity().y);
		state.push_back(rigid_body->get_angular_velocity().z);
	}
}

//This method is used to set the state of all the rigid body in this collection.
void Abstract_rb_engine::set_state(const std::vector<double>& state, const int start)
{
	clear_contact_forces();
	auto i = start;
	for (auto& rigid_body : m_rigid_bodies)
	{
		rigid_body->set_cm_position(Point3d((state)[i + 0], (state)[i + 1], (state)[i + 2]));
		i += 3;
		rigid_body->set_orientation(Quaternion((state)[i + 0], (state)[i + 1], (state)[i + 2], (state)[i + 3]));
		i += 4;
		rigid_body->set_cm_velocity(Vector3d((state)[i + 0], (state)[i + 1], (state)[i + 2]));
		i += 3;
		rigid_body->set_angular_velocity(Vector3d((state)[i + 0], (state)[i + 1], (state)[i + 2]));
		i += 3;
	}
}


//	This method returns the reference to the rigid body with the given name, or NULL if it is not found
std::shared_ptr<Rigid_body> Abstract_rb_engine::get_rb_by_name(const std::string& name) const
{
	if (name.empty())
		return nullptr;

	for (auto& a_RB : m_rigid_bodies)
	{
		if (name == a_RB->get_name())
			return a_RB;
	}
	return nullptr;
}

std::vector<std::shared_ptr<Rigid_body>> Abstract_rb_engine::get_rigid_bodies() const
{
		return m_rigid_bodies;
}

std::shared_ptr<Joint> Abstract_rb_engine::get_joint_by_id(const size_t id) const
{
	for (auto& a_joint : m_character->get_joints())
	{
		if (a_joint->get_af_id() == id)
			return a_joint;
	}
	return nullptr;
}

std::shared_ptr<Joint> Abstract_rb_engine::get_joint_by_name(std::string& name) const
{
	if (name.empty())
		return nullptr;

	for (auto& a_joint : m_character->get_joints())
	{
		if (a_joint->get_name() == name)
			return a_joint;
	}
	return nullptr;
}

std::vector<std::shared_ptr<Joint>> Abstract_rb_engine::get_joints() const
{
	std::vector<std::shared_ptr<Joint>> joints;
	for (auto& a_joint : m_character->get_joints())
	{
		joints.push_back(a_joint);
	}
	return joints;
}
