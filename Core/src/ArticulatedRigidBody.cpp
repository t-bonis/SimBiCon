#include "ArticulatedRigidBody.h"
#include "Core/src/Character.h"

Articulated_rigid_body::Articulated_rigid_body(const Articulated_rigid_body& other) : Rigid_body(other)
{

}

std::vector<Articulated_rigid_body*> Articulated_rigid_body::get_children_arbs(const bool recursive)
{
	std::vector<Articulated_rigid_body*> output;

	get_children_arbs(output, recursive);

	return output;
}

void Articulated_rigid_body::get_children_arbs(std::vector<Articulated_rigid_body*>& arbs, const bool recursive)
{
	
	for (const auto& child_joint : m_child_joints)
	{
		if (recursive)
		{
			arbs.push_back(child_joint->get_child_arb());
			child_joint->get_child_arb()->get_children_arbs(arbs, true);
		}
		else
		{
			arbs.push_back(child_joint->get_child_arb());
			child_joint->get_child_arb()->get_children_arbs(arbs, false);
		}
	}
}

std::vector<Articulated_rigid_body*> Articulated_rigid_body::get_parent_arbs(bool recursive) const
{
	std::vector<Articulated_rigid_body*> output;

	get_parents_arbs(output, recursive);

	return output;
}

void Articulated_rigid_body::get_parents_arbs(std::vector<Articulated_rigid_body*>& arbs, const bool recursive) const
{
	if (recursive && m_parent_joint)
	{
		arbs.push_back(m_parent_joint->get_parent_arb());
		m_parent_joint->get_parent_arb()->get_parents_arbs(arbs, true);
	}
	if (!recursive && m_parent_joint)
	{
		arbs.push_back(m_parent_joint->get_parent_arb());
		m_parent_joint->get_parent_arb()->get_parents_arbs(arbs, false);
	}
}

void Articulated_rigid_body::set_af_parent(Articulated_figure& a_af)
{
	m_parent_AF = &a_af;
}

Articulated_figure* Articulated_rigid_body::get_af_parent() const
{
	return m_parent_AF;
}

void Articulated_rigid_body::set_parent_joint(Joint& a_joint)
{
	m_parent_joint = &a_joint;
}

Joint* Articulated_rigid_body::get_parent_joint() const
{
	return m_parent_joint;
}


void Articulated_rigid_body::add_child_joint(Joint& a_joint)
{
	m_child_joints.push_back(&a_joint);
}

std::vector<Joint*> Articulated_rigid_body::get_children_joints() const
{
	return m_child_joints;
}

void Articulated_rigid_body::link_muscle(Muscle& t_muscle)
{
	for (const auto& attached_muscle : m_attached_muscles)
	{
		if (&t_muscle == attached_muscle)
		{
			return;
		}
	}
	m_attached_muscles.push_back(&t_muscle);
}

Joint* Articulated_rigid_body::get_joint_with(const Articulated_rigid_body& arb) const
{
	if(m_parent_joint)
	{
		if (get_parent_joint()->get_parent_arb() == &arb)
		{
			return get_parent_joint();
		}
	}
	for (const auto& joint : get_children_joints())
	{
		if (joint->get_child_arb() == &arb)
		{
			return joint;
		}
	}
	return nullptr;
}

Articulated_rigid_body::relation Articulated_rigid_body::get_relation_with(const Joint& joint)
{
	if (m_parent_joint)
	{
		if (m_parent_joint->get_name() == joint.get_name())
		{
			return parent;
		}
	}
	for(const auto child_joint : m_child_joints)
	{
		if(child_joint->get_name() == joint.get_name())
		{
			return child;
		}
	}
	throw std::logic_error("Joint not found");
}

bool Articulated_rigid_body::is_parent_of(const Articulated_rigid_body& arb) const
{
	for (const auto& joint : get_children_joints())
	{
		if (joint->get_child_arb() == &arb)
		{
			return true;
		}
	}
	return false;
}

std::vector<Contact_point*> Articulated_rigid_body::get_contact_points() const
{
	std::vector<Contact_point*> output;
	if (m_parent_AF)
	{
		for (auto contact_point : dynamic_cast<Character*>(m_parent_AF)->get_contact_points())
		{
			if (contact_point->rb1->get_name() == get_name() || contact_point->rb2->get_name() == get_name())
			{
				output.push_back(contact_point);
			}
		}
	}
	return output;
}