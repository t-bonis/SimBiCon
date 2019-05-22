#include "Joint.h"
#include "RBUtils.h"
#include "AbstractRBEngine.h"

Joint::Joint(const Joint& other)
{
	m_joint_pos_in_parent = other.m_joint_pos_in_parent;
	m_joint_pos_in_child = other.m_joint_pos_in_child;
	m_use_joint_limits = other.m_use_joint_limits;
	m_muscle_actuated = other.m_muscle_actuated;
	m_torque = other.m_torque;
	m_muscle_torque = other.m_muscle_torque;
	m_name = other.m_name;
}

//	This method is used to compute the relative orientation between the parent and the child rigid bodies, expressed in
//	the frame coordinate of the parent.
void Joint::compute_relative_orientation(Quaternion& q_rel) const
{
	//if qp is the quaternion that gives the orientation of the parent, and qc gives the orientation of the child, then  qp^-1 * qc gives the relative
	//orientation between the child and the parent, expressed in the parent's coordinates (child to parent)
	q_rel = m_parent_arb->get_orientation().get_inverse() * m_child_arb->get_orientation();
}

void Joint::compute_relative_angular_velocity(Vector3d& wRel) const
{
	wRel = m_child_arb->get_angular_velocity() - m_parent_arb->get_angular_velocity();
	//we will store wRel in the parent's coordinates, to get an orientation invariant expression for it
	wRel = m_parent_arb->get_local_coordinates(wRel);
}

void Joint::compute_relative_angular_acceleration(Vector3d& wwRel) const
{
	wwRel = m_child_arb->get_angular_acceleration() - m_parent_arb->get_angular_acceleration();
	//we will store wRel in the parent's coordinates, to get an orientation invariant expression for it
	wwRel = m_parent_arb->get_local_coordinates(wwRel);

}

void Joint::fix_joint_constraints_parent_to_child(bool fixOrientations, bool fixVelocities, bool recursive)
{
	if (!m_child_arb)
		return;

	//if it has a parent, we will assume that the parent's position is correct, and move the children to satisfy the joint constraint
	if (m_parent_arb)
	{
		//first fix the relative orientation, if desired
		if (fixOrientations)
		{
			Quaternion qRel;
			compute_relative_orientation(qRel);
			fix_angular_constraint_parent_to_child(qRel);
		}

		//now worry about the joint positions

		//compute the vector rc from the child's joint position to the child's center of mass (in world coordinates)
		auto rc = m_child_arb->get_vector_world_coordinates(Vector3d(m_joint_pos_in_child, Point3d(0, 0, 0)));
		//and the vector rp that represents the same quantity but for the parent
		auto rp = m_parent_arb->get_vector_world_coordinates(Vector3d(m_joint_pos_in_parent, Point3d(0, 0, 0)));

		//the location of the child's CM is now: pCM - rp + rc
		m_child_arb->set_cm_position(m_parent_arb->get_cm_position() + (rc - rp));

		//fix the velocities, if need be
		if (fixVelocities)
		{
			//to get the relative velocity, we note that the child rotates with wRel about the joint (axis given by wRel
			//d = vector from joint position to CM of the child),
			//but it also rotates together with the parent with the parent's angular velocity, 
			//so we need to combine these (all velocities are expressed in world coordinates already) (r is the vector
			//from the CM of the parent, to that of the child).
			Vector3d wRel = m_child_arb->get_angular_velocity() - m_parent_arb->get_angular_velocity();
			Vector3d r = Vector3d(m_parent_arb->get_cm_position(), m_child_arb->get_cm_position());
			Vector3d d = Vector3d(m_child_arb->get_point_world_coordinates(m_joint_pos_in_child), m_child_arb->get_cm_position());
			Vector3d vRel = m_parent_arb->get_angular_velocity().cross_product_with(r) + wRel.cross_product_with(d);
			m_child_arb->set_cm_velocity(m_parent_arb->get_cm_velocity() + vRel);
		}
	}

	//make sure that we recursivley fix all the other joint constraints in the articulated figure
	if (recursive)
	{
		for (auto& m_child_joint : m_child_arb->get_children_joints())
		{
			m_child_joint->fix_joint_constraints_parent_to_child(fixOrientations, fixVelocities, recursive);
		}
	}
}

void Joint::fix_joint_constraints_child_to_parent(bool fixOrientations, bool fixVelocities, bool recursive)
{
	if (!m_parent_arb)
		return;

	//if it has a parent, we will assume that the parent's position is correct, and move the children to satisfy the joint constraint
	if (m_child_arb)
	{
		//fix the orientation problems here... hopefully no rigid body is locked (except for the root, which is ok)

		//first fix the relative orientation, if desired
		if (fixOrientations)
		{
			Quaternion qRel;
			compute_relative_orientation(qRel);
			fix_angular_constraint_child_to_parent(qRel);
		}

		//now worry about the joint positions

		//compute the vector rc from the child's joint position to the child's center of mass (in world coordinates)
		const auto rc = m_child_arb->get_vector_world_coordinates(Vector3d(m_joint_pos_in_child, Point3d(0, 0, 0)));
		//and the vector rp that represents the same quantity but for the parent
		const auto rp = m_parent_arb->get_vector_world_coordinates(Vector3d(m_joint_pos_in_parent, Point3d(0, 0, 0)));

		//the location of the child's CM is now: pCM - rp + rc
		m_child_arb->set_cm_position(m_parent_arb->get_cm_position() + (rp - rc));

		//fix the velocities, if need be
		if (fixVelocities)
		{
			//to get the relative velocity, we note that the child rotates with wRel about the joint (axis given by wRel
			//d = vector from joint position to CM of the child),
			//but it also rotates together with the parent with the parent's angular velocity, 
			//so we need to combine these (all velocities are expressed in world coordinates already) (r is the vector
			//from the CM of the parent, to that of the child).
			Vector3d wRel = m_child_arb->get_angular_velocity() - m_parent_arb->get_angular_velocity();
			Vector3d r = Vector3d(m_parent_arb->get_cm_position(), m_child_arb->get_cm_position());
			Vector3d d = Vector3d(m_child_arb->get_point_world_coordinates(m_joint_pos_in_child), m_child_arb->get_cm_position());
			Vector3d vRel = m_parent_arb->get_angular_velocity().cross_product_with(r) + wRel.cross_product_with(d);
			m_child_arb->set_cm_velocity(m_parent_arb->get_cm_velocity() + vRel);
		}
	}

	//make sure that we recursivley fix all the other joint constraints in the articulated figure
	if (recursive)
	{
		for (auto& m_child_joint : m_child_arb->get_children_joints())
		{
			m_child_joint->fix_joint_constraints_child_to_parent(fixOrientations, fixVelocities, recursive);
		}
	}
}


/**
	This method is used to load the details of a joint from file. The PhysicalWorld parameter points to the world in which the objects
	that need to be linked live in.
*/
void Joint::load_from_file(FILE* f, Abstract_rb_engine& world)
{
	if (f == nullptr)
		throw std::logic_error("Invalid file pointer.");
	//have a temporary buffer used to read the file line by line...
	char buffer[200];
	char tempName[100];

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
		case rb_utils::rb_name:
			m_name = line;
			break;
		case rb_utils::rb_parent:
			sscanf_s(line, "%s", tempName, static_cast<unsigned>(_countof(tempName)));
			if (m_parent_arb != nullptr)
				throw std::logic_error("This joint already has a parent");
			m_parent_arb = dynamic_cast<Articulated_rigid_body*>(world.get_rb_by_name(tempName).get());
			if (m_parent_arb == nullptr)
				throw std::logic_error("The articulated rigid body " + std::string(tempName) + " cannot be found!");
			break;
		case rb_utils::rb_child:
			sscanf_s(line, "%s", tempName, static_cast<unsigned>(_countof(tempName)));
			if (m_child_arb != nullptr)
				throw std::logic_error("This joint already has a parent");
			m_child_arb = dynamic_cast<Articulated_rigid_body*>(world.get_rb_by_name(tempName).get());
			if (m_child_arb == nullptr)
				throw std::logic_error("The articulated rigid body " + std::string(tempName) + " cannot be found!");
			break;
		case rb_utils::rb_cpos:
			sscanf_s(line, "%lf %lf %lf", &m_joint_pos_in_child.x, &m_joint_pos_in_child.y, &m_joint_pos_in_child.z);
			break;
		case rb_utils::rb_ppos:
			sscanf_s(line, "%lf %lf %lf", &m_joint_pos_in_parent.x, &m_joint_pos_in_parent.y,
				&m_joint_pos_in_parent.z);
			break;
		case rb_utils::rb_end_joint:
			//we now have to link together the child and parent bodies
			if (m_child_arb == nullptr)
				throw std::logic_error("A joint has been found that does not have a child rigid body");
			if (m_parent_arb == nullptr)
				throw std::logic_error("A parent has been found that does not have a child rigid body");
			if (m_child_arb->get_parent_joint() != nullptr)
				throw std::logic_error("The child body " + m_child_arb->get_name() + " already has a parent.");
			m_parent_arb->add_child_joint(*this);
			m_child_arb->set_parent_joint(*this);
			return;
		case rb_utils::rb_joint_limits:
			read_joint_limits(line);
			break;
		case rb_utils::rb_not_important:
			break;
		default:
			throw std::logic_error(
				"Incorrect articulated body input file: " + std::string(buffer) + " unexpected line.");
		}
	}
	throw std::logic_error("Incorrect articulated body input file! No /ArticulatedFigure found joint");
}

void Joint::set_joint_position_in_child(double pos_x, double pos_y, double pos_z)
{
	m_joint_pos_in_child.x = pos_x;
	m_joint_pos_in_child.y = pos_y;
	m_joint_pos_in_child.z = pos_z;
}

void Joint::set_joint_position_in_parent(double pos_x, double pos_y, double pos_z)
{
	m_joint_pos_in_parent.x = pos_x;
	m_joint_pos_in_parent.y = pos_y;
	m_joint_pos_in_parent.z = pos_z;
}

void Joint::set_child_arb(Articulated_rigid_body& shared)
{
	m_child_arb = &shared;
}

void Joint::set_parent_arb(Articulated_rigid_body& shared)
{
	m_parent_arb = &shared;
}

//	This method is used to pass in information regarding the rotation axes. The string that is passed in is expected to have
//	been read from an input file.
void Joint::read_axes(char* axes)
{
}

//	This method is used to pass information regarding the joint limits for a joint. The string that is passed in is expected to
//	have been read from an input file.
void Joint::read_joint_limits(char* limits)
{
}

Point3d Joint::get_pos_in_global_coords()
{
	return get_child_arb()->get_point_world_coordinates(get_joint_position_in_child());
}
