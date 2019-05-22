#include "Muscle.h"
#include "RBUtils.h"
#include "Core/src/SimGlobals.h"
#include "AbstractRBEngine.h"
#include <utility>


const double Muscle::cA = 100; //Geijtenbeek2013 -> Wang2012

Attachment_point::Attachment_point(const Attachment_point& other,const Abstract_rb_engine& physique_engine)
{
	m_offset_point = other.m_offset_point;
	m_body = std::dynamic_pointer_cast<Articulated_rigid_body>(physique_engine.get_rb_by_name(other.m_body->get_name())).get();
}

Muscle::Muscle(std::vector<std::shared_ptr<Attachment_point>> via_points, double _fMax, double _oLength,
               double _sLength)
	: m_via_points(std::move(via_points))
{
	init_fields();
	m_mtu = std::make_shared<MuscleTendonUnit>(_fMax, _oLength, _sLength);
}

Muscle::Muscle(const Muscle& other, Abstract_rb_engine& physique_engine)
{
	m_name = other.m_name;
	desired_activation = other.desired_activation;
	previousActivations = other.previousActivations;

	m_mtu = std::make_shared<MuscleTendonUnit>(*other.m_mtu);

	for(const auto& other_via_point : other.m_via_points)
	{
		m_via_points.push_back(std::make_shared<Attachment_point>(*other_via_point, physique_engine));
	}
}

void Muscle::add_via_point(const std::shared_ptr<Attachment_point> attachment_point)
{
	m_via_points.push_back(attachment_point);
}

void Muscle::set_mtu(std::shared_ptr<MuscleTendonUnit> shared)
{
	m_mtu = shared;
}

std::vector<std::shared_ptr<Attachment_point>> Muscle::get_via_points() const
{
	return m_via_points;
}

Joint* Muscle::get_joint(size_t joint_id)
{
	return m_joints[joint_id];
}

std::vector<Joint*> Muscle::get_joints() const
{
	return m_joints;
}

Vector3d Muscle::get_moment_arm(size_t joint_id)
{
	auto i = 0;
	for(auto& joint : m_joints)
	{
		if (joint->get_af_id() == joint_id)
		{
			return m_moment_arms[i];
		}
		i++;
	}
	return Vector3d({0,0,0});
}

MuscleTendonUnit* Muscle::get_mtu() const
{
	return m_mtu.get();
}

Vector3d& Muscle::get_torques_from_mtu(int i)
{
	return m_torques_from_mtu[i];
}


//Initialize the list of the joints overlapped by this muscle's segments
//assumes the rigid bodies of the attachment points are given in order from parent to child
void Muscle::init_joints()
{
	//if two consecutive attachment points are attached to different ARBodies, we need a link to the joint between them
	for (int i = 0; i < m_via_points.size() - 1; i++)
	{
		const auto a = m_via_points[i]->get_body();
		auto b = m_via_points[i + 1]->get_body();

		while (b->get_name() != a->get_name())
		{
			m_joints.push_back(b->get_parent_joint()); //the muscle knows the joint
			b->get_parent_joint()->add_muscle(*this);
			m_lines_over_joints.push_back(i); //the index of the overlapping segment
			b->get_parent_joint()->set_muscle_actuated(true); //indicate that this joint will be actuated by muscles


			b = b->get_parent_joint()->get_parent_arb();
		}
	}

	m_torques_from_mtu = std::vector<Vector3d>(m_joints.size());
	m_moment_arms = std::vector<Vector3d>(m_joints.size());
}

//compute the world coords of the segments and their total length
double Muscle::compute_segments_path_length()
{
	//compute world coords of the via points
	std::vector<Point3d> worldCoords;
	for (auto& m_via_point : m_via_points)
	{
		worldCoords.push_back(
			m_via_point->get_body()->get_point_world_coordinates(*m_via_point->get_local_offset_point_ptr()));
	}

	//create the link vectors between the attachment points
	//and compute the sum of their lengths
	double sum = 0;
	for (int i = 1; i < m_via_points.size(); i++)
	{
		Vector3d si(worldCoords[i - 1], worldCoords[i]);
		m_segments.push_back(si);
		sum += si.length();
	}
	return sum;
}

// compute the fiber length from the total length of the muscle
void Muscle::compute_current_fiber_length(double pathLength) const
{
	if (pathLength - m_mtu->optimalLength < m_mtu->slackLength)
	{
		//stiff tendon
		m_mtu->lCE = pathLength - m_mtu->slackLength;
	}
	else
	{
		//compliant tendon
		m_mtu->lCE = m_mtu->optimalLength;
	}
}

// calculate the moment arm for the joint at the given local index in the joints list, in world coords
Vector3d Muscle::compute_moment_arm(const uint iJoint)
{
	//get the world coordinates of the joint
	const Point3d j = m_joints[iJoint]->get_pos_in_global_coords();
	//get the segment that overlaps this joint
	Vector3d s = m_segments[m_lines_over_joints[iJoint]];
	//get the world coordinates of a point on the segment
	//for the segment Pi->P(i+1), take the via point Pi
	const Point3d p = m_via_points[m_lines_over_joints[iJoint]]->get_body()->get_point_world_coordinates(
		*m_via_points[m_lines_over_joints[iJoint]]->
		get_local_offset_point_ptr());
	Vector3d jp = p - j;
	return (jp).cross_product_with(s.toUnit());
}

// update the momentArms array
void Muscle::update_moment_arms()
{
	for (uint i = 0; i < m_joints.size(); i++)
	{
		m_moment_arms[i] = compute_moment_arm(i);
	}
}

// convert m_MTU->fOut into a torque for each of the covered joints and put them in torquesFromFMTU
void Muscle::convert_forces_to_torques()
{
	for (uint i = 0; i < m_torques_from_mtu.size(); i++)
	{

		Vector3d r = m_moment_arms[i];

		const Vector3d tau = r * m_mtu->fOut;

		m_torques_from_mtu[i] = tau;
	}
}

// Initialize the muscle's joints, segments, length, contraction velocity, moment arms, force and torque
void Muscle::init_fields()
{
	const double pathLength = compute_segments_path_length();
	init_joints();
	compute_current_fiber_length(pathLength);
	update_moment_arms();
	//the initialization of other m_MTU elt is done in the m_MTU constructor
	m_mtu->calcForce();
	convert_forces_to_torques();
}

// update the muscle's segments, length, contraction velocity, moment arms, force and torque
void Muscle::update_fields()
{
	m_segments.clear();
	const double pathLength = compute_segments_path_length();
	//update MTU's length and contraction velocity
	const double old_lCE = m_mtu->lCE;
	compute_current_fiber_length(pathLength);
	update_moment_arms();
	m_mtu->vCE = (m_mtu->lCE - old_lCE) / (SimGlobals::dt);
	m_mtu->calcForce();

	convert_forces_to_torques();
}

// given the joint's index global to the character, returns the index local to this muscle
int Muscle::get_joint_local_index(int globalIndex)
{
	for (uint i = 0; i < m_joints.size(); i++)
		if (m_joints[i]->get_af_id() == globalIndex)
			return i;
	return -1;
}

// update the activation level [0,1] given the neuronal excitation u and the time step dt [Geijtenbeek2013]
void Muscle::update_activation(double u, double dt) const
{
	const double aOld = m_mtu->activation;
	const double a = aOld + cA * dt * (u - aOld);
	set_activation(a);
}


// This method is used to load the details of a muscle from a file. The PhysicalWorld parameter points to the world in which the objects
//that need to be linked live in.
void Muscle::load_from_file(FILE* f, Abstract_rb_engine& world)
{
	if (f == nullptr)
		throw std::logic_error("Invalid file pointer.");

	//have a temporary buffer used to read the file line by line...
	char buffer[200];
	char tempName[100];

	std::shared_ptr<Attachment_point> attachmentPoint = std::make_shared<Attachment_point>();
	Articulated_rigid_body* current_body = nullptr;
	//keep a track of the bodies the muscle is attached to
	std::vector<Articulated_rigid_body*> articulated_rigid_bodies;

	//this is where it happens.
	while (!feof(f))
	{
		//get a line from the file...
		fgets(buffer, 200, f);
		if (strlen(buffer) > 195)
			throw std::logic_error("The input file contains a line that is longer than ~200 characters - not allowed");
		char* line = trim(buffer);
		switch (getRBLineType(line))
		{
		case rb_utils::rb_name:
			m_name = line;
			break;
		case rb_utils::rb_arb:
			//ARBody of an attachment point
			sscanf_s(line, "%s", tempName, static_cast<unsigned>(_countof(tempName)));
			current_body = dynamic_cast<Articulated_rigid_body*>(world.get_rb_by_name(tempName).get());
			if (current_body == nullptr)
				throw std::logic_error("The articulated rigid body " + std::string(tempName) + " cannot be found!");
			attachmentPoint->set_body(*current_body);
			break;
		case rb_utils::rb_position:
			//position of the previous attachment point, local to the ARBody
			if (sscanf_s(line, "%lf %lf %lf", &attachmentPoint->get_local_offset_point_ptr()->x,
			             &attachmentPoint->get_local_offset_point_ptr()->y,
			             &attachmentPoint->get_local_offset_point_ptr()->z) != 3) //TODO : Check if values are write
				throw std::logic_error(
					"Incorrect muscle input file - 3 arguments are required to specify the local coordinates position of a muscle\n");
			if (current_body == nullptr)
				throw std::logic_error("This muscle has a point that isn't attached to a body");
			//attachment point ready to be added to the muscle
			m_via_points.push_back(attachmentPoint);
			//the body will need a pointer to the current muscle
			articulated_rigid_bodies.push_back(current_body);
			break;
		case rb_utils::rb_max_force:
			sscanf(line, "%lf", &m_mtu->fMax);
			break;
		case rb_utils::rb_o_length:
			sscanf(line, "%lf", &m_mtu->optimalLength);
			break;
		case rb_utils::rb_s_length:
			sscanf(line, "%lf", &m_mtu->slackLength);
			break;
		case rb_utils::rb_delay:
			sscanf(line, "%lf", &m_neuronal_delay);
			break;
		case rb_utils::rb_end_muscle:
			if (current_body == nullptr)
				throw std::logic_error("This muscle has a point that isn't attached to a body");
			if (m_via_points.empty())
				throw std::logic_error("This muscle isn't attached to the figure at all");
			// initialize the remaining fields of the muscle
			this->init_fields();
			// all the bodies this muscle is attached to need a reference to it
			for (auto& articulated_rigid_body : articulated_rigid_bodies)
			{
				articulated_rigid_body->link_muscle(*this);
			}
			return; //and... done
		case rb_utils::rb_not_important:
			if (strlen(line) != 0 && line[0] != '#')
				throw std::logic_error("Ignoring input line: " + std::string(line));
			break;
		default:
			throw std::logic_error(
				"Incorrect articulated body input file: " + std::string(buffer) + " - unexpected line.");
		}
	}
	throw std::logic_error("Incorrect articulated body input file! No /ArticulatedFigure found muscles");
}


void Muscle::clone_properties(const Muscle& other)
{
	m_name = other.m_name;
	m_neuronal_delay = other.m_neuronal_delay;
}