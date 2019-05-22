#pragma once
#include "MuscleTendonUnit.h"
#include "Utils/src/BufferActivation.h"
#include "Observer_interface.h"
#include "Subject_interface.h"

class Abstract_rb_engine;
class Articulated_rigid_body;
class Joint;

class Attachment_point
{
public:
	Attachment_point() = default;
	Attachment_point(const Attachment_point& other,const Abstract_rb_engine& physique_engine);


	~Attachment_point() = default;

	Attachment_point(const Attachment_point& other) = delete;
	Attachment_point(Attachment_point&& other)  = delete;
	Attachment_point& operator=(const Attachment_point& other) = delete;
	Attachment_point& operator=(Attachment_point&& other)  = delete;

	// access the articulated rigid body
	Articulated_rigid_body* get_body() const { return m_body; }

	void set_body(Articulated_rigid_body& a_body) { m_body = &a_body; }

	// access the local position
	Point3d* get_local_offset_point_ptr() { return &m_offset_point; }

	void setLocalOffsetPoint(const Point3d& a_offsetPoint) { m_offset_point = a_offsetPoint; }
private:
	// link to the skeleton
	Articulated_rigid_body* m_body{};
	// position in local coords system of the body
	Point3d m_offset_point;
};

// A Muscle consists of a geometry (points attached on rigid bodies) and a MTU that produces a force
class Muscle : public Subject_interface
{
public:
	Muscle(std::vector<std::shared_ptr<Attachment_point>> via_points, double _fMax, double _oLength, double _sLength);
	Muscle() = default;
	Muscle(const Muscle& other, Abstract_rb_engine& physique_engine);

	~Muscle() = default;

	Muscle(const Muscle& other) = delete;
	Muscle(Muscle&& other)  = delete;
	Muscle& operator=(const Muscle& other) = delete;
	Muscle& operator=(Muscle&& other)  = delete;

	

	void add_via_point(std::shared_ptr<Attachment_point> attachment_point);
	void set_mtu(std::shared_ptr<MuscleTendonUnit> shared);
	std::vector<std::shared_ptr<Attachment_point>> get_via_points() const;
	Joint* get_joint(size_t joint_id);
	std::vector<Joint*> get_joints() const;
	Vector3d get_moment_arm(size_t joint_id);
	MuscleTendonUnit* get_mtu() const;
	Vector3d& get_torques_from_mtu(int i);

	std::string get_name() const { return m_name; }

	// gives the maximum isometric force of the muscle
	double get_max_iso_force() const { return m_mtu->fMax; }

	// sets the maximum isometric force of the muscle to the given value
	void set_max_iso_force(double f) const { m_mtu->fMax = f; }

	// for tests : directly set an output force
	void set_force(double f) const { m_mtu->fOut = f; }
	void set_af_id(size_t id) { m_id = id; }

	// gives the current force of the muscle
	double get_current_force() const { return m_mtu->fOut; }

	// directly set an activation
	//normally not allowed to be set directly, need to go through the neuronal excitation u and updateActivation (controller)
	void set_activation(double a) const
	{
		if (a < 0) a = 0;
		if (a > 1) a = 1;
		m_mtu->activation = a;
	};

	//gives the current activation (in [0,1])
	double get_activation() const { return m_mtu->activation; };

	//get the neuronal delay (in seconds)
	double get_neuronal_delay() const { return m_neuronal_delay; };

	//access the current muscle fiber length (in meters)
	double get_length() const { return m_mtu->lCE; }

	//access the current contraction velocity of the muscle (m/s)
	double get_contraction_velocity() const { return m_mtu->vCE; }

	//access the tendon slack length (in m) 
	double get_tendon_slack_length() const { return m_mtu->slackLength; }

	//set the tendon slack length (in m) 
	void set_tendon_slack_length(double l) const { m_mtu->slackLength = l; }

	//access the optimal fiber length (in m) 
	double get_optimal_fiber_length() const { return m_mtu->optimalLength; }

	//set the optimal fiber length (in m) 
	void set_optimal_fiber_length(double l) const { m_mtu->optimalLength = l; }

	//initialize the muscle's joints, segments, length, contraction velocity, moment arms, force and torque
	void init_fields();

	//update the muscle's segments, length, contraction velocity, moment arms, force and torque
	void update_fields();

	//given the joint's index global to the character, returns the index local to this muscle
	int get_joint_local_index(int globalIndex);

	//This method is used to load the details of a muscle from file. The PhysicalWorld parameter points to the world in which the objects
	//that need to be linked live in.
	void load_from_file(FILE* f, Abstract_rb_engine& world);

	//update the activation level [0,1] given the neuronal excitation u and the time step dt [Geijtenbeek2013]
	void update_activation(double u, double dt) const;

	//calculate the moment arm for the joint at the given local index in the joints list, in world coords
	Vector3d compute_moment_arm(uint iJoint);

	//update the momentArms array
	void update_moment_arms();

	//convert m_MTU->fOut into a torque for each of the covered joints and put them in torques_from_mtu
	void convert_forces_to_torques();

	void clone_properties(const Muscle& other);

public:
	//constant activation and deactivation rate, per second
	static const double cA; 

	//buffer containing some of the previous activations
	Buffer_activation previousActivations; 
	
	double desired_activation{ 0 };

	std::string  m_name; 

	size_t m_id{};


private:
	// get the joints that are covered by the muscle
	void init_joints();

	// compute the segments in world coords and return the total path length
	double compute_segments_path_length();

	// compute the fiber length (lCE) from the total length of the muscle
	void compute_current_fiber_length(double pathLength) const;


private:
	// underlying MTU
	std::shared_ptr<MuscleTendonUnit> m_mtu{nullptr}; 

	// points where the tendon is attached to a bone (rigid body) in local coords
	std::vector<std::shared_ptr<Attachment_point>> m_via_points;

	// segments between the via points in world coords
	std::vector<Vector3d> m_segments; 

	//delay (in seconds) used in the computation of the desired neuronal excitation signal u, read from file, values from [Geijtenbeek2013]
	double m_neuronal_delay{0};

	//link to joint(s) covered by the muscle
	std::vector<Joint*> m_joints; 

	// the indexes of the segments overlapping the joints
	std::vector<uint> m_lines_over_joints;

	// moment arm for each overlapped joint
	std::vector<Vector3d> m_moment_arms; 
	std::vector<Vector3d> m_torques_from_mtu;
};
