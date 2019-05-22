#pragma once
#include "ode/common.h"
#include "Muscle.h"
#include "ArticulatedRigidBody.h"

class Articulated_rigid_body;

//This class is responsible with the implementation of the methods that are necessary to implement joints in an articulated system. The joints impose constraints on
//the articulated rigid bodies that they connect. Each joint will be used to link a parent body to a child body. The joints that will be considered, for now at least,
//are all rotational joints with 1, 2 or 3 degrees of freedom. The default type of joint is a Ball in Socket joint with no joint limits.
class Joint : public Subject_interface
{
public:
	enum type { stiff, stiff_os, hinge, hinge_os, ball_in_socket, ball_in_socket_os, universal, custom, undefined };

public:
	Joint() = default;
	Joint(const Joint& other);

	virtual ~Joint() = default;

	Joint(Joint&& other)  = delete;
	Joint& operator=(const Joint& other) = delete;
	Joint& operator=(Joint&& other)  = delete;
	

	//	This method is used to compute the relative orientation between the parent and the child rigid bodies, expressed in 
	//	the frame coordinate of the parent.
	void compute_relative_orientation(Quaternion& q_rel) const;
	void compute_relative_angular_velocity(Vector3d& wRel) const;
	void compute_relative_angular_acceleration(Vector3d& wRel) const;

	//	This method is used to fix the joint angular constraint to correct for drift. This is done by changing
	//	the orientation of the child relative to the parent
	virtual void fix_angular_constraint_parent_to_child(const Quaternion& qRel) = 0;


	virtual void fix_angular_constraint_child_to_parent(const Quaternion& qRel) = 0;

	//This method is used to pass in information regarding the rotation axes. The string that is passed in is expected to have
	//been read from an input file.
	virtual void read_axes(char* axes);

	//This method is used to pass information regarding the joint limits for a joint. The string that is passed in is expected to
	//have been read from an input file.
	virtual void read_joint_limits(char* limits);

	virtual std::vector<Vector3d> get_rotation_axes_in_local_coords() const = 0;

	virtual std::vector<Vector3d> get_rotation_axes_in_global_coords() const = 0;

	virtual std::vector<std::vector<double>> get_rotation_limits() = 0;

	Point3d get_pos_in_global_coords();

	size_t get_af_id() const { return m_af_id; }

	void set_af_id(size_t i) { m_af_id = i; }

	bool is_muscle_actuated() const { return m_muscle_actuated; }

	void set_muscle_actuated(bool b) { m_muscle_actuated = b; }

	void add_muscle(Muscle& m) { this->m_overlapping_muscles.push_back(&m); }

	std::vector<Muscle*> get_muscles() const { return m_overlapping_muscles; }

	// return the angles between the parent and the child in the DoFs of the joint (muscles)
	virtual Vector3d get_angles() const = 0;

	//virtual void read_axis_from_struct(Joint_from_xml joint_input) {};

	//virtual void load_coordinates_from_struct(Joint_from_xml) {};

	virtual double get_translation(int unused_indices) { return 0; };

	//This method is used to automatically fix the errors in the joints (i.e. drift errors caused by numercial integration). At some future
	//point it can be changed into a proper stabilization technique.
	void fix_joint_constraints_parent_to_child(bool fixOrientations, bool fixVelocities, bool recursive);
	void fix_joint_constraints_child_to_parent(bool fixOrientations, bool fixVelocities, bool recursive);

	//This method is used to load the details of a joint from file. The PhysicalWorld parameter points to the world in which the objects
	//that need to be linked live in.
	void load_from_file(FILE* f, Abstract_rb_engine& world);

	//Returns the type of the current joint
	virtual type get_type() const = 0;


	//	sets the torque
	void set_torque(const Vector3d& t) { m_torque = t; }
	void set_muscle_torque(const Vector3d& t) { m_muscle_torque = t; }

	// set a torque to the current one (muscles)
	void add_torque(const Vector3d& t) { m_torque += t; }

	void add_muscle_torque(const Vector3d& t) { m_muscle_torque += t; }
	// get the current torque (muscles)
	Vector3d get_torque() const { return m_torque; }

	Vector3d get_muscle_torque() const { return m_muscle_torque; }

	//retrieves the reference to the body's parent
	Articulated_rigid_body* get_parent_arb() const { return m_parent_arb; }

	//retrieves the reference to the child's parent
	Articulated_rigid_body* get_child_arb() const { return m_child_arb; }

	//returns the position of the child joint, expressed in child's coordinates
	virtual Point3d get_joint_position_in_child() { return m_joint_pos_in_child; }

	//returns the position of the parent joint, expressed in parent's coordinates
	virtual Point3d get_joint_position_in_parent() { return m_joint_pos_in_parent; }



	void set_joint_position_in_child(double pos_x, double pos_y, double pos_z);
	void set_joint_position_in_parent(double pos_x, double pos_y, double pos_z);
	void set_child_arb(Articulated_rigid_body& shared);
	void set_parent_arb(Articulated_rigid_body& shared);


	//returns the name of this joint
	std::string get_name() const { return m_name; }

	void set_name(const std::string& a_name) { m_name = a_name; }

	bool is_using_joint_limits() const
	{
		return m_use_joint_limits;
	}

	void set_use_joint_limits(const bool use_joint_limits)
	{
		m_use_joint_limits = use_joint_limits;
	}
public:
	dJointID id_ode_world{};

protected:
	//this is the parent link
	Articulated_rigid_body* m_parent_arb{nullptr};
	//this is the location of the joint on the parent body - expressed in the parent's local coordinates
	Point3d m_joint_pos_in_parent;
	//this is the child link
	Articulated_rigid_body* m_child_arb{nullptr};
	//this is the location of the joint on the child body - expressed in the child's local coordinates 
	Point3d m_joint_pos_in_child;
	//this variable is used to indicate if this joint has joint limits or not (the details regarding the limits are specified on a per joint type basis)
	bool m_use_joint_limits{false};
	// variable used to indicate ether the joint is actuated by muscles or directly by the controller
	bool m_muscle_actuated{false};
	
	//the torque applied to this joint. It should be set/reset by a controller acting on this joint.
	Vector3d m_torque;
	Vector3d m_muscle_torque;
	
	//this is the name of the joint
	std::string m_name;

	//if this joint is part of a Character, this is its index in the Character's joints list [muscle edit]
	size_t m_af_id{size_t(-1)};

	std::vector<Muscle*> m_overlapping_muscles; // these are links to the muscles that are acting on the joint
};
