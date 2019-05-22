#pragma once


#include "Utils/src/SerializationUtils.h"
#include "Rigid_body.h"
#include "ContactPoint.h"
#include "StiffJoint.h"
#include "Character.h"

/*--------------------------------------------------------------------------------------------------------------------------------------------*
 * This class implements a container for rigid bodies (both stand alone and articulated). It reads a .rbs file and interprets it.             *
 *--------------------------------------------------------------------------------------------------------------------------------------------*/
class Abstract_rb_engine
{
public:

	Abstract_rb_engine() = default;
	virtual ~Abstract_rb_engine() = default;
	Abstract_rb_engine(const Abstract_rb_engine& other) = delete;
	Abstract_rb_engine(Abstract_rb_engine&& other)  = delete;
	Abstract_rb_engine& operator=(const Abstract_rb_engine& other) = delete;
	Abstract_rb_engine& operator=(Abstract_rb_engine&& other)  = delete;

	virtual void load_af_from_struct(const Global& input);
	virtual void load_af_from_other(const Ode_world& other);
	
	virtual void load_rbs_from_struct(const Global& input);
	virtual void load_rbs_from_other(const Ode_world& other);
	virtual void load_rbs_from_file(char* f_name);

	virtual void reset_engine_state() = 0;

	//This method is used to return a pointer to the list of contact forces
	std::vector<Contact_point*> get_contact_forces();
	void clear_contact_forces();

	//This method is used to integrate the forward simulation in time.
	virtual void engine_simulation_step() = 0;

	
	std::shared_ptr<Rigid_body> get_rb_by_name(const std::string& name) const;

	std::vector<std::shared_ptr<Rigid_body>> get_rigid_bodies() const;

	std::shared_ptr<Joint> get_joint_by_id(size_t id) const;
	std::shared_ptr<Joint> get_joint_by_name(std::string& name) const;

	std::vector<std::shared_ptr<Joint>> get_joints() const;

	void get_state(std::vector<double>& state);

	virtual void set_state(const std::vector<double>& state, int start = 0);

	std::shared_ptr<Character> get_character() const
	{
		return m_character;
	}

	//	this method applies a force to a rigid body, at the specified point. The point is specified in local coordinates,
	//	and the force is also specified in local coordinates.
	virtual void apply_rel_force_to(Rigid_body& b, const Vector3d& f, const Point3d& p) = 0;

	//	this method applies a force to a rigid body, at the specified point. The point is specified in local coordinates,
	//	and the force is specified in world coordinates.
	virtual void apply_force_to(Rigid_body& b, const Vector3d& f, const Point3d& p) = 0;

	//	this method applies a torque to a rigid body. The torque is specified in world coordinates.
	virtual void apply_torque_to(Rigid_body& b, const Vector3d& t) = 0;


protected:
	std::vector<std::shared_ptr<Rigid_body>> m_rigid_bodies;

	std::shared_ptr<Character> m_character;

	std::vector<std::shared_ptr<Contact_point>> m_contact_points;
};
