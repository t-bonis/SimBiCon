#pragma once

#include "ode/ode.h"
#include "AbstractRBEngine.h"
#include "HingeJoint.h"
#include "Custom_joint.h"
#include "UniversalJoint.h"
#include "BallInSocketJoint.h"
#include "StiffJoint.h"
#include "PreCollisionQuery.h"

#include "BoxCDP.h"
#include "PlaneCDP.h"
#include "CapsuleCDP.h"
#include "SphereCDP.h"
#include "TriMeshCDP.h"

constexpr int max_contact_feedback = 1000;
constexpr int max_nb_joints = 1000;

//This class is used as a wrapper that is designed to work with the Open Dynamics Engine. It uses all the rigid bodies (together with the joints)
//that are loaded with RBCollection, and has methods that link with ODE to simulate the physics. If a different physics engine is to be used,
//then ideally only the methods of this class need to be re-implemented, and the rest of the application can stay the same.
class Ode_world : public Abstract_rb_engine
{
public:
	struct ODE_RB_Map
	{
		dBodyID id;
		Rigid_body* rb;

		ODE_RB_Map(dBodyID newId, Rigid_body& newRb)
		{
			this->id = newId;
			this->rb = &newRb;
		}
	};


	struct ODE_Joint_Map
	{
		dJointID id;
		Joint* joint;

		ODE_Joint_Map(dJointID new_id, Joint& newJoint)
		{
			this->id = new_id;
			this->joint = &newJoint;
		}
	};

public:
	Ode_world();

	Ode_world(const Ode_world& other);

	virtual ~Ode_world();

	Ode_world(Ode_world&& other) = delete;
	Ode_world& operator=(const Ode_world& other) = delete;
	Ode_world& operator=(Ode_world&& other) = delete;

	void load_rbs_from_file(char* f_name) override;
	void load_rbs_from_other(const Ode_world& other) override;

	void load_af_from_struct(const Global& input) override;
	void load_af_from_other(const Ode_world& other) override;

	void link_simbicon_with_ode();

	//this method is used to set up an ODE plane geom. It is properly placed in body coordinates.
	dGeomID get_plane_geom(Plane_cdp* p, Rigid_body& parent) const;

	//this method is used to set up an ODE sphere geom. It is properly placed in body coordinates.
	dGeomID get_sphere_geom(SphereCDP* s) const;

	//this method is used to set up an ODE box geom. It is properly placed in body coordinates.
	dGeomID get_box_geom(BoxCDP* b) const;

	//this method is used to set up an ODE sphere geom. It is properly placed in body coordinates.
	dGeomID get_capsule_geom(Capsule_cdp* c) const;

	//this method is used to set up an ODE TriMesh geom. It is properly placed in body coordinates
	dGeomID get_tri_mesh_geom(Tri_mesh_cdp* mesh) const;

	//This method is used to set up an ode hinge joint, based on the information in the hinge joint passed in as a parameter
	void setup_ode_hinge_joint(HingeJoint& hj);

	void setup_ode_hinge_joint_opensim(Custom_joint& cj);

	void setup_ode_universal_joint(UniversalJoint& uj);

	void setup_ode_ball_and_socket_joint(BallInSocketJoint& basj);

	void setup_ode_ball_and_socket_joint_opensim(Custom_joint& cj);

	void setup_ode_stiff_joint(StiffJoint& sj);

	void setup_ode_stiff_joint_opensim(StiffJoint& sj);
	//	this method is used to copy the state of the ith rigid body to its ode counterpart.


	//this method is used to create ODE geom for all the collision primitives of the rigid body that is passed in as a parameter
	void create_ode_collision_primitives(Rigid_body& body) const;

	//this method is used to process the collision between the two objects passed in as parameters. More generally,
	//it is used to determine if the collision should take place, and if so, it calls the method that generates the
	//contact points.
	void process_collisions(dGeomID o1, dGeomID o2);


	friend void collisionCallBack(void* odeWorld, dGeomID o1, dGeomID o2);

	//This method is used to integrate the forward simulation in time.
	void engine_simulation_step() override;

	void sync_ode_and_state();

	//	this method is used to transfer the state of the rigid bodies, from the simulator to the rigid body wrapper
	virtual void set_rb_state_from_engine();

	//this method is used to copy the state of the ith rigid body, from the ode object to its rigid body counterpart 
	static void set_rb_state_from_ode(Rigid_body& rigid_body);

	//this method is used to transfer the state of the rigid bodies, from the rigid body wrapper to the simulator's rigid bodies
	virtual void set_engine_state_from_rb();

	static void set_engine_state_from_rb(Rigid_body& rigid_body);


	//This method is used to set the state of all the rigid body in the physical world.
	void set_state(const std::vector<double>& state, int start = 0) override;

	void reset_engine_state() override;

	static void reset_ode_state(Rigid_body& rigid_body);

	//	this method applies a force to a rigid body, at the specified point. The point is specified in local coordinates,
	//	and the force is also specified in local coordinates.
	void apply_rel_force_to(Rigid_body& rigid_body, const Vector3d& f, const Point3d& p) override;


	//this method applies a force to a rigid body, at the specified point. The point is specified in local coordinates,
	//and the force is specified in world coordinates.
	void apply_force_to(Rigid_body& rigid_body, const Vector3d& f, const Point3d& p) override;

	//this method applies a torque to a rigid body. The torque is specified in world coordinates.
	void apply_torque_to(Rigid_body& rigid_body, const Vector3d& t) override;

private:
	void init();

public:
	std::vector<ODE_Joint_Map> ode_to_joints;
	//keep track of the mapping between the rigid bodies and their ODE counterparts with this
private:
	dThreadingImplementationID m_thread_id{};
	// ODE's id for the simulation world
	dWorldID m_world_id{};
	// id of collision detection space
	dSpaceID m_space_id{};
	// id of contact group
	dJointGroupID contactGroupID{};


	//keep an array of contact points that is used for each pair of geom collisions
	dContact* m_cps{};

	//this is the max number of contacts that are going to be processed between any two objects
	int m_max_contact_count{};

	dJointFeedback m_contact_joints_feedback[max_contact_feedback]{};
public:
	dJointFeedback m_joints_feedback[max_nb_joints]{};
private:

	//this is the current number of contact joints, for the current step of the simulation
	size_t m_contact_joint_count{ 0 };

	//this is a pointer to a physical interface object that is used as an abstract way of communicating between the simulator and the application
	std::shared_ptr<PreCollisionQuery> pcQuery{};
};

inline void my_error_handler(const int errnum, const char *msg, va_list ap)
{
	char temp[300];
	sprintf(temp, msg, ap);
	std::cout << "Simulation failure : " << errnum << temp << std::endl;
	throw std::runtime_error(temp);
}

inline void my_message_handler(int errnum, const char * msg, va_list ap)
{
	char temp[300];
	sprintf(temp, msg, ap);
	std::cout << temp << " " << errnum << "\n";
}
