#pragma once
#include "SimGlobals.h"
#include "CollisionDetectionPrimitive.h"
#include "Utils/src/SerializationUtils.h"
#include "Subject_interface.h"
#include "ode/ode.h"

class Articulated_rigid_body;

//This class is used as the basis for an Articulated Rigid Body. Together with the PhysicalWorld class, this class is used to implement the dynamics of rigid bodies.
//NOTE: It is assumed that the location of the center of mass of the object in local coordinates is (0,0,0) and that the principal moments of inertia are aligned to the
//local coordinates x, y, and z axes!
class Rigid_body : public Subject_interface
{
public:
	enum att_id
	{
		sphere
	};
	struct RBState
	{
		// the position of the center of mass of the rigid body
		Point3d position{ 0.0, 0.0, 0.0 };
		// its arb_orientation
		Quaternion orientation{ 1.0, 0.0, 0.0, 0.0 };
		// the velocity of the center of mass
		Vector3d velocity{ 0.0, 0.0, 0.0 };
		// and finally, the angular velocity about the center of mass
		Vector3d angularVelocity{ 0.0, 0.0, 0.0 };
	};

	struct RBProperties
	{
		double mass{ 1 };
		//and keep a copy of the inverse mass as well, for easy access
		double inv_mass{ 1 };
		//we'll store the moment of inertia in a vector that represents the diagonal of the local frame MOI
		Vector3d local_moi{ 1, 1, 1 };
		//we'll also compute the inverse MOI, in local coordinates
		Vector3d local_inv_moi{ 1, 1, 1 };
		//this variable indicates ether or not this rigid body is fixed. When setting this to true/false, the inverse mass and inverse moment of inertia
		//will be set to zero (for true), or to the proper inverses otherwise. 
		bool locked{ false };

		double epsilon{ 0.0 }; // restitution coeff
		double mu{ 10e30 }; //friction coeff

		//if this method is set to true, then the body is constrained to be a planar object (move in the x-y plane and only rotate about the z-axis)
		bool planar{ false };

		//these are a few specific properties (that make sense with ODE - don't know how they would match up with other simulators)
		double ground_softness{ 0.1};
		double ground_penalty{ 0.2};
	};

public:

	Rigid_body() = default;
	Rigid_body(const Rigid_body& other);

	virtual ~Rigid_body() = default;

	Rigid_body(Rigid_body&& other) = delete;
	Rigid_body& operator=(const Rigid_body& other) = delete;
	Rigid_body& operator=(Rigid_body&& other) = delete;

	double get_ground_softness() const;
	double get_ground_penalty() const;

	// This method returns the coordinates of the point that is passed in as a parameter(expressed in local coordinates), in world coordinates.
	Point3d get_point_world_coordinates(const Point3d& localPoint) const;

	// This method is used to return the local coordinates of the point that is passed in as a parameter (expressed in global coordinates)
	Point3d get_local_coordinates(const Point3d& globalPoint) const;

	// This method is used to return the local coordinates of the vector that is passed in as a parameter (expressed in global coordinates)
	Vector3d get_local_coordinates(const Vector3d& globalVector) const;

	// This method returns the vector that is passed in as a parameter(expressed in local coordinates), in world coordinates.
	Vector3d get_vector_world_coordinates(const Vector3d& localVector) const;

	// This method returns the absolute velocity of a point that is passed in as a parameter. The point is expressed in local coordinates, and the
	// resulting velocity will be expressed in world coordinates.
	Vector3d get_absolute_velocity_for_local_point(const Point3d& localPoint) const;

	// This method returns the absolute velocity of a point that is passed in as a parameter. The point is expressed in global coordinates, and the
	// resulting velocity will be expressed in world coordinates.
	Vector3d get_absolute_velocity_for_global_point(const Point3d& globalPoint) const;

	void save_previous_state()
	{
		m_previous_state = m_state;
	}

	// This method returns the world coordinates of the position of the center of mass of the object
	Point3d get_cm_position() const
	{
		return m_state.position;
	}

	// This method sets the world coordinate of the position of the center of mass of the object
	void set_cm_position(const Point3d& new_cm_pos)
	{
		m_state.position = new_cm_pos;
	}


	//	This method returns the body's center of mass velocity
	Vector3d get_cm_velocity() const
	{
		return m_state.velocity;
	}

	//This method sets the velocity of the center of mass of the object
	void set_cm_velocity(const Vector3d& new_cm_vel)
	{
		m_state.velocity = new_cm_vel;
	}

	//this method sets the angular velocity of the body
	void set_angular_velocity(const Vector3d& newAVel)
	{
		m_state.angularVelocity = newAVel;
	}

	//This method returns the rigid body's coefficient of restitution
	double get_restitution_coefficient() const
	{
		return m_properties.epsilon;
	}

	// This method returns the rigid body's coefficient of restitution
	double get_friction_coefficient() const
	{
		return m_properties.mu;
	}

	void load_rb_from_struct(const A_RigidBody& a_rigid_body);

	void generate_cdp_from_meshes();

	// This method loads all the pertinent information regarding the rigid body from a file.
	void load_from_file(FILE* f);

	// Returns the mass of the rigid body
	double get_mass() const
	{
		return m_properties.mass;
	}

	//	this method sets the id of the current rigid body.
	void set_ode_id(dBodyID new_id)
	{
		this->m_ode_id = new_id;
	}

	dBodyID get_ode_id() const
	{
		return m_ode_id;
	}

	// this method returns the body's principal moments of inertia, 
	//about the principal axes (which correspond to the bodies local coordinate
	// frame axes)
	Vector3d get_moi() const
	{
		return m_properties.local_moi;
	}

	// this method returns the body's arb_orientation
	Quaternion get_orientation() const
	{
		return m_state.orientation;
	}

	// this method sets the body's arb_orientation
	void set_orientation(const Quaternion& q)
	{
		m_state.orientation = q;
	}

	// this method returns the body's angular velocity
	Vector3d get_angular_velocity() const
	{
		return m_state.angularVelocity;
	}

	Vector3d get_angular_acceleration() const
	{
		return (m_state.angularVelocity - m_previous_state.angularVelocity)/SimGlobals::dt;
	}

	// this method returns true if the current body is locked, false otherwise
	bool is_locked() const
	{
		return m_properties.locked;
	}

	// this method returns false if this body is a simple rigid body, false if it is an articulated figure
	virtual bool is_articulated() const { return false; }

	bool is_planar() const { return m_properties.planar; }

	virtual std::vector<std::shared_ptr<Model>> get_models()
	{
		return m_models;
	}

	std::vector<std::shared_ptr<CollisionDetectionPrimitive>> get_cdps() const
	{
		return m_CDPs;
	}

	std::string get_name() const
	{
		return m_name;
	}

	void set_mass(double m)
	{
		m_properties.mass = m;
		m_properties.inv_mass = 1 / m;
		if (m_properties.locked)
			m_properties.inv_mass = 0;
	}

	void set_moi(Vector3d vM)
	{
		set_moi(vM.x, vM.y, vM.z);
	}

	void set_moi(double xM, double yM, double zM)
	{
		m_properties.local_moi.setValues(xM, yM, zM);
		m_properties.local_inv_moi.setValues(1 / xM, 1 / yM, 1 / zM);
		if (m_properties.locked)
			this->m_properties.local_inv_moi.setValues(0, 0, 0);
	}

	void lock()
	{
		m_properties.locked = true;
		m_properties.local_inv_moi.setValues(0, 0, 0);
		m_properties.inv_mass = 0;
	}

	void release()
	{
		m_properties.locked = false;
		m_properties.local_inv_moi.setValues(1 / m_properties.local_moi.x, 1 / m_properties.local_moi.y,
			1 / m_properties.local_moi.z);
		m_properties.inv_mass = 1 / m_properties.mass;
	}

	void set_af_id(size_t id)
	{
		m_af_id = id;
	}

	size_t get_af_id() const
	{
		return m_af_id;
	}

	Vector3d get_angular_velocity_avg() const
	{
		return m_angular_velocity_avg;
	}

	///@brief this function should be called once (and only once) every simulation step
	void update_angular_velocity_avg();

protected:
	// the state of the rigid body: made up of the object's position in the world, its arb_orientation and linear/angular velocities (stored in world coordinates)
	RBState m_state;

	RBState m_previous_state;

	// the physical properties of the rigid bodies: mass and inertia, stored in a convenient to use form
	RBProperties m_properties;
	// an array with all the collision detection primitives that are relevant for this rigid body
	std::vector<std::shared_ptr<CollisionDetectionPrimitive>> m_CDPs;
	// the mesh(es) that are used when displaying this rigid body
	std::vector<std::shared_ptr<Model>> m_models;
	// the name of the rigid body - it might be used to reference the object for articulated bodies
	std::string m_name;
	// the id of the rigid body
	dBodyID m_ode_id{ nullptr };

	size_t m_af_id{ size_t(-1) };


	//the next members will be used to store an avg on the velocity so that we are sure to have a continuous velcity
	std::vector<Vector3d> m_previous_angular_velocity_avg;
	Vector3d m_angular_velocity_avg;
	int m_previous_angular_vel_avg_max_size{};

	private:
		std::map<std::string, att_id> m_attribute_id{
	{"sphere",sphere},
		};
};
