#pragma once
#include "Joint.h"
#include "Rigid_body.h"
#include "ContactPoint.h"
#include "Muscle.h"

class Muscle;
class Joint;
class Articulated_figure;

// We will treat the articulated rigid bodies as normal rigid bodies that are connected by joints. 
// The joints are needed to enforce constraints between the articulated rigid bodies, but other than that,
// the dynamics are the same as for rigid bodies. We will assume that every articulated figure will be loop-free (tree hierarchies).
class Articulated_rigid_body : public Rigid_body
{
public:
	enum relation { parent, child };
public:
	Articulated_rigid_body(const Articulated_rigid_body& other);

	virtual ~Articulated_rigid_body() = default;

	Articulated_rigid_body() = default;
	Articulated_rigid_body(Articulated_rigid_body&& other)  = delete;
	Articulated_rigid_body& operator=(const Articulated_rigid_body& other) = delete;
	Articulated_rigid_body& operator=(Articulated_rigid_body&& other)  = delete;

	bool is_articulated() const override { return true;	}

	//Return children articulated bodies.
	std::vector<Articulated_rigid_body*> get_children_arbs(bool recursive = false);
	
	//Return parents articulated bodies.
	std::vector<Articulated_rigid_body*> get_parent_arbs(bool recursive = false) const;
	
	void set_parent_joint(Joint& a_joint);
	Joint* get_parent_joint() const;

	void add_child_joint(Joint& a_joint);
	std::vector<Joint*> get_children_joints() const;

	void set_af_parent(Articulated_figure& a_af);
	Articulated_figure* get_af_parent() const;

	// Add a muscle to the vector of attached_muscles if not already in
	void link_muscle(Muscle& t_muscle);
	Joint* get_joint_with(const Articulated_rigid_body& arb) const;
	//Return parent (child) if joint is a parent (child) of this arb 
	relation get_relation_with(const Joint& joint);


	bool is_parent_of(const Articulated_rigid_body& arb) const;
	std::vector<Contact_point*> get_contact_points() const;

private :
	//Return himself + children articulated body.
	void get_children_arbs(std::vector<Articulated_rigid_body*>& arbs, bool recursive = false);

	//Return himself + parents articulated body.
	void get_parents_arbs(std::vector<Articulated_rigid_body*>& arbs, bool recursive = false) const;

private :
	//this is the parent joint.
	Joint* m_parent_joint{nullptr};

	//the are children joints - it can have as many as it wants.
	std::vector<Joint*> m_child_joints;

	//articulated figure that the articulated rigid body belongs to
	Articulated_figure* m_parent_AF{nullptr};

	//the muscles attached to this articulated rigid body
	std::vector<Muscle*> m_attached_muscles;
};
