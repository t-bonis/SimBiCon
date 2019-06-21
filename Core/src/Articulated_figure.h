#pragma once
#include <cmath>

#include "Muscle.h"
#include "ArticulatedRigidBody.h"




// An articulated figure is composed of many articulated rigid bodies that are interconnected by joints. 
// Characters, cars, ropes, etc, can all be viewed as articulated figures. 
// One note is that we will only allow tree structures - no loops.                                                       
class Articulated_figure : public Subject_interface
{
public:
	enum stance { right = 0, left = 1, both = 2, none = 3 };

public:
	Joint* get_stance_hip() const
	{
		switch (get_stance()) {
		case right: return m_right_hip;
		case left: return m_left_hip;
		case both: break;
		case none: break;
		default:;
		}
		return nullptr;
	}

	Joint* get_swing_hip() const
	{
		switch (get_stance()) {
		case right: return m_left_hip;
		case left: return m_right_hip;
		case both: break;
		case none: break;
		default:;
		}
		return nullptr;
	}

	Articulated_rigid_body* get_stance_foot() const
	{
		switch(get_stance()) { 
		case right: return m_right_foot;
		case left: return m_left_foot;
		case both: break;
		case none: break;
		default: ;
		}
		return nullptr;
	}

	Articulated_rigid_body* get_swing_foot() const
	{
		switch (get_stance()) {
		case right: return m_left_foot;
		case left: return m_right_foot;
		case both: break;
		case none: break;
		default:;
		}
		return nullptr;
	}

	//Create an other Articulated figure with new joints and new arbs 
	//But no physics it is just for drawing purpose only
	Articulated_figure(const Articulated_figure& other);

	Articulated_figure() = default;

	virtual ~Articulated_figure();

	Articulated_figure(Articulated_figure&& other) = delete;
	Articulated_figure& operator=(const Articulated_figure& other) = delete;
	Articulated_figure& operator=(Articulated_figure&& other) = delete;

	void read_reduced_state_from_file(const std::string& f_name, std::vector<double>& state) const;
	void load_reduced_state_from_file(const std::string& fName);
	void load_open_sim_reduced_state_from_file(const std::string& fName);
	void save_reduced_state_to_file(char* fName);
	void save_reduced_state_to_file(char* fName, std::vector<double>& state);
	void save_muscles_to_file(char* fName);

	Vector3d get_com() const;


	void save_previous_state()
	{
		for(auto& rigid_body : m_articulated_rigid_bodies)
		{
			rigid_body->save_previous_state();
		}
	}
	Vector3d get_com_velocity() const;


	std::vector<Articulated_rigid_body*> list_arbs_involved_form_anchor_to_target(
		Articulated_rigid_body& anchor, Articulated_rigid_body& target) const;

	void compute_articulated_figure_mass();

	void get_state(std::vector<double>& state);

	std::vector<double> get_state();


	stance get_stance() const { return m_stance; }
	void set_stance(const stance stance) { m_stance = stance; }


	void set_state(const std::vector<double>& state, int start = 0);
	void set_direct_state(std::vector<double>& state);
	void get_direct_state(std::vector<double>& state);
	void set_default_state();

	static void set_arbs_lines_state(const std::vector<double>& state, std::vector<Articulated_rigid_body*> arbs_involved);

	size_t get_state_dimension() const;

	//	This method is used to automatically fix the errors in the joints (i.e. drift errors caused by numerical integration).
	// At some future point it can be changed into a proper stabilization technique.
	void fix_joint_constraints_parent_to_child(bool fixOrientations = false, bool fixVelocities = false) const;

	//############################
	// Getter and Setter
	//############################

	double get_mass() const;

	std::string get_name() const;

	void set_name(const std::string& name)
	{
		m_name = name;
	}

	Articulated_rigid_body* get_root() const { return m_root; }
	void set_root(Articulated_rigid_body& a_root);

	size_t get_object_id(const std::string& a_name);

	std::vector<std::shared_ptr<Articulated_rigid_body>> get_arbs() const;
	Articulated_rigid_body* get_arb_by_name(const std::string& a_name);

	std::vector<std::shared_ptr<Muscle>> get_muscles() const { return m_muscles; }
	std::shared_ptr<Muscle> get_muscle_by_name(const std::string& a_name);

	std::vector<std::shared_ptr<Joint>> get_joints() const
	{
		return m_joints;
	}

	std::shared_ptr<Joint> get_joint_by_name(const std::string& a_name) const ;
	std::shared_ptr<Joint> get_joint_by_id(size_t i) const;

	std::vector<Articulated_rigid_body*> get_left_side_articulated_rigid_bodies() const
	{
		return m_left_side_articulated_rigid_bodies;
	}

	std::vector<Articulated_rigid_body*> get_right_side_articulated_rigid_bodies() const
	{
		return m_right_side_articulated_rigid_bodies;
	}
	Quaternion get_heading() const;
	
protected:

	std::vector<std::shared_ptr<Muscle>> m_muscles;

	std::vector<std::shared_ptr<Joint>> m_joints;

	Articulated_rigid_body* m_root{ nullptr };

	std::vector<std::shared_ptr<Articulated_rigid_body>> m_articulated_rigid_bodies;

	std::vector<Articulated_rigid_body*> m_left_side_articulated_rigid_bodies;

	std::vector<Articulated_rigid_body*> m_right_side_articulated_rigid_bodies;

	Joint* m_left_hip;

	Joint* m_right_hip;

	Articulated_rigid_body* m_left_toes;

	Articulated_rigid_body* m_right_toes;

	Articulated_rigid_body* m_left_foot;

	Articulated_rigid_body* m_right_foot;

	double m_mass{ 0 };

	std::string m_name;

	//this value indicates which side is the stance side. (right = 0; left = 1) 
	stance m_stance{ none };

};
