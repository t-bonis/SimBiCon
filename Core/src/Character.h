#pragma once
#include "Articulated_figure.h"
#include <boost/numeric/ublas/matrix.hpp>
#include "Force_utilitary.h"

class Controller_interface;

//A character is an articulated figure with some controllers
class Character : public Articulated_figure
{
public:

	struct State
	{
		bool body_touched_the_ground;
		stance stance;
	};

public:
	~Character() override = default;

	Character() = delete;
	Character(const Character& other) = delete;
	Character(Character&& other) = delete;
	Character& operator=(const Character& other) = delete;
	Character& operator=(Character&& other) = delete;


	std::vector<Controller_interface*> get_parent_controllers() const
	{
		return m_parent_controllers;
	}

	void add_controller(Controller_interface& controller)
	{
		m_parent_controllers.push_back(&controller);
	}

	//Create an articulated figure in the engine world
	Character(FILE* f, Abstract_rb_engine& physique_engine);
	Character(const Global& input, Abstract_rb_engine& physique_engine);
	Character(const Articulated_figure& other, Abstract_rb_engine& physique_engine);

	void load_joint_from_struct(const Joint_from_xml& a_joint);
	void load_joint_from_other(const std::shared_ptr<Joint>& other);
	void load_muscle_from_struct(const Muscle_from_xml& input);
	void load_muscle_from_other(const std::shared_ptr<Muscle>& other);
	Vector3d get_cop() const;
	void apply_virtual_force(Articulated_rigid_body& target, Articulated_rigid_body& anchor,
		const Vector3d& a_force) const;
	void apply_virtual_torque(Articulated_rigid_body& target, Articulated_rigid_body& anchor,
		const Vector3d& a_torque) const;
	void set_full_state(const State& state);
	void get_full_state(State& state) const;

	void fill_easy_access_pointers();
	void add_contact_points(Contact_point& contact_point);
	boost::numeric::ublas::matrix<double> compute_jacobian_pos(std::vector<Articulated_rigid_body*>& arbs_involved) const;
	boost::numeric::ublas::matrix<double> compute_jacobian_angles(std::vector<Articulated_rigid_body*>& arbs_involved) const;
	std::vector<Articulated_rigid_body*> get_anchors();
	Vector3d get_force_on(Rigid_body* rb) const;
	Vector3d get_force_from_ground(Rigid_body* rb) const;
	Vector3d get_torque_from_ground(Rigid_body* rb) const;
	Vector3d get_force_on(std::vector<Articulated_rigid_body*> bodies) const;
	Vector3d get_force_from_ground(std::vector<Articulated_rigid_body*> bodies) const;
	Vector3d get_torque_from_ground(std::vector<Articulated_rigid_body*> bodies) const;
	std::vector<Articulated_rigid_body*> get_swing_arbs() const;
	bool is_right_arb(Articulated_rigid_body*) const;
	bool is_left_arb(Articulated_rigid_body*) const;
	std::vector<Articulated_rigid_body*> get_stance_arbs() const;
	Vector3d get_swing_foot_loc() const;

	std::vector<Contact_point*> get_contact_points() const
	{
		return m_contact_points;
	}

	void clear_contact_points();


	bool is_body_touched_the_ground() const
	{
		return m_body_touched_the_ground;
	}

	void set_body_touched_the_ground(const bool body_touched_the_ground)
	{
		m_body_touched_the_ground = body_touched_the_ground;
	}

	bool is_legs_touched() const
	{
		return m_legs_touched;
	}

	void set_legs_touched(const bool legs_touched)
	{
		m_legs_touched = legs_touched;
	}

	void get_leg_contact_influence(std::vector<double>& vect_influence);
	void get_force_info_on_foot(Rigid_body* rb, Force_struct& heelForce, Force_struct& frontFeetForce,
	                            Force_struct& toeForce);
	bool continue_simulation();

	Abstract_rb_engine* get_physique_engine() const
	{
		return m_physique_engine;
	}
private:
	Abstract_rb_engine* m_physique_engine{ nullptr };
	std::vector<Contact_point*> m_contact_points;

	bool m_body_touched_the_ground{ false };
	bool m_legs_touched{ false };

	std::vector<Controller_interface*> m_parent_controllers;

	Vector3d _force_stance_foot[4];
	Vector3d _force_stance_toes;
	Vector3d _force_swing_foot[4];
	Vector3d _force_swing_toes;
};
