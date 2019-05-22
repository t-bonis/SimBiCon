#pragma once
#include "Articulated_figure.h"

class CCD_algorithm
{
public:
	explicit CCD_algorithm(const Articulated_figure& articulated_figure, Rigid_body* anchor, Rigid_body* end_effector);
	~CCD_algorithm() = default;

	void set_end_joint_target_pos(Point3d position,Quaternion orientation);
	void set_initial_state(std::vector<double> state);
	void compute_result();
	std::vector<double> get_result() const;

private:
	std::shared_ptr<Articulated_figure> m_articulated_figure;
	Point3d m_target_last_joint_pos{};
	std::vector<double> m_initial_state;
	std::vector<double> result;
	Quaternion m_target_end_effector_orientation;
	bool m_target_set = false;

	Articulated_rigid_body* m_anchor_body;
	Articulated_rigid_body* m_end_effector;
	std::vector<Articulated_rigid_body*> m_arbs_involved;
	std::vector<Joint*> m_joints_involved;
	size_t m_nb_arbs_involved{0};
	size_t m_nb_joints{ 0 };
	double m_error_threshold{0.0001};
	unsigned int m_max_number_of_iteration{30};
	unsigned int m_iteration_count{0};

	Vector3d m_v_last{}; //Vectors E-Pi
	Vector3d m_v_target{}; //Vectors T-Pi

	Vector3d m_rotation_axis{};
	double m_rotation_angle{0};

};

