#pragma once
#include "Observer_interface.h"
#include "Utils/src/Utils.h"
#include "MathLib/src/Vector.h"
#include "Subject_interface.h"

class SimBiCon_framework;

class Gait_analyzer : public Observer_interface, public Subject_interface
{
public:
	Gait_analyzer() = delete;
	Gait_analyzer(SimBiCon_framework& conF);
	Gait_analyzer(const Gait_analyzer& other) = delete;

	~Gait_analyzer() = default;
	
	Gait_analyzer(Gait_analyzer&& other) = delete;
	Gait_analyzer& operator=(const Gait_analyzer& other) = delete;
	Gait_analyzer& operator=(Gait_analyzer&& other) = delete;

	double compute_angular_diff(std::array<size_t, 2> interval = {0,0});

	double compute_effort(std::array<size_t, 2> interval = { 0,0 });

	double compute_pelvis_pos_diff(std::array<size_t, 2> interval = { 0,0 });

	double compute_pelvis_orientation_diff(std::array<size_t, 2> interval = { 0,0 });

	double compute_gain_sum() const;

	void reset();

	void update(const Subject_interface* observable) override;

	size_t get_nb_of_missing_frame() const;
	
	void print_values() const;

private :
	void init_size_of_vectors();

public:
	std::vector<std::vector<double>> activations;

	std::vector<std::vector<Vector3d>> angles;
	std::vector<std::vector<Vector3d>> angles_reference;

	std::vector<std::vector<Vector3d>> angular_velocity;
	std::vector<std::vector<Vector3d>> angular_velocity_reference;

	std::vector<std::vector<Vector3d>> arb_angular_velocity;
	std::vector<std::vector<Vector3d>> arb_angular_velocity_reference;
	
	std::vector<std::vector<Vector3d>> angular_acceleration;
	std::vector<std::vector<Vector3d>> angular_acceleration_reference;

	std::vector<std::vector<Point3d>> pos;
	std::vector<std::vector<Point3d>> pos_reference;

	std::vector<std::vector<Vector3d>> vel;
	std::vector<std::vector<Vector3d>> vel_reference;

	std::vector<std::vector<Quaternion>> orientation;
	std::vector<std::vector<Quaternion>> orientation_reference;

	std::vector<std::vector<Vector3d>> torques1;
	std::vector<std::vector<Vector3d>> torques2;
	std::vector<std::vector<Vector3d>> forces1;
	std::vector<std::vector<Vector3d>> forces2;

	std::vector<Vector3d> force_on_r_foot;
	std::vector<Vector3d> torque_on_r_foot;
	std::vector<Vector3d> force_on_l_foot;
	std::vector<Vector3d> torque_on_l_foot;
	std::vector<size_t> l_foot_nb_contact;
	std::vector<size_t> r_foot_nb_contact;
	std::vector<Vector3d> cop;
	std::vector<Vector3d> com_vel;
	std::vector<Vector3d> com_vel_ref;
	
	size_t target_simulation_length{size_t(-1)};


private:

	SimBiCon_framework* m_con_f;

};

