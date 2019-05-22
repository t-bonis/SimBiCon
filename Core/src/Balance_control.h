#pragma once
#include "Controller_interface.h"

class Balance_control : public Controller_interface
{
public:
	explicit Balance_control(Character& character, SimBiCon_framework& control_framework);

	~Balance_control() = default;

	Balance_control() = delete;
	Balance_control(const Balance_control& other) = delete;
	Balance_control(Balance_control&& other) = delete;
	Balance_control& operator=(const Balance_control& other) = delete;
	Balance_control& operator=(Balance_control&& other) = delete;


	void add_gravity_compensation() const;
	void correct_cop_position() const;
	static Vector3d compute_pd_force(const Point3d& pRel, const Point3d& pRelD, const Vector3d& vRel, const Vector3d& vRelD);
	static Vector3d compute_pd_torque(const Quaternion& qRel, const Quaternion& qRelD, const Vector3d& wRel,
	                           const Vector3d& wRelD);

	void pre_simulation_step_phase_1() override;
	void pre_simulation_step_phase_2() override;
	void pre_simulation_step_phase_3() override;

	void post_simulation_step() override;
	Controller_interface::type get_type() override;
};

