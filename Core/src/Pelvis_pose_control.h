#pragma once
#include "Controller_interface.h"

class Pelvis_pose_control : public Controller_interface
{
public:
	Pelvis_pose_control(Character& character, SimBiCon_framework& control_framework);

	~Pelvis_pose_control() = default;

	Pelvis_pose_control() = delete;
	Pelvis_pose_control(const Pelvis_pose_control& other) = delete;
	Pelvis_pose_control(Pelvis_pose_control&& other) = delete;
	Pelvis_pose_control& operator=(const Pelvis_pose_control& other) = delete;
	Pelvis_pose_control& operator=(Pelvis_pose_control&& other) = delete;

	void add_pelvis_pose_correction() const;

	Vector3d compute_pd_force(const Point3d& pRel, const Point3d& pRelD, const Vector3d& vRel, const Vector3d& vRelD) const;
	Vector3d compute_pd_torque(const Quaternion& oRel, const Quaternion& oRelD, const Vector3d& vaRel, const Vector3d& vaRelD) const;

	void pre_simulation_step_phase_1() override;
	void pre_simulation_step_phase_2() override;
	void pre_simulation_step_phase_3() override;

	void post_simulation_step() override;
	void control_with_stance_leg() const;
	void control_with_torso() const;
	void control_with_torso_2() const;
	void test_function_modify_desired_pose() const;
	void test_function_modify_desired_pose_2() const;
	void test_function_pd_control() const;
	Controller_interface::type get_type() override;

	double get_kp_t() const
	{
		return m_kp_t;
	}

	void set_kp_t(double kp_t)
	{
		m_kp_t = kp_t;
	}

	double get_kd_t() const
	{
		return m_kd_t;
	}

	void set_kd_t(double kd_t)
	{
		m_kd_t = kd_t;
	}

	double get_kp_f() const
	{
		return m_kp_f;
	}

	void set_kp_f(double kp_f)
	{
		m_kp_f = kp_f;
	}

	double get_kd_f() const
	{
		return m_kd_f;
	}

	void set_kd_f(double kd_f)
	{
		m_kd_f = kd_f;
	}

private:
	//Desired position for easy access
	Articulated_figure* m_reference_pos{ nullptr };
	
	double m_kp_f = 0;
	double m_kd_f = 0;

	double m_kp_t = 0;
	double m_kd_t = 0;
};

