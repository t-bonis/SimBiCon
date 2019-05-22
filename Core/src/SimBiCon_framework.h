#pragma once
#include "Trajectory.h"
#include "SimBiCon.h"
#include "Gait_analyzer.h"


class Qt_gui;
class Swing_foot_controller;

class SimBiCon_framework
{
public:
	struct Ref_trajectories
	{
		Trajectory1D values;
		QVector3D translation_axis;
		QVector3D rotation_axis;
		std::string object_name;
		size_t object_id;
	};

public:

	SimBiCon_framework() = default;
	SimBiCon_framework(const SimBiCon_framework& other);
	explicit SimBiCon_framework(std::string& conF_filename);

	~SimBiCon_framework();

	SimBiCon_framework(SimBiCon_framework&& other) = delete;
	SimBiCon_framework& operator=(const SimBiCon_framework& other) = delete;
	SimBiCon_framework& operator=(SimBiCon_framework&& other) = delete;


	//############################
	// Loading
	//############################

	void load_ref_trajectory(char* input);
	void load_pelvis_pose_controller(char* input);
	void init_gait_analyzer();


	//############################
	// Simulation
	//############################

	void init_controlled_character_state() const;
	std::shared_ptr<Swing_foot_controller> get_swing_foot_controller();
	void add_drawing();
	void global_step(double dt);
	void update_desired_pos_handler();
	void update_reference_trajectories();
	void update_ref_traj_correction();
	void update_walking_cycle_time();

	//############################
	// Getter
	//############################

	std::shared_ptr<Gait_analyzer> get_gait_analyzer() const
	{
		return m_gait_analyzer;
	}

	std::shared_ptr<Articulated_figure> get_desired_pose_handler() const
	{
		return m_desired_pose_handler;
	}

	std::shared_ptr<Articulated_figure> get_reference_handler() const
	{
		return m_reference_handler;
	}

	std::vector<Ref_trajectories> get_reference_trajectories() const
	{
		return m_reference_trajectories;
	}

	double get_walking_cycle_time() const
	{
		return m_walking_cycle_time;
	}

	Vector3d get_ref_traj_correction() const
	{
		return m_ref_traj_correction;
	}

	std::vector<double> get_result() const
	{
		return m_results;
	}

	size_t get_sample_id() const
	{
		return m_sample_id;
	}

	//############################
	// Setter
	//############################


	void set_walking_cycle_time(double walking_cycle_time)
	{
		m_walking_cycle_time = walking_cycle_time;
	}

	void add_result(double result)
	{
		m_results.push_back(result);
	}

	void set_sample_id(size_t sample_id)
	{
		m_sample_id = sample_id;
	}

	void set_success(bool value)
	{
		m_simulation_success = value;
	}

	bool get_success() const
	{
		return m_simulation_success;
	}

	Character* get_controlled_character() const
	{
		return m_controlled_character;
	}

	void set_controlled_character(Character* controlled_character)
	{
		m_controlled_character = controlled_character;
	}

	Abstract_rb_engine* get_physical_world() const
	{
		return m_physical_world.get();

	}

	void link_gl_widget(My_qopengl_widget* gl_widget)
	{
		m_gl_widget = gl_widget;
	}

	std::vector<std::shared_ptr<Controller_interface>> get_controllers() const
	{
		return m_controllers;
	}

public:
	std::stringstream temp_string;

private:
	std::unique_ptr<Abstract_rb_engine> m_physical_world{ nullptr };

	Character* m_controlled_character;

	My_qopengl_widget* m_gl_widget;

	std::vector<std::shared_ptr<Controller_interface>> m_controllers;

	std::shared_ptr<Articulated_figure> m_desired_pose_handler{ nullptr };

	std::shared_ptr<Articulated_figure> m_reference_handler{ nullptr };

	std::shared_ptr<Gait_analyzer> m_gait_analyzer{ nullptr };

	std::vector<Ref_trajectories> m_reference_trajectories;

	double m_walking_cycle_time{ 0 }; //TODO : Fix it

	size_t m_walking_cycle{ 0 };

	bool m_simulation_success{ true };

	Vector3d m_ref_traj_correction{ Vector3d(0,0,0) };

	std::vector<double> m_results{};

	size_t m_sample_id{ 0 };
};
