#include "SimBiCon_framework.h"
#include "Ode_world.h"
#include "ConUtils.h"
#include "Reduced_character_state.h"
#include "Muscle_controller.h"
#include "Pelvis_pose_control.h"
#include "Swing_foot_controller.h"
#include "Balance_control.h"
#include "QtGui.h"
#include "Swing_foot_controller.h"


SimBiCon_framework::SimBiCon_framework(std::string& conF_filename)
{
	m_physical_world = std::make_unique<Ode_world>();

	FILE* f;
	const auto err = fopen_s(&f, conF_filename.c_str(), "r");
	if (err != 0 || f == nullptr)
		throw std::logic_error("Could not open file: " + std::string(conF_filename));


	//have a temporary buffer used to read the file line by line...
	char buffer[200];
	//this is where it happens.
	while (!feof(f))
	{
		//get a line from the file...
		fgets(buffer, 200, f);
		if (feof(f))
			break;
		if (strlen(buffer) > 195)
			throw std::length_error("The input file contains a line that is longer than ~200 characters - not allowed");
		auto line = lTrim(buffer);
		const auto line_type = get_con_line_type(line);
		switch (line_type)
		{
		case con_utils::load_af_from_xml:
		{
			auto xml_input = load(trim(line));
			m_physical_world->load_af_from_struct(xml_input);
			m_controlled_character = m_physical_world->get_character().get();
			break;
		}
		case con_utils::load_rb_file:
		{
			m_physical_world->load_rbs_from_file(trim(line));
			break;
		}
		case con_utils::load_simbicon:
		{
			m_controllers.push_back(std::make_shared<SimBiCon>(trim(line), *m_controlled_character, *this));
			break;
		}
		case con_utils::load_ref_trajectory_file:
			load_ref_trajectory(trim(line));
			m_reference_handler = std::make_shared<Articulated_figure>(*m_controlled_character);
			break;
		case con_utils::load_pelvis_pose_controller:
		{
			load_pelvis_pose_controller(trim(line));
			break;
		}
		case con_utils::load_swing_foot_controller:
			m_controllers.push_back(std::make_shared<Swing_foot_controller>(*m_controlled_character, *this));
			break;
		case con_utils::not_important:
		case con_utils::comment:
			break;
		default:
			throw std::logic_error("Incorrect SIMBICON input file: " + std::string(buffer) + " - unexpected line.");
		}
	}
	fclose(f);

	if (auto ode_world = dynamic_cast<Ode_world*>(m_physical_world.get()))
	{
		ode_world->link_simbicon_with_ode();
		init_controlled_character_state();
	}

	const auto simbicon = std::dynamic_pointer_cast<SimBiCon>(m_controllers[0]);
	m_walking_cycle_time = simbicon->get_starting_time();

	m_desired_pose_handler = std::make_shared<Articulated_figure>(*m_controllers[0]->get_controlled_character());
	std::dynamic_pointer_cast<SimBiCon>(m_controllers[0])->compute_desired_pose();

	init_gait_analyzer();

	update_desired_pos_handler();

	update_ref_traj_correction();
	update_reference_trajectories();

	for (auto& arb : std::dynamic_pointer_cast<SimBiCon>(m_controllers[0])->get_controlled_character()->get_arbs())
	{
		arb->notify();
	}
}


SimBiCon_framework::~SimBiCon_framework()
{
}

SimBiCon_framework::SimBiCon_framework(const SimBiCon_framework& other)
{
	//create the physical world
	m_physical_world = std::make_unique<Ode_world>(*dynamic_cast<Ode_world*>(other.m_physical_world.get()));
	m_controlled_character = m_physical_world->get_character().get();

	//ref trajectories
	m_reference_trajectories = other.m_reference_trajectories;
	m_ref_traj_correction = other.m_ref_traj_correction;

	//reference handler
	m_walking_cycle_time = other.m_walking_cycle_time;
	m_reference_handler = std::make_shared<Articulated_figure>(*m_physical_world->get_character());

	//controller
	for (const auto& controller : other.m_controllers)
	{
		switch (controller->get_type()) {
		case Controller_interface::undefined: break;
		case Controller_interface::muscular_controller:
		{
			m_controllers.push_back(std::make_shared<Muscle_controller>(*m_controllers[0]->get_controlled_character(), *this));
			break;
		}
		case Controller_interface::simbicon:
		{
			m_controllers.push_back(std::make_shared<SimBiCon>(*std::dynamic_pointer_cast<SimBiCon>(other.m_controllers[0]), *m_physical_world->get_character(), *this));
			break;
		}
		case Controller_interface::pelvis_pose_controller:
		{
			auto temp_controller = std::make_shared<Pelvis_pose_control>(*m_controllers[0]->get_controlled_character(), *this);
			temp_controller->set_kp_f(std::dynamic_pointer_cast<Pelvis_pose_control>(controller)->get_kp_f());
			temp_controller->set_kd_f(std::dynamic_pointer_cast<Pelvis_pose_control>(controller)->get_kd_f());
			temp_controller->set_kp_t(std::dynamic_pointer_cast<Pelvis_pose_control>(controller)->get_kp_t());
			temp_controller->set_kd_t(std::dynamic_pointer_cast<Pelvis_pose_control>(controller)->get_kd_t());
			m_controllers.push_back(temp_controller);
			break;
		}
		case Controller_interface::balance_controller:
		{
			m_controllers.push_back(std::make_shared<Balance_control>(*m_controllers[0]->get_controlled_character(), *this));
			break;
		}

		default:;
		}
	}

	if (auto ode_world = dynamic_cast<Ode_world*>(m_physical_world.get()))
	{
		ode_world->link_simbicon_with_ode();
		std::vector<double> state;
		other.get_physical_world()->get_character()->get_direct_state(state);
		get_physical_world()->get_character()->set_direct_state(state);
		get_physical_world()->get_character()->save_previous_state();

		for (auto& arb : m_physical_world->get_character()->get_arbs())
		{
			arb->notify();
		}
	}



	m_desired_pose_handler = std::make_shared<Articulated_figure>(*m_controllers[0]->get_controlled_character());
	std::dynamic_pointer_cast<SimBiCon>(m_controllers[0])->compute_desired_pose();

	init_gait_analyzer();

	update_desired_pos_handler();

	//update_ref_traj_correction();
	update_reference_trajectories();

	for (auto& arb : std::dynamic_pointer_cast<SimBiCon>(m_controllers[0])->get_controlled_character()->get_arbs())
	{
		arb->notify();
	}
}

void SimBiCon_framework::load_ref_trajectory(char* ref_motion_filename)
{
	//now we'll interpret the input file...
	if (ref_motion_filename == nullptr)
		throw std::logic_error("NULL file name provided.");
	FILE* f;
	const auto err = fopen_s(&f, ref_motion_filename, "r");
	if (err != 0 || f == nullptr)
		throw std::logic_error("Could not open file: " + std::string(ref_motion_filename));

	//have a temporary buffer used to read the file line by line...
	char buffer[200];

	std::string temp_name;

	while (!feof(f))
	{
		//get a line from the file...
		fgets(buffer, 200, f);
		if (feof(f))
			break;
		if (strlen(buffer) > 195)
			throw std::logic_error("The input file contains a line that is longer than ~200 characters - not allowed");
		auto line = lTrim(buffer);
		const auto line_type = get_con_line_type(line);
		switch (line_type)
		{
		case con_utils::con_trajectory_start:
			temp_name = trim(line);
			break;
		case con_utils::con_traj_component:
		{
			m_reference_trajectories.emplace_back();
			m_reference_trajectories.back().object_name = std::string(temp_name);
			m_reference_trajectories.back().object_id = m_physical_world->get_character()->get_object_id(std::string(temp_name));
			break;
		}
		case con_utils::con_traj_component_end:
		case con_utils::con_trajectory_end:
			break;
		case con_utils::con_base_trajectory_start:
		{
			Trajectory1D tempTraj;
			Fsm_state::read_trajectory1D(f, tempTraj, con_utils::con_base_trajectory_end);
			m_reference_trajectories.back().values = Trajectory1D(tempTraj);
			break;
		}
		case con_utils::con_rotation_axis:
		{
			double x, y, z;
			if (sscanf(line, "%lf %lf %lf", &x, &y, &z) != 3)
				throw std::logic_error("The axis for a trajectory is specified by three parameters!");
			m_reference_trajectories.back().rotation_axis = QVector3D(x, y, z);
			break;
		}
		case con_utils::con_transition_axis:
		{
			double x, y, z;
			if (sscanf(line, "%lf %lf %lf", &x, &y, &z) != 3)
				throw std::logic_error("The axis for a trajectory is specified by three parameters!");
			m_reference_trajectories.back().translation_axis = QVector3D(x, y, z);
			break;
		}
		case con_utils::not_important:
		case con_utils::comment:
			break;
		default:
			throw std::logic_error("Incorrect SIMBICON input file: " + std::string(buffer) + " - unexpected line.");
		}
	}
}

void SimBiCon_framework::load_pelvis_pose_controller(char* input)
{
	auto temp_controller = std::make_shared<Pelvis_pose_control>(*m_controllers[0]->get_controlled_character(), *this);

	double kp1, kp2;
	double kd1, kd2;

	sscanf(input, "%lf %lf %lf %lf", &kp1, &kd1, &kp2, &kd2);

	temp_controller->set_kp_f(kp1);
	temp_controller->set_kd_f(kd1);
	temp_controller->set_kp_t(kp2);
	temp_controller->set_kd_t(kd2);

	m_controllers.push_back(temp_controller);
}

void SimBiCon_framework::init_gait_analyzer()
{
	m_gait_analyzer = std::make_shared<Gait_analyzer>(*this);

	//m_reference_handler
	m_reference_handler->add_observer(m_gait_analyzer.get());

	//main
	m_controllers[0]->get_controlled_character()->add_observer(m_gait_analyzer.get());
}

//this method is used to advance the simulation. Typically, we will first compute the control, and then we will take one
//simulation step. If we are to apply control at this point in the simulation, we can either use a controller to recompute it,
//or we can use the values that were set before.
void SimBiCon_framework::global_step(const double dt)
{
	m_desired_pose_handler->save_previous_state();

	//compute target pose for next step (controller must be order)
	for (const auto& controller : m_controllers)
	{
		controller->pre_simulation_step_phase_1();
	}

	for (const auto& controller : m_controllers)
	{
		controller->pre_simulation_step_phase_2();
	}

	for (const auto& controller : m_controllers)
	{
		controller->pre_simulation_step_phase_3();
	}

	// advance the state and time of the reference_handler
	m_reference_handler->save_previous_state();
	update_walking_cycle_time();
	update_reference_trajectories();
	//std::cout << "ref : " << m_reference_handler->get_arb_by_name("pelvis")->get_cm_velocity() << std::endl;
	//std::cout << "arb_linear_velocity : " << m_physical_world->get_character()->get_arb_by_name("pelvis")->get_cm_velocity() << std::endl;

	//Advance the simulation (apply torque or muscle torque)
	m_physical_world->engine_simulation_step();
	for (auto& arb : m_physical_world->get_character()->get_arbs())
	{
		arb->notify();
	}
	m_desired_pose_handler->notify();
	m_reference_handler->notify();
	m_physical_world->get_character()->notify();
	//std::cout << "Real com_vel : " << m_physical_world->get_character()->get_com_velocity() << std::endl;
	//std::cout << "Ref com_vel : " << m_reference_handler->get_com_velocity() << std::endl;

	//Check if change must be done
	for (const auto& controller : m_controllers)
	{
		controller->post_simulation_step();
	}

	if (SimGlobals::draw)
	{
		add_drawing();
	}
}

void SimBiCon_framework::init_controlled_character_state() const
{
	auto reference_trajectories = get_reference_trajectories();
	auto controlled_character = m_physical_world->get_character();
	const auto starting_time = std::dynamic_pointer_cast<SimBiCon>(m_controllers[0])->get_starting_time();

	Reduced_character_state temp_handler;

	temp_handler.set_root_orientation(QQuaternion(1, 0, 0, 0));
	temp_handler.set_root_angular_velocity(Vector3d(0, 0, 0));
	temp_handler.set_root_position(Vector3d(0, 0, 0));
	temp_handler.set_root_velocity(Vector3d(0, 0, 0));
	for (size_t i = 0; i < controlled_character->get_joints().size(); i++)
	{
		temp_handler.set_joint_relative_orientation(QQuaternion(1, 0, 0, 0), i);
		temp_handler.set_joint_relative_ang_velocity(Vector3d(), i);
	}

	for (auto& reference_trajectory : reference_trajectories)
	{
		if (reference_trajectory.rotation_axis == QVector3D(0, 0, 0))
		{
			const auto pos_value = reference_trajectory.values.evaluate_catmull_rom(
				starting_time);

			auto temp_pos = Vector3d(reference_trajectory.translation_axis[0] * pos_value,
				reference_trajectory.translation_axis[1] * pos_value,
				reference_trajectory.translation_axis[2] * pos_value);

			temp_handler.set_root_position(temp_handler.get_root_position() + temp_pos);


			const auto vel_value = reference_trajectory.values.evaluate_catmull_rom_derivative(
				starting_time);

			auto temp_vel = Vector3d(reference_trajectory.translation_axis[0] * vel_value,
				reference_trajectory.translation_axis[1] * vel_value,
				reference_trajectory.translation_axis[2] * vel_value);

			temp_handler.set_root_velocity(temp_handler.get_root_velocity() + temp_vel);
			continue;
		}

		if (reference_trajectory.object_name == "pelvis")
		{
			const auto ang_value = reference_trajectory.values.evaluate_catmull_rom(
				starting_time) * 180 / M_PI;

			auto temp = temp_handler.get_root_orientation().to_QQuaternion() * QQuaternion::fromAxisAndAngle(reference_trajectory.rotation_axis, ang_value);

			temp_handler.set_root_orientation(temp.normalized());

			const auto vel_value = reference_trajectory.values.evaluate_catmull_rom_derivative(
				starting_time);

			auto temp_vel = Vector3d(reference_trajectory.rotation_axis[0] * vel_value,
				reference_trajectory.rotation_axis[1] * vel_value,
				reference_trajectory.rotation_axis[2] * vel_value);

			temp_handler.set_root_angular_velocity(temp_handler.get_root_angular_velocity() + temp_vel);
			continue;
		}

		//angular_values
		const auto ang_value = reference_trajectory.values.evaluate_catmull_rom(
			starting_time) * 180 / M_PI;

		auto temp = temp_handler.get_joint_relative_orientation(reference_trajectory.object_id).to_QQuaternion() *
			QQuaternion::fromAxisAndAngle(reference_trajectory.rotation_axis, ang_value);

		temp_handler.set_joint_relative_orientation(temp.normalized(), reference_trajectory.object_id);

		//angular_velocities
		const auto vel_value = reference_trajectory.values.evaluate_catmull_rom_derivative(
			starting_time);

		auto temp_vel = Vector3d(reference_trajectory.rotation_axis[0] * vel_value,
			reference_trajectory.rotation_axis[1] * vel_value,
			reference_trajectory.rotation_axis[2] * vel_value);

		temp_handler.set_joint_relative_ang_velocity(
			temp_handler.get_joint_relative_ang_velocity(reference_trajectory.object_id) + temp_vel,
			reference_trajectory.object_id);

	}
	controlled_character->set_state(temp_handler.get_state());
	controlled_character->save_previous_state();

	for (auto& arb : m_physical_world->get_character()->get_arbs())
	{
		arb->notify();
	}
}

std::shared_ptr<Swing_foot_controller> SimBiCon_framework::get_swing_foot_controller()
{
	for (auto& controller : m_controllers)
	{
		if (controller->get_type() == Controller_interface::swing_foot_controller)
		{
			return std::dynamic_pointer_cast<Swing_foot_controller>(controller);
		}
	}
	return nullptr;
}

void SimBiCon_framework::add_drawing()
{
	//TODO : Enable if this SimBiCon_framework is link with a OpenGL windows
	const auto& character = m_controllers[0]->get_controlled_character();
	m_gl_widget->clear_temp_object();
	m_gl_widget->add_sphere(character->get_cop());
	m_gl_widget->add_sphere(character->get_com(), { 0,0,0 }, { 0.03,0.03,0.03 });

	if(get_swing_foot_controller())
	{
		m_gl_widget->add_sphere(get_swing_foot_controller()->step_location, { 255,0,0 });
	}

	for (const auto& contact_point : character->get_contact_points())
	{
		Quaternion orientation;
		Vector3d force = contact_point->f;
		auto axis = force.cross_product_with(SimGlobals::up).toUnit();
		const auto angle = force.angle_with(SimGlobals::up);
		orientation.set_to_rotation_quaternion(angle, axis);
		m_gl_widget->add_arrow(Vector3d(contact_point->cp), orientation, force.length()*1e-3);
	}

	//COM velocity
	Quaternion orientation;
	Vector3d force = character->get_com_velocity();
	auto axis = force.cross_product_with(SimGlobals::up).toUnit();
	const auto angle = force.angle_with(SimGlobals::up);
	orientation.set_to_rotation_quaternion(angle, axis);
	m_gl_widget->add_arrow(Vector3d(character->get_com()), orientation.get_inverse(), force.length()*1e-1);
}

void SimBiCon_framework::update_desired_pos_handler()
{
	std::dynamic_pointer_cast<SimBiCon>(m_controllers[0])->update_desired_pos();
}

void SimBiCon_framework::update_walking_cycle_time()
{
	auto const t_max = m_reference_trajectories.back().values.getMaxPosition();
	if (m_walking_cycle_time <= t_max)
	{
		m_walking_cycle_time += SimGlobals::dt;
	}
	else
	{
		m_walking_cycle_time -= t_max;
		m_walking_cycle += 1;
	}
}

void SimBiCon_framework::update_ref_traj_correction()
{
	Vector3d temp_pos;
	for (auto reference_trajectory : m_reference_trajectories)
	{
		if (reference_trajectory.rotation_axis == QVector3D(0, 0, 0) && reference_trajectory.object_name == "pelvis")
		{
			const auto pos_value = reference_trajectory.values.evaluate_catmull_rom(m_walking_cycle_time);

			temp_pos += Vector3d(reference_trajectory.translation_axis[0] * pos_value,
				reference_trajectory.translation_axis[1] * pos_value,
				reference_trajectory.translation_axis[2] * pos_value);
		}
	}
	switch (m_controllers[0]->get_controlled_character()->get_stance())
	{
	case Character::right:
	case Character::left:
	{
		const auto pelvis_cm = m_controllers[0]->get_controlled_character()->get_arb_by_name("pelvis")->get_cm_position();
		m_ref_traj_correction = pelvis_cm - temp_pos;
		break;
	}
	case Character::both:
	case Character::none:
	default:;
	}
}

void SimBiCon_framework::update_reference_trajectories()
{
	Reduced_character_state handler;

	Vector3d temp_pos_start;
	for (auto reference_trajectory : m_reference_trajectories)
	{
		if (reference_trajectory.rotation_axis == QVector3D(0, 0, 0) && reference_trajectory.object_name == "pelvis")
		{
			const auto pos_value = reference_trajectory.values.evaluate_catmull_rom(reference_trajectory.values.getMinPosition());

			temp_pos_start += Vector3d(reference_trajectory.translation_axis[0] * pos_value,
				reference_trajectory.translation_axis[1] * pos_value,
				reference_trajectory.translation_axis[2] * pos_value);
		}
	}

	Vector3d temp_pos_end;
	for (auto reference_trajectory : m_reference_trajectories)
	{
		if (reference_trajectory.rotation_axis == QVector3D(0, 0, 0) && reference_trajectory.object_name == "pelvis")
		{
			const auto pos_value = reference_trajectory.values.evaluate_catmull_rom(reference_trajectory.values.getMaxPosition());

			temp_pos_end += Vector3d(reference_trajectory.translation_axis[0] * pos_value,
				reference_trajectory.translation_axis[1] * pos_value,
				reference_trajectory.translation_axis[2] * pos_value);
		}
	}

	const auto diff_start_to_end = temp_pos_end - temp_pos_start;

	handler.set_root_orientation(QQuaternion(1, 0, 0, 0));
	handler.set_root_angular_velocity(Vector3d(0, 0, 0));
	handler.set_root_position(m_ref_traj_correction + diff_start_to_end * m_walking_cycle);
	handler.set_root_velocity(Vector3d(0, 0, 0));
	for (size_t i = 0; i < m_controllers[0]->get_controlled_character()->get_joints().size(); i++)
	{
		handler.set_joint_relative_orientation(QQuaternion(1, 0, 0, 0), i);
		handler.set_joint_relative_ang_velocity(Vector3d(), i);
	}
	for (auto& reference_trajectory : m_reference_trajectories)
	{
		if (reference_trajectory.rotation_axis == QVector3D(0, 0, 0))
		{
			const auto pos_value = reference_trajectory.values.evaluate_catmull_rom(
				m_walking_cycle_time);

			auto temp_pos = Vector3d(reference_trajectory.translation_axis[0] * pos_value,
				reference_trajectory.translation_axis[1] * pos_value,
				reference_trajectory.translation_axis[2] * pos_value);

			handler.set_root_position(handler.get_root_position() + temp_pos);


			const auto vel_value = reference_trajectory.values.evaluate_catmull_rom_derivative(
				m_walking_cycle_time);

			auto temp_vel = Vector3d(reference_trajectory.translation_axis[0] * vel_value,
				reference_trajectory.translation_axis[1] * vel_value,
				reference_trajectory.translation_axis[2] * vel_value);

			handler.set_root_velocity(handler.get_root_velocity() + temp_vel);
			continue;
		}

		if (reference_trajectory.object_name == "pelvis")
		{
			const auto ang_value = reference_trajectory.values.evaluate_catmull_rom(
				m_walking_cycle_time) * 180 / M_PI;

			auto temp = handler.get_root_orientation().to_QQuaternion() * QQuaternion::fromAxisAndAngle(reference_trajectory.rotation_axis, ang_value);

			handler.set_root_orientation(temp.normalized());

			const auto vel_value = reference_trajectory.values.evaluate_catmull_rom_derivative(
				m_walking_cycle_time);

			auto temp_vel = Vector3d(reference_trajectory.rotation_axis[0] * vel_value,
				reference_trajectory.rotation_axis[1] * vel_value,
				reference_trajectory.rotation_axis[2] * vel_value);

			handler.set_root_angular_velocity(handler.get_root_angular_velocity() + temp_vel);
			continue;
		}


		const auto ang_value = reference_trajectory.values.evaluate_catmull_rom(
			m_walking_cycle_time) * 180 / M_PI;

		auto temp = handler.get_joint_relative_orientation(reference_trajectory.object_id).to_QQuaternion() *
			QQuaternion::fromAxisAndAngle(reference_trajectory.rotation_axis, ang_value);

		handler.set_joint_relative_orientation(temp.normalized(), reference_trajectory.object_id);

		const auto vel_value = reference_trajectory.values.evaluate_catmull_rom_derivative(
			m_walking_cycle_time);

		auto temp_vel = Vector3d(reference_trajectory.rotation_axis[0] * vel_value,
			reference_trajectory.rotation_axis[1] * vel_value,
			reference_trajectory.rotation_axis[2] * vel_value);

		handler.set_joint_relative_ang_velocity(
			handler.get_joint_relative_ang_velocity(reference_trajectory.object_id) + temp_vel,
			reference_trajectory.object_id);

	}

	m_reference_handler->set_state(handler.get_state());

	for (auto& arb : m_reference_handler->get_arbs())
	{
		arb->notify();
	}
}