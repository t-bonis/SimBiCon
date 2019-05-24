#include "Learning_framework.h"
#include <random>



Learning_framework::Learning_framework(std::string& input)
{
	m_default_simbicon_framework = std::make_shared<SimBiCon_framework>(input);
	SimGlobals::draw = false;
	SimGlobals::learning = true;
	m_windows_size = 10;
	m_windows_start = 0;
	m_target_simulation_time = 1000;
	compute_intervals();
}

void Learning_framework::compute_intervals()
{
	m_intervals.clear();
	const auto simbicon = std::dynamic_pointer_cast<SimBiCon>(m_default_simbicon_framework->get_controllers()[0]);
	auto fsm_states = simbicon->get_fsm_states();
	const auto state_id = simbicon->get_fsm_state_id();
	const auto starting_phi = simbicon->get_phase();

	auto trajectories = fsm_states[state_id]->get_trajectories();
	const auto state_duration = fsm_states[state_id]->get_state_duration();
	m_intervals.push_back(starting_phi * state_duration);
	const auto first_knot_id = trajectories[0]->components[0]->base_trj.get_first_larger_index(starting_phi);
	knot_ptr current_knot{ state_id , 0 , 0, first_knot_id };

	for (size_t i = 0; i < m_windows_size; ++i)
	{
		auto base_trajectory = fsm_states[current_knot.state_id]
			->get_trajectory(current_knot.trajectory_id)
			->components[current_knot.component_id]
			->base_trj;

		m_intervals.push_back(base_trajectory.getKnotPosition(current_knot.knot_id)* state_duration);
		current_knot = get_next_knot(fsm_states, current_knot);
	}
	const auto starting_time = m_intervals[0];

	for (auto& time : m_intervals)
	{
		time -= starting_time;
		time /= SimGlobals::dt;
		time = int(time);
	}
}


Learning_framework::~Learning_framework()
{
	for (auto& sim_thread : m_simulation_threads)
	{
		if (sim_thread->isRunning())
		{
			sim_thread->stop_simulation();
			sim_thread->requestInterruption();
			sim_thread->wait();
		}
	}
	if (m_analyzer_thread)
	{
		m_analyzer_thread->wait();
	}
}

void Learning_framework::load_learning_parameters(std::string& f_name)
{
	const auto dim = 15; // must be change manually (number of parameter to optimize (time is 100*dim to 300*dim^2))

	double x_start[dim];
	for (auto& i : x_start)
	{
		i = 0;
	}

	double std_dev[dim];
	for (auto& i : std_dev)
	{
		i = 0.01;
	}

	for (size_t i = 0; i < m_windows_size; i++)
	{
		auto cma = std::make_shared<Cma_object>(m_lambda, dim, x_start, std_dev);
		m_cma.push_back(cma);
	}
	m_max_cma_to_update = m_cma.size();
}

void Learning_framework::assign_values(SimBiCon_framework& available_framework, size_t sample_to_eval)
{
	available_framework.set_sample_id(sample_to_eval);
	const auto simbicon = std::dynamic_pointer_cast<SimBiCon>(available_framework.get_controllers()[0]);
	auto fsm_states = simbicon->get_fsm_states();
	const auto state_id = simbicon->get_fsm_state_id();
	const auto starting_phi = simbicon->get_phase();

	size_t param_nb = 0;
	auto trajectories = fsm_states[state_id]->get_trajectories();
	for (size_t trajectory_id = 0; trajectory_id < trajectories.size(); ++trajectory_id)
	{
		if (trajectories[trajectory_id]->joint_name == "root")
		{
			continue;
		}
		auto components = trajectories[trajectory_id]->components;
		for (size_t component_id = 0; component_id < components.size(); ++component_id)
		{
			const auto first_knot_id = components[component_id]->base_trj.get_first_larger_index(starting_phi);
			knot_ptr current_knot{ state_id , trajectory_id , component_id, first_knot_id };
			for (size_t i = 0; i < m_windows_size; ++i)
			{
				auto base_trajectory = &fsm_states[current_knot.state_id]
					->get_trajectory(current_knot.trajectory_id)
					->components[current_knot.component_id]
					->base_trj;
				const auto default_value = base_trajectory->getKnotValue(current_knot.knot_id);
				base_trajectory->setKnotValue(current_knot.knot_id, default_value + m_cma[i]->pop[sample_to_eval][param_nb]);
				current_knot = get_next_knot(fsm_states, current_knot);
			}
			param_nb++;
		}
	}
}

knot_ptr Learning_framework::get_next_knot(std::vector<std::shared_ptr<Fsm_state>> fsm_states, knot_ptr current_knot)
{
	auto base_trajectory = fsm_states[current_knot.state_id]->get_trajectory(current_knot.trajectory_id)->components[current_knot.component_id]->base_trj;
	if (base_trajectory.get_knot_count() > current_knot.knot_id)
	{
		current_knot.knot_id++;
		return current_knot;
	}
	else
	{
		current_knot.state_id = fsm_states[current_knot.state_id]->get_next_state_index();
		return current_knot;
	}

}

void Learning_framework::eval_simbicon_framework(SimBiCon_framework& simbicon_framework)
{
	for (auto i = m_simulation_threads.begin(); i != m_simulation_threads.end(); ++i)
	{
		if (i[0]->isFinished())
		{
			m_simulation_threads.erase(i);
			break;
		}
	}

	m_simulation_threads.push_back(std::make_shared<Simulation_thread>(simbicon_framework, true, m_intervals.back() - m_intervals.front()));
	connect(m_simulation_threads.back().get(), &Simulation_thread::simulation_done, this,
		&Learning_framework::simulation_done);
	m_simulation_threads.back()->start();
	//std::cout << "### Start eval ###\n";
}

void Learning_framework::start_learning()
{
	for (auto& cma : m_cma)
	{
		cma->evo.samplePopulation();
	}
	for (auto i = 0; i < SimGlobals::nb_threads; ++i)
	{
		next_eval();
	}
}


SimBiCon_framework* Learning_framework::create_simbicon_framework()
{
	//Load Model
	try
	{
		auto simbicon_framework = std::make_shared<SimBiCon_framework>(*m_default_simbicon_framework);
		//Setup OpenGLWindow for rendering
		m_simbicon_frameworks.push_back(simbicon_framework);
		return simbicon_framework.get();
	}
	catch (const std::exception& err)
	{
		std::cerr << "Error during model loading : " << err.what();
		return nullptr;
	}
}

bool Learning_framework::is_feasible(double* const p, Cma_object* cma) const
{

	//for (auto i = 0; i < cma->parameters.N; i++)
	//{
	//	//if (p[i] <= 0)
	//	//{
	//	//	return false;
	//	//}
	//}
	return true;
}

void Learning_framework::next_eval()
{
	//std::cout << "S"<< m_next_sample_to_eval;
	if (!((m_next_sample_to_eval+1) % 500))
	{
		std::cout << "-";
	}
	auto simbicon_available = create_simbicon_framework();
	assign_values(*simbicon_available, m_next_sample_to_eval);
	m_next_sample_to_eval++;
	eval_simbicon_framework(*simbicon_available);
}

void Learning_framework::next_pop()
{
	auto advance_windows = 0;
	std::cout << "\n### Setup_next_pop ###\n";
	bool previous_terminated = true;
	for (size_t i = 0; i < m_max_cma_to_update; ++i)
	{
		m_cma[i]->evo.updateDistribution(m_cma[i]->fitvals);
		std::cout << m_cma[i]->evo.get(m_cma[i]->evo.Generation) << " " << m_cma[i]->evo.get(m_cma[i]->evo.Sigma) << " " << m_cma[i]->evo.get(m_cma[i]->evo.Fitness) << std::endl;
		if(m_cma[i]->evo.testForTermination() && previous_terminated)
		{
			std::cout << "cma " << i << " : " << m_cma[i]->evo.getStopMessage();
			++advance_windows;
		}
		else
		{
			previous_terminated = false;
		}
	}
	//TODO update : update intervals, simbicon_default_start, cma
	if(advance_windows> 0)
	{//TODO : run_default simbicon for advance_windows
		std::cout << advance_windows << std::endl;
		
		m_simulation_threads.push_back(std::make_shared<Simulation_thread>(*m_default_simbicon_framework, true, m_intervals[advance_windows]));
		m_simulation_threads.back()->start();
		m_simulation_threads.back()->wait();
		m_simulation_threads.erase(m_simulation_threads.end());

		//Increase reconstructed time
		m_reconstructed_time += m_intervals[1];
		if (m_reconstructed_time > m_target_simulation_time)
		{
			end_reconstruction();
		}

		//update intervals (then must stay the same)
		compute_intervals();
		
		//create new cma for new intervals
		const auto dim = 15; // must be change manually (number of parameter to optimize (time is 100*dim to 300*dim^2))

		double x_start[dim];
		for (auto& i : x_start)
		{
			i = 0;
		}

		double std_dev[dim];
		for (auto& i : std_dev)
		{
			i = 0.01;
		}

		for(auto i = 0; i < advance_windows; ++i)
		{
			m_cma.erase(m_cma.begin());
			auto cma = std::make_shared<Cma_object>(m_lambda, dim, x_start, std_dev);
			m_cma.push_back(cma);
		}
	}

	//Restart for a new pop.
	compute_intervals();
	m_max_cma_to_update = m_cma.size();
	m_next_sample_to_eval = 0;
	m_sample_evaluated = 0;
	++m_current_pop;
	start_learning();	
}



void Learning_framework::simulation_done(SimBiCon_framework* simbicon_framework)
{
	if (m_lambda > m_next_sample_to_eval)
	{
		next_eval();
	}
	if (m_analyzer_thread)
	{
		m_analyzer_thread->wait();
	}
	m_analyzer_thread = std::make_shared<Analyzer_thread>(*simbicon_framework, *this);
	connect(m_analyzer_thread.get(), &Analyzer_thread::analyze_done, this, &Learning_framework::analyze_done);
	m_analyzer_thread->start();
	//std::cout << "### Simulation done ###\n";

}

void Learning_framework::analyze_done(SimBiCon_framework* simbicon_framework)
{
	size_t i = 0;
	for (auto& cma : m_cma)
	{
		if (i < simbicon_framework->get_result().size())
		{
			cma->fitvals[simbicon_framework->get_sample_id()] = simbicon_framework->get_result()[i];
		}
		else
		{
			m_max_cma_to_update = i;
			m_intervals.resize(i+1);
			break;
		}
		++i;
	}
	//simbicon_framework->temp_string << simbicon_framework->get_result();
	//BOOST_LOG_TRIVIAL(trace) << simbicon_framework->temp_string.str();
	auto j = m_simbicon_frameworks.begin();
	for (auto& sim_bi : m_simbicon_frameworks)
	{
		if (sim_bi.get() == simbicon_framework)
		{
			break;
		}
		++j;
	}
	m_simbicon_frameworks.erase(j);
	//std::cout << "### Analyze done ### " << m_lambda << " " << m_sample_evaluated <<"\n";
	m_sample_evaluated++;
	if (m_lambda == m_sample_evaluated)
	{
		next_pop();
	}

}

void Learning_framework::end_reconstruction()
{
	const auto simbicon = std::dynamic_pointer_cast<SimBiCon>(m_default_simbicon_framework->get_controllers()[0]);
	auto fsm_states = simbicon->get_fsm_states();
	for (auto& fsm_state : fsm_states)
	{
		FILE* file = fopen("../../out/result.txt", "w");
		fsm_state->write_state(file, 0);
	}
}