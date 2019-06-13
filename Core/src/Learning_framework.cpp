#include "Learning_framework.h"
#include <random>



Learning_framework::Learning_framework(std::string& input)
{
	if (!m_lambda % m_sample_to_save)
	{
		throw std::logic_error("Lambda must be a multiple of sample to save");
	}
	m_all_cost.resize(1);
	m_default_simbicon_framework = std::make_shared<SimBiCon_framework>(input);
	SimGlobals::draw = false;
	SimGlobals::learning = true;
	m_target_simulation_time = 1500;
	compute_intervals();

	for (auto& i : m_x_start)
	{
		i = 0;
	}

	for (auto& i : m_std_dev)
	{
		i = 0.1;
	}

	for (auto i = 0; i < m_target_nb_iterations; ++i)
	{
		auto cma = std::make_shared<Cma_object>(m_lambda, m_dim, m_x_start.data(), m_std_dev.data());
		m_cma.push_back(cma);
	}
}

void Learning_framework::compute_intervals()
{
	m_intervals.clear();
	const auto simbicon = std::dynamic_pointer_cast<SimBiCon>(m_default_simbicon_framework->get_controllers()[0]);
	auto fsm_states = simbicon->get_fsm_states();
	const auto state_id = simbicon->get_fsm_state_id();
	auto previous_phi = simbicon->get_phase();


	auto trajectories = fsm_states[state_id]->get_trajectories();
	auto state_duration = fsm_states[state_id]->get_state_duration();
	const auto first_knot_id = trajectories[0]->components[0]->base_trj.get_first_larger_index(previous_phi);
	knot_ptr previous_knot{ state_id , 0 , 0, first_knot_id - 1 };
	m_intervals.push_back(0);

	for (size_t i = 0; i < m_target_nb_iterations; ++i)
	{
		auto current_knot = get_next_knot(fsm_states, previous_knot);
		auto base_trajectory = fsm_states[current_knot.state_id]
			->get_trajectory(current_knot.trajectory_id)
			->components[current_knot.component_id]
			->base_trj;

		auto current_phi = base_trajectory.getKnotPosition(current_knot.knot_id);

		if (previous_knot.state_id != current_knot.state_id)
		{
			previous_knot = current_knot;
			previous_phi = current_phi;
			state_duration = fsm_states[previous_knot.state_id]->get_state_duration();
			current_knot = get_next_knot(fsm_states, previous_knot);
			base_trajectory = fsm_states[current_knot.state_id]
				->get_trajectory(current_knot.trajectory_id)
				->components[current_knot.component_id]
				->base_trj;

			current_phi = base_trajectory.getKnotPosition(current_knot.knot_id);
		}
		m_intervals.push_back(m_intervals.back() + (current_phi - previous_phi) * state_duration);

		previous_knot = current_knot;
		previous_phi = current_phi;
	}

	for (auto& time : m_intervals)
	{
		time /= SimGlobals::dt;
		time = int(ceil(time));
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
			auto base_trajectory = &fsm_states[current_knot.state_id]
				->get_trajectory(current_knot.trajectory_id)
				->components[current_knot.component_id]
				->base_trj;
			const auto default_value = base_trajectory->getKnotValue(current_knot.knot_id);
			base_trajectory->setKnotValue(current_knot.knot_id, default_value + m_cma[m_iteration]->pop[sample_to_eval][param_nb]);
			current_knot = get_next_knot(fsm_states, current_knot);
			param_nb++;
		}
	}
}

void Learning_framework::assign_best_values(SimBiCon_framework& available_framework, size_t nomber_to_update)
{
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
			for (size_t i = 0; i < nomber_to_update; ++i)
			{
				auto base_trajectory = &fsm_states[current_knot.state_id]
					->get_trajectory(current_knot.trajectory_id)
					->components[current_knot.component_id]
					->base_trj;
				const auto default_value = base_trajectory->getKnotValue(current_knot.knot_id);

				base_trajectory->setKnotValue(current_knot.knot_id, default_value + m_cma[i]->evo.getPtr(m_cma[i]->evo.XBest)[param_nb]);
				current_knot = get_next_knot(fsm_states, current_knot);
			}
			param_nb++;
		}
	}
}

knot_ptr Learning_framework::get_next_knot(std::vector<std::shared_ptr<Fsm_state>> fsm_states, knot_ptr current_knot)
{
	auto base_trajectory = fsm_states[current_knot.state_id]->get_trajectory(current_knot.trajectory_id)->components[current_knot.component_id]->base_trj;
	if (base_trajectory.get_knot_count() > current_knot.knot_id + 1)
	{
		current_knot.knot_id++;
		return current_knot;
	}
	else
	{
		current_knot.state_id = fsm_states[current_knot.state_id]->get_next_state_index();
		current_knot.knot_id = 0;
		return current_knot;
	}

}

void Learning_framework::eval_simbicon_framework(SimBiCon_framework& simbicon_framework, size_t interval_begining, size_t nb_of_intervals)
{
	for (auto i = m_simulation_threads.begin(); i != m_simulation_threads.end(); ++i)
	{
		if (i[0]->isFinished())
		{
			m_simulation_threads.erase(i);
			break;
		}
	}

	m_simulation_threads.push_back(std::make_shared<Simulation_thread>(simbicon_framework, true, m_intervals[interval_begining + nb_of_intervals] - m_intervals[interval_begining]));
	connect(m_simulation_threads.back().get(), &Simulation_thread::simulation_done, this,
		&Learning_framework::simulation_done);
	m_simulation_threads.back()->start();
	//std::cout << "### Start eval ###\n";
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
	m_analyzer_thread = std::make_shared<Analyzer_thread>(*simbicon_framework);
	connect(m_analyzer_thread.get(), &Analyzer_thread::analyze_done, this, &Learning_framework::analyze_done);
	m_analyzer_thread->start();
	//std::cout << "### Simulation done ###\n";

}

void Learning_framework::analyze_done(SimBiCon_framework* simbicon_framework)
{
	size_t i = 0;

	if (!simbicon_framework->get_success())
	{
		m_next_saved_frameworks.clear();
		next_pop();
		return;
	}

	m_all_cost[m_iteration].push_back(simbicon_framework->get_result()[0]);

	auto rank = get_rank(simbicon_framework);

	if (rank < m_sample_to_save)
	{
		m_next_saved_frameworks.insert(m_next_saved_frameworks.begin() + rank, simbicon_framework);
		if (m_next_saved_frameworks.size() > m_sample_to_save)
		{
			delete_framework(m_next_saved_frameworks.back());
			m_next_saved_frameworks.pop_back();
		}
	}
	else
	{
		delete_framework(simbicon_framework);
	}

	m_sample_evaluated++;

	if (m_lambda == m_sample_evaluated)
	{
		next_iteration();
	}

}


SimBiCon_framework* Learning_framework::create_simbicon_framework(SimBiCon_framework* in_simbicon_framework)
{
	//Load Model
	try
	{
		auto out_simbicon_framework = std::make_shared<SimBiCon_framework>(*in_simbicon_framework);
		//Setup OpenGLWindow for rendering
		m_simbicon_frameworks.push_back(out_simbicon_framework);
		return out_simbicon_framework.get();
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

void Learning_framework::start_learning()
{
	for (auto& cma : m_cma)
	{
		cma->evo.samplePopulation();
	}
	m_timer = std::make_unique<QElapsedTimer>();
	m_timer->start();
	if (SimGlobals::nb_threads > m_lambda)
	{
		SimGlobals::nb_threads = m_lambda;
	}
	for (auto i = 0; i < SimGlobals::nb_threads; ++i)
	{
		next_eval();
	}
}

void Learning_framework::next_eval()
{
	//std::cout << "S"<< m_next_sample_to_eval;
	if (!(int(m_next_sample_to_eval + 1) % int(m_lambda / 4)))
	{
		std::cout << "-";
	}
	SimBiCon_framework* simbicon_available;
	if (m_iteration == 0)
	{
		simbicon_available = create_simbicon_framework(m_default_simbicon_framework.get());
	}
	else
	{
		simbicon_available = create_simbicon_framework(m_saved_frameworks[m_combinaison_history[m_iteration - 1][m_next_sample_to_eval].first]);
	}
	//TODO : assign value of the first sample
	assign_values(*simbicon_available, m_next_sample_to_eval);
	++m_next_sample_to_eval;
	eval_simbicon_framework(*simbicon_available, m_iteration, 1);

}

void Learning_framework::next_iteration()
{
	m_analyzer_thread->wait();
	std::cout << "Timer elapsed : " << m_timer->elapsed() << std::endl;
	m_timer->restart();
	//TODO : prepare the simu : order n_offsring from nSaved
	m_combinaison_history.resize(m_combinaison_history.size() + 1);
	auto nb_for_each = m_lambda / m_sample_to_save;
	for (size_t i = 0; i < m_sample_to_save; ++i)
	{
		for (size_t j = 0; j < nb_for_each; ++j)
		{
			m_combinaison_history[m_iteration].emplace_back(i, m_next_saved_frameworks[i]->get_sample_id());
		}
	}

	if (m_saved_frameworks.size() > 0)
	{
		for (auto& saved_framework : m_saved_frameworks)
		{
			delete_framework(saved_framework);
		}
		m_saved_frameworks.clear();
	}
	m_saved_frameworks = m_next_saved_frameworks;
	m_next_saved_frameworks.clear();

	++m_iteration;
	m_next_sample_to_eval = 0;
	m_sample_evaluated = 0;

	std::cout << std::endl;

	if (m_iteration == m_target_nb_iterations)
	{
		next_pop();
	}
	else
	{
		for (auto i = 0; i < SimGlobals::nb_threads; ++i)
		{
			m_all_cost.resize(m_iteration + 1);
			next_eval();
		}
	}
}


void Learning_framework::next_pop()
{
	auto advance_windows = 0;
	std::cout << "\n### Setup_next_pop ###\n";
	bool previous_terminated = true;

	//TODO : Assign value for CMA
	//The height of subtree
	//Create all trees
	auto combinaison_history_size = int(m_combinaison_history.size());
	std::vector<std::vector<double>> node_best_value;
	node_best_value.resize(m_all_cost.size());
	std::vector<std::vector<int>> node_subtree_height;
	node_subtree_height.resize(m_all_cost.size());

	//For the last iteration, best_cost is just cost
	node_best_value.back() = m_all_cost.back();
	for (int sample = 0; sample < int(m_all_cost.back().size()); ++sample)
	{
		node_subtree_height.back().push_back(0);
	}

	//For other, total cost is sum of cost + best next node cost
	//height is 1 + best next node height
	for(auto iteration = int(m_all_cost.size()) - 2; iteration >= 0; --iteration)
	{
		node_subtree_height[iteration].resize(m_lambda);
		node_best_value[iteration].resize(m_lambda);
		for (int sample = 0; sample < int(m_all_cost[iteration].size()); ++sample)
		{
			int best_heigth = -1;
			double best_cost = INFINITY;
			for (int next_sample = 0; next_sample < int(m_all_cost[iteration].size()); ++next_sample)
			{
				if (m_combinaison_history[iteration][next_sample].second == sample)
				{
					if (node_subtree_height[iteration + 1][next_sample] > best_heigth)
					{
						best_heigth = node_subtree_height[iteration + 1][next_sample];
					}
					if (node_best_value[iteration + 1][next_sample] < best_cost)
					{
						best_cost = node_best_value[iteration + 1][next_sample];
					}
				}
			}
			if (best_cost == INFINITY)
			{
				best_cost = 0;
			}
			node_subtree_height[iteration][sample] = best_heigth + 1;
			node_best_value[iteration][sample] = (best_cost* node_subtree_height[iteration][sample] + m_all_cost[iteration][sample])/
				(node_subtree_height[iteration][sample]+1);

		}
	}

	std::stringstream message;
	for (int iteration = 0; iteration < int(node_best_value.size()); ++iteration)
	{
		//message << "Iteration : " << iteration << std::endl;
		for (int sample = 0; sample < int(node_best_value[iteration].size()); ++sample)
		{
			auto cost = node_best_value[iteration][sample];
			auto height = ((m_target_nb_iterations - iteration) - node_subtree_height[iteration][sample] - 1) / double(m_target_nb_iterations - iteration);
			m_cma[iteration]->fitvals[sample] = cost + height;
			
			//std::cout << cost << " " << height << " " << m_cma[iteration]->fitvals[sample] << std::endl;
			//std::cout << "Iteration : " << iteration << " Sample : " << sample << " : " << node_best_value[iteration][sample] << " et " << node_subtree_height[iteration][sample] << std::endl;
			
			message << cost << " ";
		}
		message << std::endl;
	}
	BOOST_LOG_TRIVIAL(trace) << message.str();



	//TODO : update CMA distrib
	for (size_t i = 0; i < m_target_nb_iterations; ++i)
	{
		m_cma[i]->evo.updateDistribution(m_cma[i]->fitvals);
		print_state(m_cma[i]);
		if (m_cma[i]->evo.testForTermination() && previous_terminated)
		{
			std::stringstream message;
			message << "cma " << i << " : " << m_cma[i]->evo.getStopMessage() << std::endl;
			std::cout << message.str();
			//BOOST_LOG_TRIVIAL(trace) << message.str();
			++advance_windows;
		}
		else
		{
			previous_terminated = false;
		}
	}
	//TODO update : update intervals, simbicon_default_start, cma
	if (advance_windows > 0)
	{//TODO : run_default simbicon for advance_windows
		std::cout << "advance_windows : " << advance_windows << std::endl;
		//BOOST_LOG_TRIVIAL(trace) << "advance_windows : " << advance_windows << std::endl;
		assign_best_values(*m_default_simbicon_framework, advance_windows);
		m_simulation_threads.push_back(std::make_shared<Simulation_thread>(*m_default_simbicon_framework, true, m_intervals[advance_windows]));
		m_simulation_threads.back()->start();
		m_simulation_threads.back()->wait();
		m_analyzer_thread = std::make_shared<Analyzer_thread>(*m_default_simbicon_framework, true);
		m_analyzer_thread->start();
		m_analyzer_thread->wait();
		//Increase reconstructed time
		m_reconstructed_time += m_intervals[advance_windows];
		if (m_reconstructed_time > m_target_simulation_time)
		{
			end_reconstruction();
		}

		const auto simbicon = std::dynamic_pointer_cast<SimBiCon>(m_default_simbicon_framework->get_controllers()[0]);
		simbicon->write_state("../../out/state1.txt");

		//create new cma for new intervals
		for (auto i = 0; i < advance_windows; ++i)
		{
			m_cma.erase(m_cma.begin());
			auto cma = std::make_shared<Cma_object>(m_lambda, m_dim, m_x_start.data(), m_std_dev.data());
			m_cma.push_back(cma);
		}
	}

	//Restart for a new pop.
	compute_intervals();

	m_next_sample_to_eval = 0;
	m_sample_evaluated = 0;
	m_iteration = 0;
	m_all_cost.clear();
	m_all_cost.resize(1);
	++m_current_pop;
	start_learning();
}

void Learning_framework::print_state(const std::shared_ptr<Cma_object>& cma)
{
	std::stringstream message;
	message << std::setprecision(2) << cma->evo.get(cma->evo.Generation) << "->";
	for (auto i = 0; i < m_dim; ++i)
	{
		message << "(" << cma->evo.getPtr(cma->evo.XMean)[i] << ",";
		message << cma->evo.getPtr(cma->evo.StdDev)[i] << ");";
	}
	message << " B: " << cma->evo.get(cma->evo.Fitness);
	std::cout << message.str() << std::endl;
	//BOOST_LOG_TRIVIAL(trace) << message.str() << std::endl;
}

size_t Learning_framework::get_rank(SimBiCon_framework* simbicon_framework)
{
	size_t rank = 0;
	auto value = simbicon_framework->get_result()[0];
	int lower_bond = 0;
	int upper_bond = int(m_next_saved_frameworks.size()) - 1;
	while (lower_bond <= upper_bond)
	{
		rank = floor((lower_bond + upper_bond) / 2);
		auto current_value = m_next_saved_frameworks[rank]->get_result()[0];
		if (current_value < value)
		{
			lower_bond = rank + 1;
		}
		else if (current_value > value)
		{
			upper_bond = rank - 1;
		}
		else
		{
			return rank;
		}
	}
	return lower_bond;
}

void Learning_framework::delete_framework(SimBiCon_framework* simbicon_framework)
{
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
}


void Learning_framework::end_reconstruction()
{
	const auto simbicon = std::dynamic_pointer_cast<SimBiCon>(m_default_simbicon_framework->get_controllers()[0]);
	auto fsm_states = simbicon->get_fsm_states();
	for (auto& fsm_state : fsm_states)
	{
		simbicon->write_state("../../out/state1.txt");
	}
}