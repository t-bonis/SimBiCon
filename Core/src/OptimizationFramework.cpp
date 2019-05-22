#include "OptimizationFramework.h"
#include "Simulation_thread.h"
#include <boost/algorithm/string.hpp>
#include <QtCore/QThreadPool>
#include "Pelvis_pose_control.h"

Optimization_framework::Optimization_framework(std::string& input) : pop(nullptr), fitvals(nullptr), fmean(0)
{
	m_default_simbicon_framework = std::make_shared<SimBiCon_framework>(input);
	SimGlobals::draw = false;
	SimGlobals::optimization = true;
}

Optimization_framework::~Optimization_framework()
{
	for (auto& sim_thread : m_simulation_threads)
	{
		if(sim_thread->isRunning())
		{
			sim_thread->stop_simulation();
			sim_thread->requestInterruption();
			sim_thread->wait();
		}
	}
	if(m_analyzer_thread)
	{
		m_analyzer_thread->wait();
	}
}

void Optimization_framework::load_optimization_parameters(std::string& f_name)
{
	std::ifstream ifs;
	ifs.open(f_name);

	// read to the end
	while (!ifs.eof())
	{
		std::string line;
		getline(ifs, line);
		// Syntax : ParamsName | Mean value
		Param_for_opt temp_param_for_opt;
		std::vector<std::string> words;
		boost::algorithm::trim(line);
		if (!line.empty() && line.front() != '#')
		{
			split(words, line, boost::is_space(), boost::algorithm::token_compress_on);
			if (words.size() < 3)
			{
				throw;
			}
			temp_param_for_opt.attribute_id = m_attribute_id.find(words[0])->second;
			switch (m_attribute_id.find(words[1])->second)
			{
			case none:
				temp_param_for_opt.name = words[0];
				break;
			case knee:
			case hip:
			case ankle:
			case subtalar:
			case mtp:
				temp_param_for_opt.name = words[1];
				temp_param_for_opt.name_id.push_back(m_default_simbicon_framework->get_controlled_character()->get_object_id(words[1] + "_l"));
				temp_param_for_opt.name_id.push_back(m_default_simbicon_framework->get_controlled_character()->get_object_id(words[1] + "_r"));
				break;
			case back:
				temp_param_for_opt.name = words[1];
				temp_param_for_opt.name_id.push_back(m_default_simbicon_framework->get_controlled_character()->get_object_id(words[1]));
				break;
			default:;
			}

			for (size_t i = 2; i < words.size(); i++)
			{
				temp_param_for_opt.values.push_back(std::stod(words[i]));
			}
			m_global_optimization_params.push_back(temp_param_for_opt);
		}
	}

	init_optimization_parameters();
}

void Optimization_framework::init_optimization_parameters()
{
	const auto dim = 11; // must be change manually (number of parameter to optimize (time is 100*dim to 300*dim^2))

	double x_start[dim];
	for (auto& i : x_start)
	{
		i = 1;
	}

	double std_dev[dim];
	for (auto& i : std_dev)
	{
		i = 0.1;
	}

	parameters.init(dim, x_start, std_dev);
	parameters.lambda = 32;
	//parameters.mu = 4; //default mu = lambda/2
	parameters.stopMaxFunEvals = 300 * pow(dim, 2);
	parameters.stopTolFunHist = 0.001;
	parameters.stopTolX = 0.001;
	fitvals = evo.init(parameters); // alloc fitness values
}

//This function generate Lambda samples of params of the controller 
//they are stored in the matrix samplesOfParams[sample_i][param_i]
void Optimization_framework::generate_population()
{
	pop = evo.samplePopulation(); // do not change content of pop

	for (auto i = 0; i < evo.get(CMAES<double>::PopSize); ++i)
	{
		while (!is_feasible(pop[i]))
		{
			evo.reSampleSingle(i);
			std::cout << "re sampled" << std::endl;
		}
	}

	m_samples_of_params.clear();
	for (auto sample_i = 0; sample_i < parameters.lambda; sample_i++)
	{
		std::vector<Param_for_opt> temp_sample_of_params;
		if (m_global_optimization_params.size() != parameters.N)
		{
			throw std::logic_error("optimization size not compatible with N");
		}
		auto param_i = 0;
		while (param_i < parameters.N)
		{
			for (auto& a_list : m_global_optimization_params)
			{
				Param_for_opt temp_param_for_opt;
				for (auto& value : a_list.values)
				{
					temp_param_for_opt.values.push_back(pop[sample_i][param_i] * value);
					param_i++;
				}
				temp_param_for_opt.attribute_id = a_list.attribute_id;
				temp_param_for_opt.name_id = a_list.name_id;
				temp_param_for_opt.name = a_list.name;
				temp_sample_of_params.push_back(temp_param_for_opt);
			}
		}
		// write the samples values into a struct
		m_samples_of_params.push_back(temp_sample_of_params);
	}
	std::cout << "###############" << std::endl;
	std::cout << "### New pop ###" << std::endl;
	std::cout << "###############" << std::endl;

}

bool Optimization_framework::is_feasible(double* const p) const
{

	for (auto i = 0; i < parameters.N; i++)
	{
		if (p[i] <= 0)
		{
			return false;
		}
	}
	return true;
}

void Optimization_framework::next_pop()
{
	evo.updateDistribution(fitvals);

	if (!evo.testForTermination() && !SimGlobals::force_end)
	{
		popToEval++;
		try
		{
			next_sample_to_eval = 0;
			sample_evaluated = 0;
			generate_population();
			for (auto i = 0; i < SimGlobals::nb_threads - 2; ++i)
			{
				next_eval();
			}

		}
		catch (const std::exception& err)
		{
			std::cout << "Error during population loop : " << err.what();
		}
	}
	else
	{
		std::cout << "Stop:" << std::endl << evo.getStopMessage();
		evo.writeToFile(CMAES<double>::WKAll, "resume_evo.dat");
	}

}

void Optimization_framework::next_eval()
{
	std::cout << "### Setup_next_eval ###\n";
	auto simbicon_available = create_simbicon_framework();
	assign_samples_of_params(*simbicon_available, m_samples_of_params[next_sample_to_eval]);
	simbicon_available->set_sample_id(next_sample_to_eval);
	next_sample_to_eval++;
	eval_simbicon_framework(*simbicon_available);
	
}

void Optimization_framework::simulation_done(SimBiCon_framework* simbicon_framework)
{
	if (m_samples_of_params.size() > next_sample_to_eval)
	{
		next_eval();
	}
	if(m_analyzer_thread)
	{
		m_analyzer_thread->wait();
	}
	m_analyzer_thread = std::make_shared<Analyzer_thread>(*simbicon_framework);
	connect(m_analyzer_thread.get(), &Analyzer_thread::analyze_done, this, &Optimization_framework::analyze_done);
	m_analyzer_thread->start();
	std::cout << "### Simulation done ###\n";
}

void Optimization_framework::analyze_done(SimBiCon_framework* simbicon_framework)
{
	fitvals[simbicon_framework->get_sample_id()] = simbicon_framework->get_result()[0];
	simbicon_framework->temp_string << simbicon_framework->get_result();
	BOOST_LOG_TRIVIAL(trace) << simbicon_framework->temp_string.str();
	auto i = m_simbicon_frameworks.begin();
	for (auto& sim_bi : m_simbicon_frameworks)
	{
		if (sim_bi.get() == simbicon_framework)
		{
			break;
		}
		++i;
	}
	m_simbicon_frameworks.erase(i);
	std::cout << "### Analyze done ###\n";
	sample_evaluated++;
	if (m_samples_of_params.size() == sample_evaluated)
	{
		next_pop();
	}
}

void Optimization_framework::eval_simbicon_framework(SimBiCon_framework& simbicon_framework)
{
	for(auto i = m_simulation_threads.begin(); i != m_simulation_threads.end(); ++i)
	{
		if (i[0]->isFinished())
		{
			m_simulation_threads.erase(i);
			break;
		}
	}

	m_simulation_threads.push_back(std::make_shared<Simulation_thread>(simbicon_framework, true, 1000));
	connect(m_simulation_threads.back().get(), &Simulation_thread::simulation_done, this,
		&Optimization_framework::simulation_done);
	m_simulation_threads.back()->start();
	std::cout << "### Start eval ###\n";
}

SimBiCon_framework* Optimization_framework::create_simbicon_framework()
{
	//Load Model
	try
	{
		std::string inputConF = "../init/input.conF";
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

void Optimization_framework::assign_samples_of_params(SimBiCon_framework& simbicon_framework, std::vector<Param_for_opt> samples_of_params)
{
	std::cout << "eval : " << popToEval * parameters.lambda + next_sample_to_eval + 1 << std::endl;
	simbicon_framework.temp_string << std::setprecision(std::numeric_limits<double>::digits10 + 1);
	simbicon_framework.temp_string << popToEval * parameters.lambda + next_sample_to_eval + 1 << " ";
	for (auto& params : samples_of_params)
	{
		switch (params.attribute_id)
		{
		case kd:
			std::dynamic_pointer_cast<SimBiCon>(simbicon_framework.get_controllers()[0])->get_pose_controller()->set_kd(params.name_id[0], params.values[0]);
			simbicon_framework.temp_string << params.values[0] << " ";
			break;

		case kp:
			std::dynamic_pointer_cast<SimBiCon>(simbicon_framework.get_controllers()[0])->get_pose_controller()->set_kp(params.name_id[0], params.values[0]);
			simbicon_framework.temp_string << params.values[0] << " ";
			break;

		case both_kd:
			std::dynamic_pointer_cast<SimBiCon>(simbicon_framework.get_controllers()[0])->get_pose_controller()->set_kd(params.name_id[0], params.values[0]);
			std::dynamic_pointer_cast<SimBiCon>(simbicon_framework.get_controllers()[0])->get_pose_controller()->set_kd(params.name_id[1], params.values[0]);
			simbicon_framework.temp_string << params.values[0] << " ";
			break;

		case both_kp:
			std::dynamic_pointer_cast<SimBiCon>(simbicon_framework.get_controllers()[0])->get_pose_controller()->set_kp(params.name_id[0], params.values[0]);
			std::dynamic_pointer_cast<SimBiCon>(simbicon_framework.get_controllers()[0])->get_pose_controller()->set_kp(params.name_id[1], params.values[0]);
			simbicon_framework.temp_string << params.values[0] << " ";
			break;

		case pelvis_control_kd_t:
			std::dynamic_pointer_cast<Pelvis_pose_control>(simbicon_framework.get_controllers()[1])->set_kd_t(params.values[0]);
			simbicon_framework.temp_string << params.values[0] << " ";
			break;
		case pelvis_control_kp_t:
			std::dynamic_pointer_cast<Pelvis_pose_control>(simbicon_framework.get_controllers()[1])->set_kp_t(params.values[0]);
			simbicon_framework.temp_string << params.values[0] << " ";
			break;
		case pelvis_control_kd_f:
			std::dynamic_pointer_cast<Pelvis_pose_control>(simbicon_framework.get_controllers()[1])->set_kd_f(params.values[0]);
			simbicon_framework.temp_string << params.values[0] << " ";
			break;
		case pelvis_control_kp_f:
			std::dynamic_pointer_cast<Pelvis_pose_control>(simbicon_framework.get_controllers()[1])->set_kp_f(params.values[0]);
			simbicon_framework.temp_string << params.values[0] << " ";
			break;
		default:
			break;
		}
	}
}
