#pragma once
#include "QtGui.h"
#include "cma-es/cmaes.h"

struct Cma_object
{
	size_t popToEval = 0;
	CMAES<double> evo; // the optimizer
	Parameters<double> parameters;
	double*const* pop; // sampled population
	double* fitvals; // objective function values of sampled population

	Cma_object::Cma_object(size_t lambda, const int dim,const double* x_start,const double* std_dev)
	{
		parameters.init(dim, x_start, std_dev);
		parameters.lambda = lambda;
		parameters.logWarnings = true;
		parameters.mu = 6;
		parameters.stopMaxFunEvals = 1e9;
		//parameters.stopTolFunHist = 0.005;
		parameters.stopTolX = 0.05;
		parameters.stopTolUpXFactor = 100;
		//parameters.weightMode = parameters.LINEAR_WEIGHTS;
		fitvals = evo.init(parameters); // alloc fitness values
		pop = evo.samplePopulation();
	}
};

struct knot_ptr
{
	size_t state_id;
	size_t trajectory_id;
	size_t component_id;
	size_t knot_id;
};

class Learning_framework : public QObject
{
	Q_OBJECT
public:
	Learning_framework() = delete;
	explicit Learning_framework(std::string& input);
	void compute_intervals();

	~Learning_framework();

	Learning_framework(const Learning_framework& other) = delete;
	Learning_framework(Learning_framework&& other) = delete;
	Learning_framework& operator=(const Learning_framework& other) = delete;
	Learning_framework& operator=(Learning_framework&& other) = delete;

	void load_learning_parameters(std::string& f_name);
	void assign_values(SimBiCon_framework& simbicon_framework, size_t sample_to_eval);
	void assign_best_values(SimBiCon_framework& available_framework, size_t nomber_to_update);
	static knot_ptr get_next_knot(std::vector<std::shared_ptr<Fsm_state>> fsm_states, knot_ptr current_knot);
	void eval_simbicon_framework(SimBiCon_framework & simbicon_framework, size_t interval_begining, size_t nb_of_intervals);
	void start_learning();
	SimBiCon_framework* create_simbicon_framework(SimBiCon_framework* in_simbicon_framework);
	bool is_feasible(double* p, Cma_object* cma) const;
	void next_eval();
	void next_iteration();
	void next_pop();

	void print_state(const std::shared_ptr<Cma_object>& cma);

	std::vector<double>& get_intervals()
	{
		return m_intervals;
	}

private slots:
	void simulation_done(SimBiCon_framework* simbicon_framework);
	void analyze_done(SimBiCon_framework* simbicon_framework);

	size_t get_rank(SimBiCon_framework * simbicon_framework);

private:
	void delete_framework(SimBiCon_framework * simbicon_framework);
	std::shared_ptr<SimBiCon_framework> get_ptr(SimBiCon_framework * simbicon_framework);
	void end_reconstruction();

private:
	std::shared_ptr<SimBiCon_framework> m_default_simbicon_framework;
	std::vector<std::shared_ptr<SimBiCon_framework>> m_simbicon_frameworks;
	std::vector<SimBiCon_framework*> m_saved_frameworks;
	std::vector<SimBiCon_framework*> m_next_saved_frameworks;

	std::vector<std::shared_ptr<Simulation_thread>> m_simulation_threads;
	std::shared_ptr<Analyzer_thread> m_analyzer_thread;

	std::vector<std::shared_ptr<Cma_object>> m_cma;
	size_t m_target_nb_iterations = 10;
	size_t m_iteration = 0;
	size_t m_sample_to_save = 10;
	std::vector<std::vector<double>> m_all_cost;
	std::vector<double> m_final_cost;
	std::vector<std::vector<std::pair<size_t, size_t>>> m_combinaison_history;
	std::vector<double> m_intervals{};

	size_t m_next_sample_to_eval = 0;
	
	size_t m_current_pop = 0;
	size_t m_sample_evaluated = 0;
	double m_reconstructed_time = 0;
	size_t m_target_simulation_time;
	int m_dim = 15;
	size_t m_lambda = 200;
	std::array<double,15> m_x_start;
	std::array<double,15> m_std_dev;
};

