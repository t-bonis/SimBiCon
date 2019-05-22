#pragma once
#include "QtGui.h"
#include "cma-es/cmaes.h"


class Optimization_framework : public QObject
{
	Q_OBJECT
public:
	enum att_id
	{
		none,
		knee,
		hip,
		ankle,
		subtalar,
		mtp,
		back,
		both_kd,
		both_kp,
		kp,
		kd,
		pelvis_control_kp_f,
		pelvis_control_kd_f,
		pelvis_control_kp_t,
		pelvis_control_kd_t,
	};

	struct Param_for_opt
	{
		std::vector<size_t> name_id{};
		std::string name;
		att_id attribute_id{};
		std::vector<double> values;
	};

public:
	Optimization_framework() = delete;
	explicit Optimization_framework(std::string& input);

	~Optimization_framework();

	Optimization_framework(const Optimization_framework& other) = delete;
	Optimization_framework(Optimization_framework&& other) = delete;
	Optimization_framework& operator=(const Optimization_framework& other) = delete;
	Optimization_framework& operator=(Optimization_framework&& other) = delete;

	void load_optimization_parameters(std::string& f_name);

	bool is_feasible(double* const p) const;
	void generate_population();
	void next_eval();
	void next_pop();
private:
	void init_optimization_parameters();

	void assign_samples_of_params(SimBiCon_framework& simbicon_framework, std::vector<Param_for_opt> samples_of_params);
	void eval_simbicon_framework(SimBiCon_framework& simbicon_framework);
	SimBiCon_framework* create_simbicon_framework();

private slots:
	void simulation_done(SimBiCon_framework* simbicon_framework);
	void analyze_done(SimBiCon_framework* simbicon_framework);

private:
	std::vector<Param_for_opt> m_global_optimization_params;
	std::vector<std::vector<Param_for_opt>> m_samples_of_params;
	std::vector < std::shared_ptr<Simulation_thread>> m_simulation_threads;
	std::shared_ptr<Analyzer_thread> m_analyzer_thread;

	std::map<std::string, att_id> m_attribute_id{
	{"/",none},
	{"knee",knee},
	{"hip",hip},
	{"ankle",ankle},
	{"subtalar",subtalar},
	{"back",back},
	{"mtp",mtp},
	{"both_kd",both_kd},
	{"both_kp",both_kp},
	{"kp",kp},
	{"kd",kd},
	{"pelvis_control_kp_f", pelvis_control_kp_f},
	{"pelvis_control_kd_f", pelvis_control_kd_f},
	{"pelvis_control_kp_t", pelvis_control_kp_t},
	{"pelvis_control_kd_t", pelvis_control_kd_t},
	};

	std::shared_ptr<SimBiCon_framework> m_default_simbicon_framework;
	std::vector<std::shared_ptr<SimBiCon_framework>> m_simbicon_frameworks;

	size_t next_sample_to_eval = 0;
	size_t sample_evaluated = 0;
	size_t popToEval = 0;
	CMAES<double> evo; // the optimizer
	Parameters<double> parameters;
	double*const* pop; // sampled population
	double* fitvals; // objective function values of sampled population
	double fbestever = 0, *xbestever = nullptr; // store best solution
	double fmean;

	int irun = 0;
	int countevals = 0;
	int iSampleEval = 0;
};
