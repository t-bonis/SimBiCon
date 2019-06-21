#include "Analyzer_thread.h"
#include "SimBiCon_framework.h"
#include "Learning_framework.h"


Analyzer_thread::Analyzer_thread(SimBiCon_framework& conF, bool print_result)
{
	m_simbicon_framework = &conF;
	m_print_result = print_result;
}

void Analyzer_thread::run()
{
	eval_simulation();
	emit analyze_done(m_simbicon_framework);
	quit();
}


void Analyzer_thread::eval_simulation() const
{
	if (!m_simbicon_framework->get_gait_analyzer())
	{
		return;
	}
	if (SimGlobals::print_value)
	{
		//m_simbicon_framework->get_gait_analyzer()->notify();
		m_simbicon_framework->get_gait_analyzer()->print_values();
	}
	const auto cost_pose_control = m_simbicon_framework->get_gait_analyzer()->compute_pose_control();
	const auto cost_root_control = m_simbicon_framework->get_gait_analyzer()->compute_root_control();
	const auto cost_end_effector_control = m_simbicon_framework->get_gait_analyzer()->compute_end_effector_control();
	const auto cost_balance_control = m_simbicon_framework->get_gait_analyzer()->compute_balance_control();
	const auto nb_frame = m_simbicon_framework->get_gait_analyzer()->get_nb_frame();
	auto result = 8 * cost_pose_control + 5*cost_root_control + 20*cost_end_effector_control + 20 * cost_balance_control;
	

	if (m_print_result)
	{
		std::cout << 8 * cost_pose_control << " + " << 5 * cost_root_control << " + " << 20 * cost_end_effector_control << " + " << 20 * cost_balance_control << " = " << result << std::endl;
		std::cout << nb_frame << std::endl;
		//BOOST_LOG_TRIVIAL(trace) << angular_diff << " + " << pos_diff << " = " << result << std::endl;
	}
	//if (m_simbicon_framework->get_success() == false)
	//{
	//	auto result = -1;
	//	m_simbicon_framework->add_result(result);
	//	return;
	//}
	//if (result > 0.1)
	//{
	//	m_simbicon_framework->set_success(false);
	//	result = -1;
	//}
	m_simbicon_framework->add_result(result);


}


//void Analyzer_thread::eval_intervals() const
//{
//	//TODO : define interval then eval interval and same values
//	if (!m_simbicon_framework->get_gait_analyzer())
//	{
//		return;
//	}
//
//	auto nb_missing_frame = m_simbicon_framework->get_gait_analyzer()->get_nb_of_missing_frame();
//
//	std::vector<std::array<size_t, 2>> intervals;
//
//	for (size_t i = 0; i < m_learning_framework->get_intervals().size() - 1; ++i)
//	{
//		if(m_learning_framework->get_intervals()[i+1] > m_learning_framework->get_intervals().back() - nb_missing_frame)
//		{
//			break;
//		}
//		else
//		{
//			const std::array<size_t, 2> interval{ m_learning_framework->get_intervals()[i], m_learning_framework->get_intervals()[i + 1] };
//			intervals.push_back(interval);
//		}
//	}
//
//	for(auto& interval : intervals)
//	{
//		const auto angular_diff = m_simbicon_framework->get_gait_analyzer()->compute_pose_control(interval);
//		const auto pos_diff = m_simbicon_framework->get_gait_analyzer()->compute_root_control(interval);
//		const auto result = angular_diff +  pos_diff ;
//
//		//std::cout << angular_diff  << " + " << pos_diff  << " = " << result << std::endl;
//
//		m_simbicon_framework->add_result(result);
//
//
//	}
//}