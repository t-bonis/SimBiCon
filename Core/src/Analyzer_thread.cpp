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
}


void Analyzer_thread::eval_simulation() const
{
	if (!m_simbicon_framework->get_gait_analyzer())
	{
		return;
	}
	if (SimGlobals::print_value)
	{
		m_simbicon_framework->get_gait_analyzer()->notify();
		m_simbicon_framework->get_gait_analyzer()->print_values();
	}
	const auto angular_diff = m_simbicon_framework->get_gait_analyzer()->compute_angular_diff();
	const auto pos_diff = m_simbicon_framework->get_gait_analyzer()->compute_pelvis_pos_diff();
	const auto result = angular_diff + pos_diff;
	if (m_print_result)
	{
		std::cout << angular_diff << " + " << pos_diff << " = " << result << std::endl;
		//BOOST_LOG_TRIVIAL(trace) << angular_diff << " + " << pos_diff << " = " << result << std::endl;
	}
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
//		const auto angular_diff = m_simbicon_framework->get_gait_analyzer()->compute_angular_diff(interval);
//		const auto pos_diff = m_simbicon_framework->get_gait_analyzer()->compute_pelvis_pos_diff(interval);
//		const auto result = angular_diff +  pos_diff ;
//
//		//std::cout << angular_diff  << " + " << pos_diff  << " = " << result << std::endl;
//
//		m_simbicon_framework->add_result(result);
//
//
//	}
//}