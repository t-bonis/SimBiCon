#include "Simulation_thread.h"
#include "Ode_world.h"
#include "SimBiCon_framework.h"
#include "Pelvis_pose_control.h"


Simulation_thread::Simulation_thread(SimBiCon_framework& con_f, const bool start, size_t target_simulation_length)
{
	m_forced_run = start;
	m_target_simulation_length = target_simulation_length;
	m_simbicon_framework = &con_f;
	if(m_simbicon_framework->get_gait_analyzer())
	{
		m_simbicon_framework->get_gait_analyzer()->target_simulation_length = target_simulation_length;
	}

}

void Simulation_thread::run()
{
	m_timer_synch = std::make_unique<QElapsedTimer>();
	
	while (!m_done) 
	{
		if (!SimGlobals::optimization || !SimGlobals::learning)
		{
			m_min_step_duration_ns = unsigned int(SimGlobals::dt*1e9 / SimGlobals::speed_multiplier);
		}
		if(isInterruptionRequested())
		{
			return;
		}
		while (m_forced_run || m_step_to_advance > 0)
		{
			if (m_step_to_advance > 0)
			{
				--m_step_to_advance;
			}
			m_timer_synch->start();
			process_task();
			const auto remaining = m_min_step_duration_ns - m_timer_synch->nsecsElapsed();
			if(remaining > 0)
			{
				usleep(remaining*1e-3);
			}
			
		}
	}
	emit simulation_done(m_simbicon_framework);
}

void Simulation_thread::start_simulation()
{
	m_forced_run = true;
}

void Simulation_thread::stop_simulation()
{
	m_forced_run = false;
	m_step_to_advance = 0;
}

void Simulation_thread::process_one_task()
{
	m_step_to_advance++;
}


void Simulation_thread::process_task()
{
	if (m_total_nb_of_task >= m_target_simulation_length)
	{
		m_forced_run = false;
		m_done = true;
	}
	else if (m_simbicon_framework->get_controllers()[0]->get_controlled_character()->continue_simulation())
	{
		try
		{
			m_simbicon_framework->global_step(SimGlobals::dt);
			m_total_nb_of_task++;
		}
		catch (const std::exception& err)
		{
			m_simbicon_framework->set_success(false);
			std::cout << err.what() << std::endl;
			m_forced_run = false;
			m_done = true;
		}
	}
	else
	{
		m_forced_run = false;
		m_done = true;
		m_simbicon_framework->set_success(false);
	}
}
