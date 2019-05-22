#pragma once
#include <QtCore/QTime>
#include <QtCore/QObject>
#include <QtCore/QThreadPool>
#include <QtCore/QElapsedTimer>

class SimBiCon_framework;

class Simulation_thread :public QThread
{
	Q_OBJECT
public :
	Simulation_thread() = delete;
	explicit Simulation_thread(SimBiCon_framework& con_f, bool force_start = true, size_t target_simulation_length = -1);

	~Simulation_thread() = default;

	Simulation_thread(const Simulation_thread& other) = delete;
	Simulation_thread(Simulation_thread&& other)  = delete;
	Simulation_thread& operator=(const Simulation_thread& other) = delete;
	Simulation_thread& operator=(Simulation_thread&& other)  = delete;

public slots:
	void start_simulation();
	void stop_simulation();
	void process_one_task();

signals:
	void simulation_done(SimBiCon_framework* simbicon_framework);

private:
	void process_task();
	void run() override;

private:
	SimBiCon_framework* m_simbicon_framework;
	std::unique_ptr<QElapsedTimer> m_timer_synch;

	size_t m_total_nb_of_task{ 0 };
	size_t m_step_to_advance{ 0 };
	size_t m_target_simulation_length{ size_t(-1) };
	unsigned int m_min_step_duration_ns{ 0 };
	bool m_forced_run{ false };
	bool m_done{ false };

};
