#pragma once
#include <QtWidgets/QWidget.h>
#include <QtCore/QThread>
#include <QtCore/QTimer>
#include "ui_Qt_gui.h"

#include "SimBiCon_framework.h"
#include "My_qopengl_widget.h"
#include "Simulation_thread.h"
#include "Analyzer_thread.h"
#include "Viewer_interface.h"


class Optimization_framework;
class Learning_framework;

class Qt_gui : public QMainWindow
{
	Q_OBJECT
public:
	explicit Qt_gui(QWidget* parent = nullptr);

	~Qt_gui();

	Qt_gui(const Qt_gui& other) = delete;
	Qt_gui(Qt_gui&& other) = delete;
	Qt_gui& operator=(const Qt_gui& other) = delete;
	Qt_gui& operator=(Qt_gui&& other) = delete;

	Ui::QtGuiClass get_ui() const
	{
		return m_ui;
	}

	void set_ui(const Ui::QtGuiClass& ui)
	{
		m_ui = ui;
	}

	void setup_open_gl_visual(SimBiCon_framework& simbicon_framework);
	void setup_data_viewer(SimBiCon_framework& simbicon_framework);

private slots:
	void simulation_done();
	void setup_simulation();
	void setup_optimization();
	void start_learning();
	void stop_opti() const;
	void set_speed_value(int) const;

signals:

private:
	Ui::QtGuiClass m_ui{};
	std::unique_ptr<QTimer> m_timer;

	std::shared_ptr<SimBiCon_framework> m_simbicon_framework;
	std::unique_ptr<My_qopengl_widget> m_gl_widget{ nullptr };
	std::vector<std::shared_ptr<Viewer_interface>> m_data_viewers{ nullptr };

	std::shared_ptr<Optimization_framework> m_optimization_framework;
	std::shared_ptr<Learning_framework> m_learning_framework;

	std::shared_ptr<Simulation_thread> m_simulation_thread;
	std::shared_ptr<Analyzer_thread> m_analyzer_thread;

	


};