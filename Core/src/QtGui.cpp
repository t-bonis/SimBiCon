#include "QtGui.h"
#include "Simulation_thread.h"
#include "OptimizationFramework.h"
#include <QtCore/QThreadPool>
#include "Velocities_viewer.h"
#include "Angles_viewer.h"
#include "Angular_velocities.h"
#include "Feet_viewer.h"
#include "Learning_framework.h"

Qt_gui::Qt_gui(QWidget* parent) : QMainWindow(parent)
{
	m_ui.setupUi(this);
}

Qt_gui::~Qt_gui()
{
	if(m_simulation_thread)
	{
		m_simulation_thread->stop_simulation();
		m_simulation_thread->requestInterruption();
		m_simulation_thread->wait();
	}
}

//Load SimBiCon_framework from files, create a GLWindow for drawing and windows of charts 
void Qt_gui::setup_simulation()
{
	get_ui().pb_simulation->setEnabled(false);
	try
	{

		SimGlobals::draw = true;

		std::cout << std::setprecision(std::numeric_limits<double>::digits10 + 1);
		std::string inputConF = "../../init/input.conF";

		//Setup SimBiCon_framework
		//auto temp = std::make_shared<SimBiCon_framework>(inputConF);
		m_simbicon_framework = std::make_shared<SimBiCon_framework>(inputConF);
		//temp.reset();

		//Setup OpenGLWindow for rendering
		setup_open_gl_visual(*m_simbicon_framework);

		m_timer = std::make_unique<QTimer>(this);
		connect(m_timer.get(), &QTimer::timeout, m_gl_widget.get(), QOverload<>::of(&QWidget::update));
		m_timer->setInterval(32);
		m_timer->start();

		m_gl_widget->setVisible(true);
		this->showMaximized();

	}
	catch (const std::exception& err)
	{
		BOOST_LOG_TRIVIAL(error) << "Error during model loading : " << err.what();
		std::cout << "Error during model loading : " << err.what();
		return;
	}

	//Start a tread where simulation steps will be computed
	m_simulation_thread = std::make_shared<Simulation_thread>(*m_simbicon_framework, false, 186);
	connect(m_gl_widget.get(), SIGNAL(key_d_pressed()), m_simulation_thread.get(), SLOT(process_one_task()));
	connect(get_ui().pb_start, SIGNAL(released()), m_simulation_thread.get(), SLOT(start_simulation()));
	connect(m_simulation_thread.get(), &Simulation_thread::simulation_done, this,
		&Qt_gui::simulation_done);
	m_simulation_thread->start();
	get_ui().pb_start->setEnabled(true);
	
}



void Qt_gui::setup_optimization()
{
	try
	{
		SimGlobals::optimization = true;
		//Load SimBiCon_framework
		std::string input_con_f = "../../init/input.conF";
		m_optimization_framework = std::make_shared<Optimization_framework>(input_con_f);

		//Load and setup optimization parameters
		std::string f_name = "../../init/opti_both_leg.optI";
		m_optimization_framework->load_optimization_parameters(f_name);
	}
	catch (const std::exception& err)
	{
		std::cout << "Error during model loading : " << err.what();
		return;
	}

	//Start the main optimization loop
	m_optimization_framework->generate_population();
	for(auto i = 0; i < SimGlobals::nb_threads - 2; ++i)
	{
		m_optimization_framework->next_eval();
	}
}

void Qt_gui::setup_learning()
{
	std::cout << "Learning start\n";
	try
	{
		SimGlobals::learning = true;
		//Load SimBiCon_framework
		std::string input_con_f = "../../init/input.conF";
		m_learning_framework = std::make_shared<Learning_framework>(input_con_f);

		//Load and setup optimization parameters
		std::string f_name = "../../init/opti_both_leg.optI";
		m_learning_framework->load_learning_parameters(f_name);
	}
	catch (const std::exception& err)
	{
		std::cout << "Error during model loading : " << err.what();
		return;
	}
	get_ui().pb_test_default->setEnabled(true);
	get_ui().pb_run_learning->setEnabled(true);
}

void Qt_gui::run_learning()
{
	get_ui().pb_run_learning->setEnabled(false);
	//Start the main learning loop
	m_learning_framework->start_learning();

}

void Qt_gui::test_default_config()
{
	m_learning_framework->test_default_config();
}

void Qt_gui::stop_opti() const
{
	SimGlobals::force_end = true;
}

void Qt_gui::set_speed_value(int value) const
{
	SimGlobals::speed_multiplier = pow(2,value);
	std::stringstream text;
	text << "Speed : " << std::setprecision(2) << SimGlobals::speed_multiplier;
	get_ui().label->setText(QString::fromStdString(text.str()));
}

void Qt_gui::setup_open_gl_visual(SimBiCon_framework& simbicon_framework)
{
	//Setup OpenGLWidget
	m_gl_widget = std::make_unique<My_qopengl_widget>(*this, simbicon_framework, get_ui().tabWidget);
	simbicon_framework.link_gl_widget(m_gl_widget.get());
	get_ui().tabWidget->addTab(m_gl_widget.get(), "3D Render");
	get_ui().tabWidget->setEnabled(true);
	m_gl_widget->setObjectName(QStringLiteral("openGLWidget"));
	m_gl_widget->setMouseTracking(true);
	m_gl_widget->setFocusPolicy(Qt::FocusPolicy::ClickFocus);
}

void Qt_gui::setup_data_viewer(SimBiCon_framework& simbicon_framework)
{
	//m_data_viewers.push_back(std::make_shared<Position_viewer>(simbicon_framework, get_ui().tabWidget));
	//get_ui().tabWidget->insertTab(1, m_data_viewers.back().get(), "Positions");
	//m_gait_analyzer->add_observer(dynamic_cast<Position_viewer*>(m_data_viewers.back().get()));

	m_data_viewers.push_back(std::make_shared<Velocities_viewer>(simbicon_framework, get_ui().tabWidget));
	get_ui().tabWidget->insertTab(2, m_data_viewers.back().get(), "Velocities");
	simbicon_framework.get_gait_analyzer()->add_observer(dynamic_cast<Velocities_viewer*>(m_data_viewers.back().get()));

	m_data_viewers.push_back(std::make_shared<Angles_viewer>(simbicon_framework, get_ui().tabWidget));
	get_ui().tabWidget->insertTab(3, m_data_viewers.back().get(), "Angles");
	simbicon_framework.get_gait_analyzer()->add_observer(dynamic_cast<Angles_viewer*>(m_data_viewers.back().get()));

	m_data_viewers.push_back(std::make_shared<Angular_velocities>(simbicon_framework, get_ui().tabWidget));
	get_ui().tabWidget->insertTab(4, m_data_viewers.back().get(), "Angular velocities");
	simbicon_framework.get_gait_analyzer()->add_observer(dynamic_cast<Angular_velocities*>(m_data_viewers.back().get()));

	//m_data_viewers.push_back(std::make_shared<Angular_acceleration>(*this, get_ui().tabWidget));
	//get_ui().tabWidget->insertTab(5, m_data_viewers.back().get(), "Angular acceleration");
	//m_gait_analyzer->add_observer(dynamic_cast<Angular_acceleration*>(m_data_viewers.back().get()));

	m_data_viewers.push_back(std::make_shared<Feet_viewer>(simbicon_framework, get_ui().tabWidget));
	get_ui().tabWidget->insertTab(6, m_data_viewers.back().get(), "Feet");
	simbicon_framework.get_gait_analyzer()->add_observer(dynamic_cast<Feet_viewer*>(m_data_viewers.back().get()));

	//m_data_viewers.push_back(std::make_shared<Joint_info_viewer>(simbicon_framework, get_ui().tabWidget));
	//qt_gui.get_ui().tabWidget->insertTab(7, m_data_viewers.back().get(), "Joint info");
	//m_gait_analyzer->add_observer(dynamic_cast<Joint_info_viewer*>(m_data_viewers.back().get()));

	//m_data_viewers.push_back(std::make_shared<Arb_angular_velocities>(simbicon_framework, get_ui().tabWidget));
	//qt_gui.get_ui().tabWidget->insertTab(8, m_data_viewers.back().get(), "Arb angular velocities");
	//m_gait_analyzer->add_observer(dynamic_cast<Arb_angular_velocities*>(m_data_viewers.back().get()));
}

void Qt_gui::simulation_done()
{
	m_analyzer_thread = std::make_shared<Analyzer_thread>(*m_simbicon_framework,true);
	m_analyzer_thread->start();
}
