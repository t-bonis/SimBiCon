#include "Feet_viewer.h"
#include "SimBiCon_framework.h"
#include "ui_Viewer_widget_form.h"


Feet_viewer::Feet_viewer(SimBiCon_framework& con_f, QWidget *parent) : Viewer_interface(con_f, parent)
{
	const auto articulated_figure = m_con_f->get_controlled_character();
	m_one_line_charts.resize(6 + 3 + 2 + 6);
	m_two_lines_charts.resize(3);

	for (size_t i = 0; i < 3; i++)
	{
		m_one_line_charts[i].chart->setTitle(QString::fromStdString("Force on right foot"));

		m_ui->gridLayout->addWidget(new QChartView(m_one_line_charts[i].chart.get()), int(i / 3), i % 3);
	}

	for (size_t i = 3; i < 6; i++)
	{
		m_one_line_charts[i].chart->setTitle(QString::fromStdString("Force on left foot"));

		m_ui->gridLayout->addWidget(new QChartView(m_one_line_charts[i].chart.get()), int(i / 3), i % 3);
	}

	for (size_t i = 6; i < 9; i++)
	{
		m_one_line_charts[i].chart->setTitle(QString::fromStdString("Center of pressure"));

		m_ui->gridLayout->addWidget(new QChartView(m_one_line_charts[i].chart.get()), int(i / 3), i % 3);
	}

	m_one_line_charts[9].chart->setTitle(QString::fromStdString("Nb contact point r_foot"));

	m_ui->gridLayout->addWidget(new QChartView(m_one_line_charts[9].chart.get()), 9 / 3, 9 % 3);

	m_one_line_charts[10].chart->setTitle(QString::fromStdString("Nb contact point l_foot"));

	m_ui->gridLayout->addWidget(new QChartView(m_one_line_charts[10].chart.get()), 10 / 3, 10 % 3);

	for (size_t i = 11; i < 14; i++)
	{
		m_one_line_charts[i].chart->setTitle(QString::fromStdString("Torque on right foot"));

		m_ui->gridLayout->addWidget(new QChartView(m_one_line_charts[i].chart.get()), int((i+1) / 3), (i+1) % 3);
	}

	for (size_t i = 14; i < 17; i++)
	{
		m_one_line_charts[i].chart->setTitle(QString::fromStdString("Torque on left foot"));

		m_ui->gridLayout->addWidget(new QChartView(m_one_line_charts[i].chart.get()), int((i+1) / 3), (i+1) % 3);
	}

	for (size_t i = 0; i < 3; i++)
	{
		m_two_lines_charts[i].chart->setTitle(QString::fromStdString("Center of mass vel"));
		
		m_ui->gridLayout->addWidget(new QChartView(m_two_lines_charts[i].chart.get()), int((i + 18) / 3), (i + 18) % 3);
	}
}


void Feet_viewer::update(const Subject_interface* subject)
{
	if (const auto gait_analyzer = dynamic_cast<const Gait_analyzer*>(subject))
	{
		// force on r foot
		auto chart1 = &m_one_line_charts[0];
		auto chart2 = &m_one_line_charts[1];
		auto chart3 = &m_one_line_charts[2];

		for (size_t i = 0; i < gait_analyzer->force_on_r_foot.size(); i = i + SimGlobals::m_interval_between_measure)
		{
			const auto y11 = gait_analyzer->force_on_r_foot[i].x;
			const auto y12 = gait_analyzer->force_on_r_foot[i].y;
			const auto y13 = gait_analyzer->force_on_r_foot[i].z;

			const auto x = i;

			chart1->series->append(x, y11);
			chart2->series->append(x, y12);
			chart3->series->append(x, y13);
		}
		chart1->update_axis();
		chart2->update_axis();
		chart3->update_axis();


		// force on l foot
		chart1 = &m_one_line_charts[3];
		chart2 = &m_one_line_charts[4];
		chart3 = &m_one_line_charts[5];

		for (size_t i = 0; i < gait_analyzer->force_on_l_foot.size(); i = i + SimGlobals::m_interval_between_measure)
		{
			const auto y11 = gait_analyzer->force_on_l_foot[i].x;
			const auto y12 = gait_analyzer->force_on_l_foot[i].y;
			const auto y13 = gait_analyzer->force_on_l_foot[i].z;

			const auto x = i;

			chart1->series->append(x, y11);
			chart2->series->append(x, y12);
			chart3->series->append(x, y13);
		}
		chart1->update_axis();
		chart2->update_axis();
		chart3->update_axis();

		// torque on r foot
		chart1 = &m_one_line_charts[11];
		chart2 = &m_one_line_charts[12];
		chart3 = &m_one_line_charts[13];

		for (size_t i = 0; i < gait_analyzer->torque_on_r_foot.size(); i = i + SimGlobals::m_interval_between_measure)
		{
			const auto y11 = gait_analyzer->torque_on_r_foot[i].x;
			const auto y12 = gait_analyzer->torque_on_r_foot[i].y;
			const auto y13 = gait_analyzer->torque_on_r_foot[i].z;

			const auto x = i;

			chart1->series->append(x, y11);
			chart2->series->append(x, y12);
			chart3->series->append(x, y13);
		}
		chart1->update_axis();
		chart2->update_axis();
		chart3->update_axis();

		// torque on l foot
		chart1 = &m_one_line_charts[14];
		chart2 = &m_one_line_charts[15];
		chart3 = &m_one_line_charts[16];

		for (size_t i = 0; i < gait_analyzer->torque_on_l_foot.size(); i = i + SimGlobals::m_interval_between_measure)
		{
			const auto y11 = gait_analyzer->torque_on_l_foot[i].x;
			const auto y12 = gait_analyzer->torque_on_l_foot[i].y;
			const auto y13 = gait_analyzer->torque_on_l_foot[i].z;

			const auto x = i;

			chart1->series->append(x, y11);
			chart2->series->append(x, y12);
			chart3->series->append(x, y13);
		}
		chart1->update_axis();
		chart2->update_axis();
		chart3->update_axis();

		// cop
		chart1 = &m_one_line_charts[6];
		chart2 = &m_one_line_charts[7];
		chart3 = &m_one_line_charts[8];

		for (size_t i = 0; i < gait_analyzer->cop.size(); i = i + SimGlobals::m_interval_between_measure)
		{
			const auto y11 = gait_analyzer->cop[i].x;
			const auto y12 = gait_analyzer->cop[i].y;
			const auto y13 = gait_analyzer->cop[i].z;

			const auto x = i;

			chart1->series->append(x, y11);
			chart2->series->append(x, y12);
			chart3->series->append(x, y13);
		}
		chart1->update_axis();
		chart2->update_axis();
		chart3->update_axis();


		// nb contact r foot
		chart1 = &m_one_line_charts[9];

		for (size_t i = 0; i < gait_analyzer->r_foot_nb_contact.size(); i = i + SimGlobals::m_interval_between_measure)
		{
			const auto y11 = gait_analyzer->r_foot_nb_contact[i];

			const auto x = i;

			chart1->series->append(x, y11);
		}

		chart1->update_axis();

		// nb contact l foot
		chart1 = &m_one_line_charts[10];

		for (size_t i = 0; i < gait_analyzer->l_foot_nb_contact.size(); i = i + SimGlobals::m_interval_between_measure)
		{
			const auto y11 = gait_analyzer->l_foot_nb_contact[i];

			const auto x = i;

			chart1->series->append(x, y11);
		}
		chart1->update_axis();

		// cop
		auto chart4 = &m_two_lines_charts[0];
		auto chart5 = &m_two_lines_charts[1];
		auto chart6 = &m_two_lines_charts[2];

		for (size_t i = 0; i < gait_analyzer->cop.size(); i = i + SimGlobals::m_interval_between_measure)
		{
			const auto y11 = gait_analyzer->com_vel[i].x;
			const auto y12 = gait_analyzer->com_vel[i].y;
			const auto y13 = gait_analyzer->com_vel[i].z;

			const auto y21 = gait_analyzer->com_vel_ref[i].x;
			const auto y22 = gait_analyzer->com_vel_ref[i].y;
			const auto y23 = gait_analyzer->com_vel_ref[i].z;

			const auto x = i;

			chart4->series1->append(x, y11);
			chart5->series1->append(x, y12);
			chart6->series1->append(x, y13);

			chart4->series2->append(x, y21);
			chart5->series2->append(x, y22);
			chart6->series2->append(x, y23);
		}
		chart4->update_axis();
		chart5->update_axis();
		chart6->update_axis();

	}
}
