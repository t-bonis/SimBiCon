#include "Joint_info_viewer.h"
#include "SimBiCon_framework.h"
#include "ui_Viewer_widget_form.h"


Joint_info_viewer::Joint_info_viewer(SimBiCon_framework& con_f, QWidget *parent) : Viewer_interface(con_f, parent)
{

	const auto joints = m_con_f->get_controlled_character()->get_joints();
	m_two_lines_charts.resize(joints.size() * 3 * 2);

	for (size_t i = 0; i < m_two_lines_charts.size(); i++)
	{
		m_two_lines_charts[i].chart->setTitle(QString::fromStdString(joints[i / 6]->get_name()));
		m_ui->gridLayout->addWidget(new QChartView(m_two_lines_charts[i].chart.get()), int(i / 3), i % 3);
	}
}


void Joint_info_viewer::update(const Subject_interface* subject)
{
	if (const auto gait_analyzer = dynamic_cast<const Gait_analyzer*>(subject))
	{
		for (size_t i = 0; i < gait_analyzer->forces1.size(); i++)
		{
			auto chart1 = &m_two_lines_charts[i * 6];
			auto chart2 = &m_two_lines_charts[i * 6 + 1];
			auto chart3 = &m_two_lines_charts[i * 6 + 2];
			auto chart4 = &m_two_lines_charts[i * 6 + 3];
			auto chart5 = &m_two_lines_charts[i * 6 + 4];
			auto chart6 = &m_two_lines_charts[i * 6 + 5];

			for (size_t j = 0; j < gait_analyzer->forces1[i].size(); j = j + SimGlobals::m_interval_between_measure)
			{
				const auto y11 = gait_analyzer->forces1[i][j].x;
				const auto y12 = gait_analyzer->forces1[i][j].y;
				const auto y13 = gait_analyzer->forces1[i][j].z;
				const auto y14 = gait_analyzer->torques1[i][j].x;
				const auto y15 = gait_analyzer->torques1[i][j].y;
				const auto y16 = gait_analyzer->torques1[i][j].z;

				const auto y21 = gait_analyzer->forces2[i][j].x;
				const auto y22 = gait_analyzer->forces2[i][j].y;
				const auto y23 = gait_analyzer->forces2[i][j].z;
				const auto y24 = gait_analyzer->torques2[i][j].x;
				const auto y25 = gait_analyzer->torques2[i][j].y;
				const auto y26 = gait_analyzer->torques2[i][j].z;

				const auto x = j;

				chart1->series1->append(x, y11);
				chart2->series1->append(x, y12);
				chart3->series1->append(x, y13);
				chart4->series1->append(x, y14);
				chart5->series1->append(x, y15);
				chart6->series1->append(x, y16);

				chart1->series2->append(x, y21);
				chart2->series2->append(x, y22);
				chart3->series2->append(x, y23);
				chart4->series2->append(x, y24);
				chart5->series2->append(x, y25);
				chart6->series2->append(x, y26);
			}

			chart1->update_axis();
			chart2->update_axis();
			chart3->update_axis();
			chart4->update_axis();
			chart5->update_axis();
			chart6->update_axis();
		}
	}
}
