#include "Position_viewer.h"
#include "SimBiCon_framework.h"
#include "ui_Viewer_widget_form.h"

Position_viewer::Position_viewer(SimBiCon_framework& con_f, QWidget *parent) : Viewer_interface(con_f, parent)
{
	const auto articulated_figure = m_con_f->get_controlled_character();
	m_two_lines_charts.resize(articulated_figure->get_arbs().size() * 3);

	for (size_t i = 0; i < m_two_lines_charts.size(); i++)
	{
		m_two_lines_charts[i].chart->setTitle(QString::fromStdString(articulated_figure->get_arbs()[i / 3]->get_name()));

		m_ui->gridLayout->addWidget(new QChartView(m_two_lines_charts[i].chart.get()), int(i / 3), i % 3);
	}
}


void Position_viewer::update(const Subject_interface* subject)
{
	if (const auto gait_analyzer = dynamic_cast<const Gait_analyzer*>(subject))
	{
		for (size_t i = 0; i < gait_analyzer->arb_position.size(); i++)
		{
			auto chart1 = &m_two_lines_charts[i * 3];
			auto chart2 = &m_two_lines_charts[i * 3 + 1];
			auto chart3 = &m_two_lines_charts[i * 3 + 2];

			for (size_t j = 0; j < gait_analyzer->arb_position[i].size(); j = j + SimGlobals::m_interval_between_measure)
			{
				const auto y11 = gait_analyzer->arb_position[i][j].x;
				const auto y12 = gait_analyzer->arb_position[i][j].y;
				const auto y13 = gait_analyzer->arb_position[i][j].z;

				const auto y21 = gait_analyzer->arb_position_ref[i][j].x;
				const auto y22 = gait_analyzer->arb_position_ref[i][j].y;
				const auto y23 = gait_analyzer->arb_position_ref[i][j].z;
				
				const auto x = j;
				
				chart1->series1->append(x, y11);
				chart2->series1->append(x, y12);
				chart3->series1->append(x, y13);

				chart1->series2->append(x, y21);
				chart2->series2->append(x, y22);
				chart3->series2->append(x, y23);
			}

			chart1->update_axis();
			chart2->update_axis();
			chart3->update_axis();
		}
	}
}

