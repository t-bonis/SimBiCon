#pragma once
#include <QtCharts\QtCharts>
#include <QtWidgets\QGraphicsScene>
#include <QtWidgets\QWidget>

class SimBiCon_framework;
class Ui_Viewer_widget_form;

class Viewer_interface : public QWidget
{
public:
	struct Two_lines_chart
	{
		std::shared_ptr<QChart> chart;
		std::shared_ptr < QLineSeries> series1;
		std::shared_ptr < QLineSeries> series2;
		std::shared_ptr < QValueAxis> axis_x;
		std::shared_ptr < QValueAxis> axis_y;

		Two_lines_chart()
		{
			axis_x = std::make_shared<QValueAxis>();
			axis_y = std::make_shared<QValueAxis>();

			axis_x->setRange(0, 0);
			axis_x->setTickCount(11);
			axis_x->setLabelFormat("%.0f");
			axis_y->setRange(0, 0);
			axis_y->setTickCount(11);
			axis_y->setLabelFormat("%.3f");

			chart = std::make_shared<QChart>();
			chart->setAnimationOptions(QChart::NoAnimation);
			chart->setDropShadowEnabled(false);
			chart->setMinimumSize(500, 400);

			series1 = std::make_shared<QLineSeries>();
			series2 = std::make_shared<QLineSeries>();		


			series1->setName("Real");
			series2->setName("Ref");

			chart->addSeries(series1.get());
			chart->addSeries(series2.get());

			chart->setAxisX(axis_x.get(), series1.get());
			chart->setAxisY(axis_y.get(), series1.get());
			chart->setAxisX(axis_x.get(), series2.get());
			chart->setAxisY(axis_y.get(), series2.get());
		}

		void update_axis() const
		{
			if (!series1->pointsVector().empty())
			{
				auto min_x = series1->pointsVector()[0].x();
				auto max_x = series1->pointsVector()[0].x();

				auto min_y = series1->pointsVector()[0].y();
				auto max_y = series1->pointsVector()[0].y();


				for (auto value : series1->pointsVector())
				{
					if (min_x > value.x())
					{
						min_x = value.x();
					}

					if (min_y > value.y())
					{
						min_y = value.y();
					}

					if (max_x < value.x())
					{
						max_x = value.x();
					}

					if (max_y < value.y())
					{
						max_y = value.y();
					}
				}

				for (auto value : series2->pointsVector())
				{
					if (min_x > value.x())
					{
						min_x = value.x();
					}

					if (min_y > value.y())
					{
						min_y = value.y();
					}

					if (max_x < value.x())
					{
						max_x = value.x();
					}

					if (max_y < value.y())
					{
						max_y = value.y();
					}
				}
				axis_x->setMin(min_x);
				axis_x->setMax(max_x);

				axis_y->setMin(min_y);
				axis_y->setMax(max_y);
			}
		}
	};

	struct One_line_chart
	{
		std::shared_ptr<QChart> chart;
		std::shared_ptr < QLineSeries> series;
		std::shared_ptr < QValueAxis> axis_x;
		std::shared_ptr < QValueAxis> axis_y;

		One_line_chart()
		{
			axis_x = std::make_shared<QValueAxis>();
			axis_y = std::make_shared<QValueAxis>();

			axis_x->setRange(0, 0);
			axis_x->setTickCount(11);
			axis_x->setLabelFormat("%.0f");
			axis_y->setRange(0, 0);
			axis_y->setTickCount(11);
			axis_y->setLabelFormat("%.3f");

			chart = std::make_shared<QChart>();
			chart->setAnimationOptions(QChart::NoAnimation);
			chart->setDropShadowEnabled(false);
			chart->setMinimumSize(500, 400);

			series = std::make_shared<QLineSeries>();
			chart->addSeries(series.get());

			chart->setAxisX(axis_x.get(), series.get());
			chart->setAxisY(axis_y.get(), series.get());
		}

		void update_axis() const
		{
			if (!series->pointsVector().empty())
			{
				auto min_x = series->pointsVector()[0].x();
				auto max_x = series->pointsVector()[0].x();

				auto min_y = series->pointsVector()[0].y();
				auto max_y = series->pointsVector()[0].y();


				for (auto value : series->pointsVector())
				{
					if (min_x > value.x())
					{
						min_x = value.x();
					}

					if (min_y > value.y())
					{
						min_y = value.y();
					}

					if (max_x < value.x())
					{
						max_x = value.x();
					}

					if (max_y < value.y())
					{
						max_y = value.y();
					}
				}

				axis_x->setMin(min_x);
				axis_x->setMax(max_x);

				axis_y->setMin(min_y);
				axis_y->setMax(max_y);
			}
		}
	};


	Viewer_interface() = delete;
	explicit Viewer_interface(SimBiCon_framework& conF, QWidget* parent = nullptr);

	~Viewer_interface() = default;

	Viewer_interface(const Viewer_interface& other) = delete;
	Viewer_interface(Viewer_interface&& other) = delete;
	Viewer_interface& operator=(const Viewer_interface& other) = delete;
	Viewer_interface& operator=(Viewer_interface&& other) = delete;

protected:
	SimBiCon_framework* m_con_f;
	std::vector<Two_lines_chart> m_two_lines_charts{ 0 };
	std::vector<One_line_chart> m_one_line_charts{ 0 };

	std::shared_ptr<Ui_Viewer_widget_form> m_ui;
};
