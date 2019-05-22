#include "Viewer_interface.h"
#include "ui_Viewer_widget_form.h"
#include <memory>

Viewer_interface::Viewer_interface(SimBiCon_framework& conF, QWidget * parent) : QWidget(parent)
{
	m_con_f = &conF;
	m_ui = std::make_shared<Ui_Viewer_widget_form>();

	m_ui->setupUi(this);
}
