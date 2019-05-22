#pragma once
#include "Observer_interface.h"
#include "Viewer_interface.h"

class Position_viewer : public Viewer_interface, public Observer_interface
{
public :
	Position_viewer(SimBiCon_framework& con_f, QWidget* parent);
private :
	void update(const Subject_interface* subject) override;
};

