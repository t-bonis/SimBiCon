#pragma once
#include "Viewer_interface.h"
#include "Observer_interface.h"

class Angles_viewer : public Viewer_interface, public Observer_interface
{
public:
	Angles_viewer(SimBiCon_framework& con_f, QWidget* parent);
private:
	void update(const Subject_interface* subject) override;
};

