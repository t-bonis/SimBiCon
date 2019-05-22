#pragma once
#include "Viewer_interface.h"
#include "Observer_interface.h"

class Angular_acceleration : public Viewer_interface, public Observer_interface
{
public:
	Angular_acceleration(SimBiCon_framework& con_f, QWidget* parent);

private:
	void update(const Subject_interface* subject) override;
};
