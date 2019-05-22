#include "Controller_interface.h"
#include "SimBiCon_framework.h"

Controller_interface::Controller_interface(Articulated_figure& character,SimBiCon_framework& framework)
{
	m_controlled_character = dynamic_cast<Character*>(&character);

	m_framework = &framework;

	m_controlled_character->add_controller(*this);
}
