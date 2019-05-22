#include "Subject_interface.h"

void Subject_interface::add_observer(Observer_interface* obs)
{
	m_list.push_back(obs);
	obs->add_subject(this);
}

void Subject_interface::del_observer(Observer_interface* obs)
{
	const auto it = find(m_list.begin(), m_list.end(), obs);
	if (it != m_list.end())
		m_list.erase(it);
}

Subject_interface::~Subject_interface()
{
	auto itb = m_list.begin();
	const const_iterator ite = m_list.end();

	for (; itb != ite; ++itb)
	{
		(*itb)->del_subject(this);
	}
}
