#include "Observer_interface.h"
#include "Subject_interface.h"

void Observer_interface::add_subject(Subject_interface* sub)
{
	m_list.push_back(sub);
}

void Observer_interface::del_subject(Subject_interface* sub)
{
	const auto it = std::find(m_list.begin(), m_list.end(), sub);
	if (it != m_list.end())
		m_list.erase(it);
}

Observer_interface::~Observer_interface()
{
	const const_iterator ite = m_list.end();

	for (auto itb = m_list.begin(); itb != ite; ++itb)
	{
		(*itb)->del_observer(this);
	}
}
