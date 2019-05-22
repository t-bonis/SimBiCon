#pragma once
#include "Observer_interface.h"

class Subject_interface
{
public:
	Subject_interface() = default;
	Subject_interface(const Subject_interface& other) = delete;
	Subject_interface(Subject_interface&& other) = delete;
	Subject_interface& operator=(const Subject_interface& other) = delete;
	Subject_interface& operator=(Subject_interface&& other) = delete;

	void add_observer(Observer_interface* obs);
	void del_observer(Observer_interface* obs);

	virtual ~Subject_interface();

private:
	std::list<Observer_interface*> m_list;

	typedef std::list<Observer_interface*>::iterator iterator;
	typedef std::list<Observer_interface*>::const_iterator const_iterator;


public:
	virtual void notify()
	{
		auto itb = m_list.begin();
		const const_iterator ite = m_list.end();

		for (; itb != ite; ++itb)
		{
			(*itb)->update(this);
		}
	}
};
