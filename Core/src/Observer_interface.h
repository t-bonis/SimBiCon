#pragma once
#include <MathLib/src/Vector3d.h>
#include <MathLib/src/Quaternion.h>

class Subject_interface;

class Observer_interface
{
public:
	Observer_interface() = default;
	Observer_interface(const Observer_interface& other) = delete;
	Observer_interface(Observer_interface&& other) = delete;
	Observer_interface& operator=(const Observer_interface& other) = delete;
	Observer_interface& operator=(Observer_interface&& other) = delete;

	virtual void update(const Subject_interface* observable) = 0;

	void add_subject(Subject_interface* sub);
	void del_subject(Subject_interface* sub);

protected:
	std::list<Subject_interface*> m_list;
	typedef std::list<Subject_interface*>::iterator iterator;
	typedef std::list<Subject_interface*>::const_iterator const_iterator;
	virtual ~Observer_interface();
};



