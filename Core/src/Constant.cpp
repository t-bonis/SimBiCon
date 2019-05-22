#include "Constant.h"


Constant::Constant()
{
	set_value(0);
}

Constant::Constant(const double value)
{
	set_value(value);
}


void Constant::copy_data(const Constant& aConstant)
{
	m_value = aConstant.m_value;
}

Constant& Constant::operator=(const Constant& aConstant)
{
	// BASE CLASS
	Function::operator=(aConstant);

	// DATA
	copy_data(aConstant);

	return (*this);
}

void Constant::set_value(double aValue)
{
	m_value = aValue;
}
