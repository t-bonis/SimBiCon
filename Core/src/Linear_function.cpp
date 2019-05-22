#include "Linear_function.h"
#include <utility>

Linear_function::Linear_function()
{
	setSlope(1.0);
	setIntercept(0.0);
}

void Linear_function::copyData(const Linear_function& aLinear_function)
{
	m_coefficients = aLinear_function.m_coefficients;
}

Linear_function& Linear_function::operator=(const Linear_function& aLinearFunction)
{
	// BASE CLASS
	Function::operator=(aLinearFunction);

	// DATA
	copyData(aLinearFunction);

	return (*this);
}

Linear_function::Linear_function(std::vector<double> in_coefficients)
{
	setCoefficients(std::move(in_coefficients));
}

void Linear_function::setCoefficients(std::vector<double> in_coefficients)
{
	m_coefficients = std::move(in_coefficients);
}
