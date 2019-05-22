#pragma once
#include "Function.h"
#include "Utils/src/Utils.h"

class Linear_function :public Function
{
public:
	Linear_function();
	Linear_function(std::vector<double> in_coefficients);
	Linear_function& operator=(const Linear_function& other);

	~Linear_function() = default;

	Linear_function(const Linear_function& other) = delete;
	Linear_function(Linear_function&& other) = delete;
	Linear_function& operator=(Linear_function&& other) = delete;


	void copyData(const Linear_function& aLinear_function);

	//Set Coefficients for slope and intercept
	void setCoefficients(std::vector<double> in_coefficients);

	//Set slope
	void setSlope(double slope) { m_coefficients[0] = slope; }

	//Set intercept
	void setIntercept(double intercept) { m_coefficients[1] = intercept; }

	//Get Coefficients
	std::vector<double> getCoefficients() const
	{
		return m_coefficients;
	}

	//Get Slope
	double getSlope() { return m_coefficients[0]; }

	//Get Intercept
	double getIntercept() { return m_coefficients[1]; }

	double calc_value(double x) const override
	{
		return m_coefficients[0] * x + m_coefficients[1];
	}

private:
	std::vector<double> m_coefficients;

};
