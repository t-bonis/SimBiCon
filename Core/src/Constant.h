#pragma once
#include "Function.h"

class Constant : public Function
{
public:
	Constant();
	explicit Constant(double value);
	Constant& operator=(const Constant& aConstant);


	~Constant() = default;

	Constant(const Constant& other) = delete;
	Constant(Constant&& other) = delete;
	Constant& operator=(Constant&& other) = delete;

	void copy_data(const Constant& aConstant);

	void set_value(double aValue);

	double calc_value(double xUnused) const override
	{
		return m_value;
	}

	double get_value() const { return m_value; }

private:
	double m_value{};

};
