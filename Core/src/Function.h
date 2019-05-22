#pragma once

class Function
{
public:
	Function() = default;
	Function& operator=(const Function& other) = default;

	virtual ~Function() = default;

	Function(const Function& other) = delete;
	Function(Function&& other) = delete;
	Function& operator=(Function&& other) = delete;

	virtual double calc_value(double x) const = 0;
};
