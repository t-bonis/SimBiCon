#pragma once
#include "Function.h"
#include "Utils/src/Utils.h"

constexpr double roundoff_error = 0.0000000000002;

#define DABS(a) ((a)>(double)0.0?(a):(-(a)))
#define EQUAL_WITHIN_ERROR(a,b) (DABS(((a)-(b))) <= roundoff_error)

class Simm_spline : public Function
{
public:

	Simm_spline() = default;

	Simm_spline& operator=(const Simm_spline& aSpline);

	~Simm_spline() = default;

	Simm_spline(const Simm_spline& other) = delete;
	Simm_spline(Simm_spline&& other) = delete;
	Simm_spline& operator=(Simm_spline&& other) = delete;

	Simm_spline(std::vector<double> aX, std::vector<double> aY);


	size_t getSize() const;
	const std::vector<double>& getX() const;
	const std::vector<double>& getY() const;
	virtual const double* getXValues() const;
	virtual const double* getYValues() const;
	virtual size_t getNumberOfPoints() const { return x_.size(); }
	virtual double getX(size_t aIndex) const;
	virtual double getY(size_t aIndex) const;
	virtual double getZ(int aIndex) const { return 0.0; }
	virtual void setX(size_t aIndex, double aValue);
	virtual void setY(size_t aIndex, double aValue);
	virtual bool deletePoint(size_t aIndex);
	virtual bool deletePoints(const std::vector<int>& indices);
	virtual size_t addPoint(double aX, double aY);

	double calc_value(double x) const override;

private:
	void setEqual(const Simm_spline& aSpline);

	void calcCoefficients();

private:
	std::vector<double> x_;
	std::vector<double> y_;

	std::vector<double> b_;
	std::vector<double> c_;
	std::vector<double> d_;
};
