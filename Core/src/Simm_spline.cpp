#include "Simm_spline.h"

Simm_spline::Simm_spline(std::vector<double> aX, std::vector<double> aY)
{
	size_t min_size = 2;

	if (aX.size() < min_size || aY.size() < min_size)
	{
		throw std::logic_error("SimmSpline: ERROR- there must be 2 or more data points.\n");
	}

	// INDEPENDENT VALUES (KNOT SEQUENCE)
	x_ = aX;

	y_ = aY;

	// FIT THE SPLINE
	calcCoefficients();
}

void Simm_spline::setEqual(const Simm_spline& aSpline)
{
	// CHECK ARRAY SIZES
	if (aSpline.getSize() <= 0) return;

	// ALLOCATE ARRAYS
	x_ = aSpline.x_;
	y_ = aSpline.y_;
	b_ = aSpline.b_;
	c_ = aSpline.c_;
	d_ = aSpline.d_;
}


Simm_spline& Simm_spline::operator=(const Simm_spline& aSpline)
{
	// BASE CLASS
	Function::operator=(aSpline);

	// DATA
	setEqual(aSpline);

	return (*this);
}

size_t Simm_spline::getSize() const
{
	return (x_.size());
}

const std::vector<double>& Simm_spline::getX() const
{
	return (x_);
}

const std::vector<double>& Simm_spline::getY() const
{
	return (y_);
}

const double* Simm_spline::getXValues() const
{
	return (&x_[0]);
}

const double* Simm_spline::getYValues() const
{
	return (&y_[0]);
}

void Simm_spline::calcCoefficients()
{
	auto n = x_.size();
	size_t i;
	double t;

	if (n < 2)
		return;

	b_.resize(n);
	c_.resize(n);
	d_.resize(n);

	if (n == 2)
	{
		t = MAX(TINY_NUMBER, x_[1] - x_[0]);
		b_[0] = b_[1] = (y_[1] - y_[0]) / t;
		c_[0] = c_[1] = 0.0;
		d_[0] = d_[1] = 0.0;
		return;
	}

	size_t nm1 = n - 1;
	size_t nm2 = n - 2;

	/* Set up tridiagonal system:
	* b = diagonal, d = offdiagonal, c = right-hand side
	*/

	d_[0] = MAX(TINY_NUMBER, x_[1] - x_[0]);
	c_[1] = (y_[1] - y_[0]) / d_[0];
	for (i = 1; i < nm1; i++)
	{
		d_[i] = MAX(TINY_NUMBER, x_[i + 1] - x_[i]);
		b_[i] = 2.0 * (d_[i - 1] + d_[i]);
		c_[i + 1] = (y_[i + 1] - y_[i]) / d_[i];
		c_[i] = c_[i + 1] - c_[i];
	}

	/* End conditions. Third derivatives at x[0] and x[n-1]
	* are obtained from divided differences.
	*/

	b_[0] = -d_[0];
	b_[nm1] = -d_[nm2];
	c_[0] = 0.0;
	c_[nm1] = 0.0;

	if (n > 3)
	{
		double d31 = MAX(TINY_NUMBER, x_[3] - x_[1]);
		double d20 = MAX(TINY_NUMBER, x_[2] - x_[0]);
		double d1 = MAX(TINY_NUMBER, x_[nm1] - x_[n - 3]);
		double d2 = MAX(TINY_NUMBER, x_[nm2] - x_[n - 4]);
		double d30 = MAX(TINY_NUMBER, x_[3] - x_[0]);
		double d3 = MAX(TINY_NUMBER, x_[nm1] - x_[n - 4]);
		c_[0] = c_[2] / d31 - c_[1] / d20;
		c_[nm1] = c_[nm2] / d1 - c_[n - 3] / d2;
		c_[0] = c_[0] * d_[0] * d_[0] / d30;
		c_[nm1] = -c_[nm1] * d_[nm2] * d_[nm2] / d3;
	}

	/* Forward elimination */

	for (i = 1; i < n; i++)
	{
		t = d_[i - 1] / b_[i - 1];
		b_[i] -= t * d_[i - 1];
		c_[i] -= t * c_[i - 1];
	}

	/* Back substitution */

	c_[nm1] /= b_[nm1];
	for (size_t j = 0; j < nm1; j++)
	{
		i = nm2 - j;
		c_[i] = (c_[i] - d_[i] * c_[i + 1]) / b_[i];
	}

	/* compute polynomial coefficients */

	b_[nm1] = (y_[nm1] - y_[nm2]) / d_[nm2] +
		d_[nm2] * (c_[nm2] + 2.0 * c_[nm1]);
	for (i = 0; i < nm1; i++)
	{
		b_[i] = (y_[i + 1] - y_[i]) / d_[i] - d_[i] * (c_[i + 1] + 2.0 * c_[i]);
		d_[i] = (c_[i + 1] - c_[i]) / d_[i];
		c_[i] *= 3.0;
	}
	c_[nm1] *= 3.0;
	d_[nm1] = d_[nm2];
}


double Simm_spline::getX(size_t aIndex) const
{
	if (aIndex >= 0 && aIndex < x_.size())
	{
		return x_[aIndex];
	}
	throw std::logic_error("Simm_spline::getX(): index out of bounds.");
}

double Simm_spline::getY(size_t aIndex) const
{
	if (aIndex >= 0 && aIndex < y_.size())
	{
		return y_[aIndex];
	}
	throw std::logic_error("Simm_spline::getY(): index out of bounds.");
}

void Simm_spline::setX(size_t aIndex, double aValue)
{
	if (aIndex >= 0 && aIndex < x_.size())
	{
		x_[aIndex] = aValue;
		calcCoefficients();
	}
	else
	{
		throw std::logic_error("Simm_spline::setX(): index out of bounds.");
	}
}

void Simm_spline::setY(size_t aIndex, double aValue)
{
	if (aIndex >= 0 && aIndex < y_.size())
	{
		y_[aIndex] = aValue;
		calcCoefficients();
	}
	else
	{
		throw std::logic_error("Simm_spline::setY(): index out of bounds.");
	}
}

bool Simm_spline::deletePoint(size_t aIndex)
{
	if (x_.size() > 2 && y_.size() > 2 &&
		aIndex < x_.size() && aIndex < y_.size())
	{
		x_.erase(x_.begin() + aIndex);
		y_.erase(y_.begin() + aIndex);

		// Recalculate the coefficients
		calcCoefficients();
		return true;
	}

	return false;
}

bool Simm_spline::deletePoints(const std::vector<int>& indices)
{
	bool pointsDeleted = false;
	auto numPointsLeft = x_.size() - indices.size();

	if (numPointsLeft >= 2)
	{
		// Assume the indices are sorted highest to lowest
		for (size_t index : indices)
		{
			if (index >= 0 && index < x_.size())
			{
				x_.erase(x_.begin() + index);
				y_.erase(y_.begin() + index);
				pointsDeleted = true;
			}
		}
		if (pointsDeleted)
			calcCoefficients();
	}

	return pointsDeleted;
}

size_t Simm_spline::addPoint(double aX, double aY)
{
	size_t i;
	for (i = 0; i < x_.size(); i++)
		if (x_[i] > aX)
			break;

	x_.insert(x_.begin() + i, aX);
	y_.insert(y_.begin() + i, aY);

	// Recalculate the slopes
	calcCoefficients();

	return i;
}

double Simm_spline::calc_value(double x) const
{
	// NOT A NUMBER
	if (y_.empty()) return (std::nan(""));
	if (b_.empty()) return (std::nan(""));
	if (c_.empty()) return (std::nan(""));
	if (d_.empty()) return (std::nan(""));

	size_t k;

	auto n = x_.size();
	double aX = x;

	/* Check if the abscissa is out of range of the function. If it is,
	* then use the slope of the function at the appropriate end point to
	* extrapolate. You do this rather than printing an error because the
	* assumption is that this will only occur in relatively harmless
	* situations (like a motion file that contains an out-of-range coordinate
	* value). The rest of the SIMM code has many checks to clamp a coordinate
	* value within its range of motion, so if you make it to this function
	* and the coordinate is still out of range, deal with it quietly.
	*/

	if (aX < x_[0])
		return y_[0] + (aX - x_[0]) * b_[0];
	if (aX > x_[n - 1])
		return y_[n - 1] + (aX - x_[n - 1]) * b_[n - 1];

	/* Check to see if the abscissa is close to one of the end points
	* (the binary search method doesn't work well if you are at one of the
	* end points.
	*/
	if (EQUAL_WITHIN_ERROR(aX, x_[0]))
		return y_[0];
	if (EQUAL_WITHIN_ERROR(aX, x_[n - 1]))
		return y_[n - 1];

	if (n < 3)
	{
		/* If there are only 2 function points, then set k to zero
		* (you've already checked to see if the abscissa is out of
		* range or equal to one of the endpoints).
		*/
		k = 0;
	}
	else
	{
		/* Do a binary search to find which two points the abscissa is between. */
		size_t i = 0;
		size_t j = n;
		while (true)
		{
			k = (i + j) / 2;
			if (aX < x_[k])
				j = k;
			else if (aX > x_[k + 1])
				i = k;
			else
				break;
		}
	}

	double dx = aX - x_[k];
	return y_[k] + dx * (b_[k] + dx * (c_[k] + dx * d_[k]));
}
