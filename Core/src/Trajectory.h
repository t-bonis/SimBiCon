#pragma once
#include "Utils/src/Utils.h"
#include "polyfit/polyfit.hpp"

//This class is used to represent generic trajectories. The class parameter T can be anything that provides basic operation such as addition and subtraction.
//We'll define a trajectory that can be parameterized by a one-d parameter (called t). Based on a set of knots ( tuples <t, T>), we can evaluate the 
//trajectory at any t, through interpolation. This is not used for extrapolation. Outside the range of the knots, the closest known value is returned instead.
template <class T>
class GenericTrajectory
{
public:
	GenericTrajectory()
	{
		lastIndex = 0;
	}
	GenericTrajectory(const GenericTrajectory<T>& other)
	{
		lastIndex = 0;
		copy(other);
	}
	GenericTrajectory& operator=(const GenericTrajectory& other)
	{
		lastIndex = 0;
		copy(other);
		return *this;
	}
	GenericTrajectory& operator=(GenericTrajectory&& other) = default;
	~GenericTrajectory() = default;

	GenericTrajectory(GenericTrajectory&& other) = delete;
	

	T evaluate_poly(double t, int degree = 6)
	{
		if(poly_coeff.size() != degree + 1)
		{
			evaluate_poly_coeff(degree);
		}
		std::vector<double> x = { t };
		auto output = polyval<T>(poly_coeff, x);
		return output[0];
	}

	T evaluate_polyd(double t, int degree = 6)
	{
		if (poly_coeff.size() != degree + 1)
		{
			evaluate_poly_coeff(degree);
		}
		std::vector<double> x = { t };
		auto output = polydval<T>(poly_coeff, x);
		return output[0];
	}



	//	This method performs linear interpolation to evaluate the trajectory at the point t
	T evaluate_linear(double t)
	{
		const auto size = tValues.size();
		if (t <= tValues[0]) return values[0];
		if (t >= tValues[size - 1]) return values[size - 1];
		auto index = get_first_larger_index(t);

		//now linearly interpolate between index-1 and index
		t = (t - tValues[index - 1]) / (tValues[index] - tValues[index - 1]);
		return (values[index - 1]) * (1 - t) + (values[index]) * t;
	}

	T evaluate_two_point_derivative(double t)
	{
		const auto size = tValues.size();
		size_t index;
		if (t <= tValues[0])
		{
			index = 1;
		}
		else if (t >= tValues[size - 1])
		{
			index = size - 1;
		}
		else
		{
			index = get_first_larger_index(t);
		}

		//now compute the derivative between previous and next index
		return (values[index] - values[index - 1]) / (tValues[index] - tValues[index - 1]);
	}

	//This method interprets the trajectory as a Catmul-Rom spline, and evaluates it at the point t
	T evaluate_catmull_rom_derivative(double t)
	{
		const auto size = tValues.size();
		double v1, v2;
		if (t <= tValues[0])
		{
			v1 = evaluate_catmull_rom(t);
			v2 = evaluate_catmull_rom(t + SimGlobals::dt);
		}
		else if (t >= tValues[size - 1])
		{
			v1 = evaluate_catmull_rom(t - SimGlobals::dt);
			v2 = evaluate_catmull_rom(t);
		}
		else
		{
			v1 = evaluate_catmull_rom(t - SimGlobals::dt/2.0);
			v2 = evaluate_catmull_rom(t + SimGlobals::dt / 2.0);
		}

		//now compute the derivative between previous and next index
		return (v2 - v1) / SimGlobals::dt;

	}


	//This method interprets the trajectory as a Catmul-Rom spline, and evaluates it at the point t
	T evaluate_catmull_rom(double t)
	{
		const auto size = tValues.size();
		if (t <= tValues[0]) return values[0];
		if (t >= tValues[size - 1]) return values[size - 1];
		auto index = get_first_larger_index(t);

		//now that we found the interval, get a value that indicates how far we are along it
		t = (t - tValues[index - 1]) / (tValues[index] - tValues[index - 1]);

		T p0 = (index - 1 <= 0) ? (values[index - 1]) : (values[index - 2]);
		T p1 = values[index - 1];
		T p2 = values[index];
		T p3 = (index + 1 >= size) ? (values[index]) : (values[index + 1]);

		double t0 = (index - 1 <= 0) ? (tValues[index - 1]) : (tValues[index - 2]);
		double t1 = tValues[index - 1];
		double t2 = tValues[index];
		double t3 = (index + 1 >= size) ? (tValues[index]) : (tValues[index + 1]);

		double d1 = (t2 - t0);
		double d2 = (t3 - t1);

		if (d1 > -TINY && d1 < 0) d1 = -TINY;
		if (d1 < TINY && d1 >= 0) d1 = TINY;
		if (d2 > -TINY && d2 < 0) d2 = -TINY;
		if (d2 < TINY && d2 >= 0) d2 = TINY;

//#ifdef FANCY_SPLINES
		T m1 = (p2 - p0) * (1 - (t1 - t0) / d1);
		T m2 = (p3 - p1) * (1 - (t3 - t2) / d2);
//#else
//		T m1 = 0.5 * (p2 - p0);
//		T m2 = 0.5 * (p3 - p1);
//#endif

		t2 = t * t;
		t3 = t2 * t;

		//and now perform the interpolation using the four hermite basis functions from wikipedia
		return p1 * (2 * t3 - 3 * t2 + 1) + m1 * (t3 - 2 * t2 + t) + p2 * (-2 * t3 + 3 * t2) + m2 * (t3 - t2);
	}

	//	Returns the value of the ith knot. It is assumed that i is within the correct range.
	T getKnotValue(int i)
	{
		return values[i];
	}

	//	Returns the position of the ith knot. It is assumed that i is within the correct range.
	double getKnotPosition(int i)
	{
		return tValues[i];
	}

	//	Sets the value of the ith knot to val. It is assumed that i is within the correct range.
	void setKnotValue(int i, const T& val)
	{
		values[i] = val;
	}

	//	Sets the position of the ith knot to arb_position. It is assumed that i is within the correct range.
	void setKnotPosition(int i, double pos)
	{
		if (i - 1 >= 0 && tValues[i - 1] >= pos) return;
		if ((uint)(i + 1) < tValues.size() - 1 && tValues[i + 1] <= pos) return;
		tValues[i] = pos;
	}

	//	Return the smallest tValue or infinity if none
	double getMinPosition()
	{
		if (tValues.empty())
			return std::numeric_limits<double>::infinity();
		return tValues.front();
	}

	//	Return the largest tValue or -infinity if none
	double getMaxPosition()
	{
		if (tValues.empty())
			return -std::numeric_limits<double>::infinity();
		return tValues.back();
	}


	//returns the number of knots in this trajectory
	size_t get_knot_count() const
	{
		return tValues.size();
	}

	//This method is used to insert a new knot in the current trajectory
	void addKnot(double t, T val)
	{
		//first we need to know where to insert it, based on the t-values
		auto index = get_first_larger_index(t);

		tValues.insert(tValues.begin() + index, t);
		values.insert(values.begin() + index, val);
	}

	//	This method is used to remove a knot from the current trajectory.
	//	It is assumed that i is within the correct range.
	void removeKnot(int i)
	{
		tValues.erase(tValues.begin() + i);
		values.erase(values.begin() + i);
	}

	//	Simplify the curve by iteratively adding knots
	void simplify_catmull_rom(double maxError, int nbSamples = 100)
	{
		if (get_knot_count() < 3)
			return;

		double startTime = tValues.front();
		double endTime = tValues.back();

		GenericTrajectory<T> result;
		result.addKnot(startTime, values.front());
		result.addKnot(endTime, values.back());


		while (true)
		{
			double currError = 0;
			double currErrorTime = -std::numeric_limits<double>::infinity();

			for (int i = 0; i < nbSamples; ++i)
			{
				double interp = (double)i / (nbSamples - 1.0);
				double time = startTime * (1 - interp) + endTime * interp;
				double error = abs(result.evaluate_catmull_rom(time) - evaluate_catmull_rom(time));
				if (error > currError)
				{
					currError = error;
					currErrorTime = time;
				}
			}

			if (currError <= maxError)
				break;

			result.addKnot(currErrorTime, evaluate_catmull_rom(currErrorTime));
		}

		copy(result);
	}


	void copy(const GenericTrajectory<T>& other)
	{
		tValues.clear();
		values.clear();
		auto size = other.get_knot_count();

		tValues.reserve(size);
		values.reserve(size);
		for (int i = 0; i < size; ++i)
		{
			tValues.push_back(other.tValues[i]);
			values.push_back(other.values[i]);
		}
	}

	//	This method returns the index of the first knot whose value is larger than the parameter value t. If no such index exists (t is larger than any
	//	of the values stored), then values.size() is returned.
	size_t get_first_larger_index(double t)
	{
		const auto size = tValues.size();
		if (size == 0)
			return 0;
		if (t < tValues[(lastIndex + size - 1) % size])
			lastIndex = 0;
		for (auto i = 0; i < size; i++)
		{
			const auto index = (i + lastIndex) % size;
			if (t < tValues[index])
			{
				lastIndex = index;
				return index;
			}
		}
		return size;
	}
private:
	void evaluate_poly_coeff(int poly_dim)
	{
		poly_coeff = polyfit<T>(tValues, values, poly_dim);
	}

private:
	std::vector<double> tValues;
	std::vector<T> values;

	// A caching variable to optimize searching
	volatile size_t lastIndex;

	std::vector<T> poly_coeff;
};


typedef GenericTrajectory<double> Trajectory1D;
typedef GenericTrajectory<Vector3d> Trajectory3D;
