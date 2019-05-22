#include "MuscleTendonUnit.h"
#include <cmath>

// values cf OpenSim, Lee2014
const double MuscleTendonUnit::GAMMA = 0.5;
const double MuscleTendonUnit::E_0 = 0.6; //0.6 young, 0.5 old humans [Thelen2003]
const double MuscleTendonUnit::K_PE = 4;
const double MuscleTendonUnit::A_F = 0.3;
const double MuscleTendonUnit::N_V_MAX = 10;
const double MuscleTendonUnit::N_F_LEN = 1.8;

MuscleTendonUnit::MuscleTendonUnit(double _fMax, double _oLength, double _sLength)
	: optimalLength(_oLength), slackLength(_sLength), fMax(_fMax)
{
	//assumed initial state
	lCE = optimalLength;
	vCE = 0;
	initActivation = 0.0001;
	initExcitation = 0.0001;
	activation = initActivation;
	fOut = 0;
}

MuscleTendonUnit::~MuscleTendonUnit() = default;

// force computations cf Locomotion Control for Many-Muscle Humanoids - Lee et al 2014
double MuscleTendonUnit::active_force_length(double length) const
{
	return exp(-(length - 1) * (length - 1) / GAMMA);
}

double MuscleTendonUnit::passive_force_length(double length) const
{
	if (length <= 1)
	{
		return 0;
	}
	return (exp(K_PE * (length - 1) / E_0) - 1) / (exp(K_PE) - 1);
}

double MuscleTendonUnit::force_velocity(double velocity) const
{
	if (velocity <= 0)
	{
		return (velocity + N_V_MAX) / (N_V_MAX - velocity / A_F);
	}
	const double x = velocity * (2 + 2 / A_F);
	const double y = N_V_MAX * (N_F_LEN - 1);
	return (N_F_LEN * x + y) / (x + y); // in Lee2014, N_F_LEN should not be normalized here ...
}

double MuscleTendonUnit::calcCEForce()
{
	return activation * active_force_length(lCE / optimalLength) * force_velocity(vCE / optimalLength);
}

double MuscleTendonUnit::calcPEForce()
{
	return passive_force_length(lCE / optimalLength);
}

void MuscleTendonUnit::calcForce()
{
	fOut = fMax * calcCEForce() + calcPEForce(); // * cos(pennationAngle)
}
