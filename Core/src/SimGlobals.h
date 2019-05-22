#pragma once
#include "MathLib/src/Vector3d.h"


/**
	This class is used as a container for all the constants that are pertinent for the physical simulations, the controllers, etc.
*/

class SimGlobals
{
public:
	SimGlobals() = default;

	~SimGlobals() = default;

	SimGlobals(const SimGlobals& other) = delete;
	SimGlobals(SimGlobals&& other)  = delete;
	SimGlobals& operator=(const SimGlobals& other) = delete;
	SimGlobals& operator=(SimGlobals&& other)  = delete;

public:
	//We will assume that the gravity is in the y-direction (this can easily be changed if need be), and this value gives its magnitude. 
	static double gravity;
	//this is the direction of the up-vector
	static Vector3d up;
	//and this is the desired time interval for each simulation timestep (does not apply to animations that are played back).
	static double dt;
	static double speed_multiplier;

	//(muscles)
	static double timeFrequency;
	//save data to a file (torques, activations ...)
	static int saveData;
	static bool fileOpened;
	static FILE* fileData;

	static double rootSagittal;
	static double rootLateral;
	static double swingHipSagittal;
	static double swingHipLateral;
	static double stanceAngleSagittal;
	static double stanceAngleLateral;
	static double stanceKnee;


	static double COMOffsetX;
	static double COMOffsetZ;
	static bool use_muscle_actuation;
	static size_t m_interval_between_measure;
	static int nb_threads;
	static bool learning;
	static double desired_heading;
	static bool foot_flat_on_ground;

	static bool draw;
	static bool force_end;

	static bool print_value;

	static double velDCoronal;
	static double velDSagittal;

	static double virtual_force_effectiveness;

	static double step_width;

	static bool optimization;

	static bool force_ipm;

	static double ipm_alteration_effectiveness;
	static Vector3d avg_speed;
};
