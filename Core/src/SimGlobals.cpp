#include "SimGlobals.h"

//initialize all the parameters to some sensible values.

//give this a very high value so that we can use the scripted values in the rb specs for the value to use
double SimGlobals::gravity = -9.81;
Vector3d SimGlobals::up = Vector3d(0, 1, 0);
double SimGlobals::timeFrequency = 2000; //Default value 2000Hz
double SimGlobals::dt = 1.0 / timeFrequency;
double SimGlobals::speed_multiplier = 1 ;

bool SimGlobals::use_muscle_actuation = false;
size_t SimGlobals::m_interval_between_measure = 7;
int SimGlobals::nb_threads = 6;
bool SimGlobals::force_end = false;



double SimGlobals::desired_heading = 0.0;

bool SimGlobals::foot_flat_on_ground = false;

double SimGlobals::rootSagittal = 0;
double SimGlobals::rootLateral = 0;
double SimGlobals::swingHipSagittal = 0;
double SimGlobals::swingHipLateral = 0;
double SimGlobals::stanceAngleSagittal = 0;
double SimGlobals::stanceAngleLateral = 0;
double SimGlobals::stanceKnee = 0;

double SimGlobals::velDSagittal = 0.7;//0.95;
double SimGlobals::velDCoronal = 0;

double SimGlobals::virtual_force_effectiveness = 1;

double SimGlobals::step_width = 0.1;

bool SimGlobals::force_ipm = false;

bool SimGlobals::draw = false;

bool SimGlobals::print_value = false;

bool SimGlobals::learning = false;

bool SimGlobals::optimization = false;

double SimGlobals::ipm_alteration_effectiveness = 1;

Vector3d SimGlobals::avg_speed = Vector3d();