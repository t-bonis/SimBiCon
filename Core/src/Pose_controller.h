#pragma once
#include "Utils/src/Utils.h"
#include "Joint.h"
#include "Reduced_character_state.h"


class Controller_interface;

//	This class is used as a container for the properties needed by a PD controller
//	A pose controller is used to have the character track a given pose.
//	Each pose is given as a series of relative orientations, one for
//	each parent-child pair (i.e. joint). Classes extending this one 
//	have to worry about setting the desired relative orientation properly.
class Pose_controller
{
public:
	struct Control_params
	{
		//this variable is set to true if the current joint is being actively controlled, false otherwise
		bool controlled;
		//these two variables are the proportional and derivative gains for the PD controller used to compute the torque
		double kp, kd;
		//this is the maximum absolute value torque allowed at this joint
		double maxAbsTorque;
		//the torques about the about the x, y and z axis will be scaled differently to account for the potentially different principal moments of inertia
		//of the child
		Vector3d scale;

		//and this is the coordinate frame that the desired orientation is specified in, if relToCharFrame is true
		Quaternion char_frame;

		Control_params()
		{
			controlled = false;
			kp = kd = 0;
			maxAbsTorque = 0;
			scale = Vector3d();
		}

		Control_params(double a_kp, double a_kd, double a_maxAbsTorque, Vector3d a_scale)
		{
			controlled = true;
			kp = a_kp;
			kd = a_kd;
			maxAbsTorque = a_maxAbsTorque;
			scale = a_scale;
		}
	};

public:
	Pose_controller(FILE* f, Controller_interface& controller);
	Pose_controller(const Pose_controller& other, Controller_interface& controller);
	Pose_controller() = delete;

	~Pose_controller() = default;

	Pose_controller(const Pose_controller& other) = delete;
	Pose_controller(Pose_controller&& other) = delete;
	Pose_controller& operator=(const Pose_controller& other) = delete;
	Pose_controller& operator=(Pose_controller&& other) = delete;

	//	This method is used to compute the torques, based on the current and desired poses
	void compute_torques();

	//This method is used to compute the PD torque, given the current relative orientation of two coordinate frames (child and parent),
	//the relative angular velocity, the desired values for the relative orientation and ang. vel, as well as the virtual motor's
	//PD gains. The torque returned is expressed in the coordinate frame of the 'parent'.
	static Vector3d compute_pd_torque(const Quaternion& qRel, const Quaternion& qRelD, const Vector3d& wRel,
		const Vector3d& wRelD, Control_params* cParams);
	static Vector3d compute_spd_torque(const Quaternion& qRel, const Quaternion& qRelD, const Vector3d& wRel,
	                            const Vector3d& wRelD, const Vector3d& wwRel, Control_params* cParams);

	//This method is used to scale and apply joint limits to the torque that is passed in as a parameter. The orientation that transforms
	//the torque from the coordinate frame that it is currently stored in, to the coordinate frame of the 'child' to which the torque is
	//applied to (it wouldn't make sense to scale the torques in any other coordinate frame)  is also passed in as a parameter.
	static void scale_and_limit_torque(Vector3d* torque, Control_params* cParams, const Quaternion& qToChild);

	//This method is used to apply joint limits to the torque passed in as a parameter. It is assumed that
	//the torque is already represented in the correct coordinate frame
	static void limit_torque(Vector3d* torque, Control_params* cParams);

	//This method is used to read the gain coefficients, as well as max torque allowed for each joint
	//from the file that is passed in as a parameter.
	void read_gains(FILE* f);

	std::vector<double> get_all_gains();

	//sets the targets to match the current state of the character
	void set_targets_from_state() const;

	//This method is used to parse the information passed in the string. This class knows how to read lines
	//that have the name of a joint, followed by a list of the pertinent parameters. If this assumption is not held,
	//then classes extended this one are required to provide their own implementation of this simple parser
	void parse_gain_line(char* line);

	void set_kp(size_t index, double value)
	{
		m_control_params[index].kp = value;
	}

	void set_kd(size_t index, double value)
	{
		m_control_params[index].kd = value;
	}

	double get_kp(size_t index)
	{
		return m_control_params[index].kp;
	}

	double get_kd(size_t index)
	{
		return m_control_params[index].kd;
	}
public:
	std::vector<double> desired_pose;

	std::vector<double> desired_pose_dt;

private:
	Controller_interface* m_parent_controller;

	//this is the pose that the character is aiming at achieving

	//this is the array of joint properties used to specify the 
	std::vector<Control_params> m_control_params;
};
