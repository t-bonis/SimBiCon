#include "Pose_controller.h"
#include "ConUtils.h"
#include "Reduced_character_state.h"
#include "Controller_interface.h"

Pose_controller::Pose_controller(FILE* f, Controller_interface& controller)
{
	m_parent_controller = &controller;

	//copy the current state of the character into the desired pose - makes sure that it's the correct size
	desired_pose = m_parent_controller->get_controlled_character()->get_state();

	desired_pose_dt = m_parent_controller->get_controlled_character()->get_state();

	//set initial values
	Reduced_character_state rs(desired_pose);
	rs.set_root_position(Vector3d());
	rs.set_root_velocity(Vector3d());
	rs.set_root_orientation(Quaternion(1, 0, 0, 0));
	rs.set_root_angular_velocity(Vector3d());

	for (auto& joint : m_parent_controller->get_controlled_character()->get_joints())
	{
		m_control_params.emplace_back();
		rs.set_joint_relative_ang_velocity(Vector3d(), joint->get_af_id());
		rs.set_joint_relative_orientation(Quaternion(), joint->get_af_id());
		joint->set_muscle_torque(Vector3d());
		joint->set_torque(Vector3d());
	}

	read_gains(f);


}



Pose_controller::Pose_controller(const Pose_controller& other, Controller_interface& controller)
{
	m_parent_controller = &controller;

	//copy the current state of the character into the desired pose - makes sure that it's the correct size
	desired_pose = m_parent_controller->get_controlled_character()->get_state();

	desired_pose_dt = m_parent_controller->get_controlled_character()->get_state();

	//set initial values
	Reduced_character_state rs(desired_pose);
	rs.set_root_position(Vector3d());
	rs.set_root_velocity(Vector3d());
	rs.set_root_orientation(Quaternion(1, 0, 0, 0));
	rs.set_root_angular_velocity(Vector3d());

	for (auto& joint : m_parent_controller->get_controlled_character()->get_joints())
	{
		rs.set_joint_relative_ang_velocity(Vector3d(), joint->get_af_id());
		rs.set_joint_relative_orientation(Quaternion(), joint->get_af_id());
		joint->set_muscle_torque(Vector3d());
		joint->set_torque(Vector3d());
	}

	m_control_params = other.m_control_params;
}

Vector3d Pose_controller::compute_pd_torque(const Quaternion& qRel, const Quaternion& qRelD, const Vector3d& wRel,
	const Vector3d& wRelD, Control_params* cParams)
{
	Vector3d axis;
	double angle;
	//the torque will have the form:
	// T = kp*(qRel - qRelD) + kd * (wRel - wRelD)


	(qRel * qRelD.get_inverse()).get_axis_and_angle(axis, angle);

	auto torque = axis * angle * cParams->kp + (wRel - wRelD)*cParams->kd;

	//now the torque is stored in parent coordinates - we need to scale it and apply torque limits
	//scale_and_limit_torque(&torque, cParams, qRel.get_conjugate());

	//and we're done...
	return torque;
}

// T = -kp*D(qRel + wRel*dt - qRelD(t+dt))-kd * (wRel + wwRel*dt - wRelD(t+dt))
Vector3d Pose_controller::compute_spd_torque(const Quaternion& qRel, const Quaternion& qRelD, const Vector3d& wRel,
	const Vector3d& wRelD, const Vector3d& wwRel, Control_params* cParams)
{
	Vector3d axis;
	double angle;
	//the torque will have the form:
	// T = -kp*(qRel + wRel*dt - qRelD)-kd * (wRel + wwRel*dt - wRelD)

	//std::cout <<  angle -> wRel.length;

	const auto angular_vel = Quaternion::get_rotation_quaternion(wRel.length()*SimGlobals::dt, wRel.unit());

	(qRelD * (angular_vel * qRel).get_inverse()).get_axis_and_angle(axis, angle);

	auto torque = -axis * angle * cParams->kp + (wRel + wwRel * SimGlobals::dt - wRelD)*cParams->kd;

	//now the torque is stored in parent coordinates - we need to scale it and apply torque limits
	//scale_and_limit_torque(&torque, cParams, qRel.get_conjugate());

	//and we're done...
	return torque;
}

void Pose_controller::limit_torque(Vector3d* torque, Control_params* cParams)
{
	if (torque->x < -cParams->scale.x * cParams->maxAbsTorque) torque->x = -cParams->scale.x * cParams->maxAbsTorque;
	if (torque->x > cParams->scale.x * cParams->maxAbsTorque) torque->x = cParams->scale.x * cParams->maxAbsTorque;
	if (torque->y < -cParams->scale.y * cParams->maxAbsTorque) torque->y = -cParams->scale.y * cParams->maxAbsTorque;
	if (torque->y > cParams->scale.y * cParams->maxAbsTorque) torque->y = cParams->scale.y * cParams->maxAbsTorque;
	if (torque->z < -cParams->scale.z * cParams->maxAbsTorque) torque->z = -cParams->scale.z * cParams->maxAbsTorque;
	if (torque->z > cParams->scale.z * cParams->maxAbsTorque) torque->z = cParams->scale.z * cParams->maxAbsTorque;
}

void Pose_controller::scale_and_limit_torque(Vector3d* torque, Control_params* cParams, const Quaternion& qToChild)
{
	//now change the torque to child coordinates
	*torque = qToChild.rotate(*torque);

	//and scale it differently along the main axis...
	torque->x *= cParams->scale.x;
	torque->y *= cParams->scale.y;
	torque->z *= cParams->scale.z;

	limit_torque(torque, cParams);

	// and now change it back to the original coordinates
	*torque = qToChild.get_conjugate().rotate(*torque);
}

void Pose_controller::compute_torques()
{
	Quaternion qRelD;
	Vector3d relAngVelD;

	Quaternion qRel;
	Vector3d wRel;
	Vector3d wwRel;

	Reduced_character_state rs(desired_pose);
	Reduced_character_state rs_dt(desired_pose_dt);

	for (auto& joint : m_parent_controller->get_controlled_character()->get_joints())
	{
		const auto joint_id = joint->get_af_id();
		if (m_control_params[joint_id].controlled)
		{
			//get the current relative orientation between the child and parent
			joint->compute_relative_orientation(qRel);
			//and the relative angular velocity, computed in parent coordinates
			joint->compute_relative_angular_velocity(wRel);

			joint->compute_relative_angular_acceleration(wwRel);

			//PD : now compute the torque (set in world coords)
			const auto torque = joint->get_parent_arb()->get_vector_world_coordinates(
				compute_pd_torque(qRel, rs.get_joint_relative_orientation(joint_id), wRel,
					rs.get_joint_relative_ang_velocity(joint_id), &m_control_params[joint_id]));
			joint->set_torque(torque);

			//SPD : now compute the torque (set in world coords)
			//const auto torque = joint->get_parent_arb()->get_vector_world_coordinates(
			//	compute_spd_torque(qRel, rs_dt.get_joint_relative_orientation(joint_id), wRel,
			//		rs_dt.get_joint_relative_ang_velocity(joint_id), wwRel,
			//		&m_control_params[joint_id]));
			//joint->set_torque(torque);

		}
		else
		{
			joint->set_torque(Vector3d());
		}
	}
}

void Pose_controller::read_gains(FILE* f)
{
	if (f == nullptr)
		throw std::logic_error("Can't open file");

	//have a temporary buffer used to read the file line by line...
	char buffer[200];

	//this is where it happens.
	while (!feof(f))
	{
		//get a line from the file...
		fgets(buffer, 200, f);
		if (strlen(buffer) > 195)
			throw std::logic_error("The input file contains a line that is longer than ~200 characters - not allowed");
		char* line = lTrim(buffer);
		const auto line_type = get_con_line_type(line);
		switch (line_type)
		{
		case con_utils::con_pd_gains_end:
			return;
		case con_utils::con_pd_gains_start:
		case con_utils::comment:
			break;
		default:
			parse_gain_line(line);
			break;
		}
	}
	throw std::logic_error("Incorrect controller input file: No ' Kp Kd MaxT ' found");
}

std::vector<double> Pose_controller::get_all_gains()
{
	std::vector<double> output;

	for (auto control_params : m_control_params)
	{
		output.push_back(control_params.kd);
		output.push_back(control_params.kp);
	}
	return output;
}

void Pose_controller::parse_gain_line(char* line)
{
	double kp, kd, tMax, scX, scY, scZ;
	char jName[100];
	const int nrParams = sscanf_s(line, "%s %lf %lf %lf %lf %lf %lf\n", jName, unsigned(_countof(jName)), &kp, &kd,
		&tMax, &scX, &scY, &scZ);
	if (nrParams == 2)
	{
		const auto tmp = m_parent_controller->get_controlled_character()->get_joint_by_name(jName)->get_child_arb()->get_moi();
		double maxM = std::max(tmp.x, tmp.y);
		maxM = std::max(maxM, tmp.z);
		kd = kp / 10;
		tMax = 10000;
		scX = tmp.x / maxM;
		scY = tmp.y / maxM;
		scZ = tmp.z / maxM;
	}
	else if (nrParams != 7)
	{
		throw std::logic_error(
			"To specify the gains, you need: 'joint name Kp Kd Tmax scaleX scaleY scaleZ'! --> " + std::string(line));
	}
	const auto j_index = m_parent_controller->get_controlled_character()->get_object_id(jName);
	if (j_index == size_t(-1))
		throw std::logic_error("Cannot find joint: " + std::string(jName));
	m_control_params[j_index] = Control_params(kp, kd, tMax, Vector3d(scX, scY, scZ));
}

//sets the targets to match the current state of the character
void Pose_controller::set_targets_from_state() const
{
	Reduced_character_state rs(desired_pose);
	Quaternion qTemp;
	for (auto& joint : m_parent_controller->get_controlled_character()->get_joints())
	{
		joint->compute_relative_orientation(qTemp);
		rs.set_joint_relative_orientation(qTemp, joint->get_af_id());
	}
}
