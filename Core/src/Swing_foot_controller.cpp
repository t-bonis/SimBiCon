#include "Swing_foot_controller.h"
#include "Reduced_character_state.h"
#include "Velocity_controller.h"
#include "Two_link_ik.h"
#include "CCD_algorithm.h"


Swing_foot_controller::Swing_foot_controller(Character& character, SimBiCon_framework& control_framework)
	: Controller_interface(character, control_framework)
{
	swing_foot_traj = nullptr;
	evolve_sagittal = false;
	steps_left_sag = 0;

	ipm_active = false;
	_is_recovery_step = false;

	leg_length = 0.90;//the leg does 0.94 but since the leg ain't centered on the pelvis the effective length is a bit lower
	coronal_step_width = 0.1;
	ipm_alt_sagittal = 0;
	ipm_alt_coronal_left = 0;
	ipm_alt_coronal_right = 0;


	swing_foot_trajectory_coronal.addKnot(0, 0);
	swing_foot_trajectory_coronal.addKnot(1, 0);

	swing_foot_trajectory_sagittal.addKnot(0, 0);
	swing_foot_trajectory_sagittal.addKnot(1, 0);

	init_new_character_step(false, 0, NULL);
}


Swing_foot_controller::~Swing_foot_controller()
{
	//swing_foot_trajectory_coronal.clear();
	//swing_foot_trajectory_sagittal.clear();

	//nothing to do the trajectory is just a pointer for easy access
}

void Swing_foot_controller::restart()
{
	coronal_step_width = 0.1;
	ipm_alt_sagittal = 0;
	ipm_alt_coronal_left = 0;
	ipm_alt_coronal_right = 0;

	//init_new_character_step(false,0,NULL);
}

void Swing_foot_controller::init_new_character_step(bool is_recovery_step, double cur_state_time,
	Velocity_controller* vel_control)
{
	//reset the trajectories
	//    swing_foot_trajectory_coronal.clear();
	//    swing_foot_trajectory_sagittal.clear();

	//    swing_foot_trajectory_coronal.addKnot(0, 0);
	//    swing_foot_trajectory_sagittal.addKnot(0, 0);


	coronal_step_width = SimGlobals::step_width;
	heading = Quaternion::get_rotation_quaternion(SimGlobals::desired_heading, SimGlobals::up);

	_is_recovery_step = is_recovery_step;

	//read the velocity from the interface
	if (velD_sagittal != SimGlobals::velDSagittal) {
		velD_sagittal = SimGlobals::velDSagittal;
	}
	if (velD_coronal != SimGlobals::velDCoronal) {
		velD_coronal = SimGlobals::velDCoronal;
	}

	this->cur_state_time = cur_state_time;

	ipm_active = false;


	compute_ipm_alteration(vel_control);
}

void Swing_foot_controller::preprocess_simulation_step()
{
	phi = std::dynamic_pointer_cast<SimBiCon>(m_framework->get_controllers()[0])->get_phase();
	cur_state_time = std::dynamic_pointer_cast<SimBiCon>(m_framework->get_controllers()[0])->get_current_state_time();

	Quaternion q = Quaternion::get_rotation_quaternion(SimGlobals::desired_heading, SimGlobals::up);

	Vector3d vd(velD_coronal, 0, velD_sagittal);
	vd = q.to_world_coordinate(vd);

	vd = m_controlled_character->get_heading().to_local_coordinate(vd);

	velD_coronal_current_heading = vd.x;
	velD_sagittal_current_heading = vd.z;

	heading = q;


	v = m_controlled_character->get_heading().to_local_coordinate(m_controlled_character->get_com_velocity());
}

void Swing_foot_controller::simulation_step(const Quaternion& desired_heading_pelvis)
{

	double cur_phi = MIN(phi, 1);
	double future_phi = MIN(cur_phi + SimGlobals::dt / cur_state_time, 1);

	//add the point for future values
	//    swing_foot_trajectory_coronal.addKnot(100, 0);
	//    swing_foot_trajectory_sagittal.addKnot(100, 0);

	//set the phases
	int pos = swing_foot_trajectory_sagittal.get_knot_count() - 2;
	swing_foot_trajectory_sagittal.setKnotPosition(pos, cur_phi);
	swing_foot_trajectory_sagittal.setKnotPosition(pos + 1, future_phi);

	swing_foot_trajectory_coronal.setKnotPosition(pos, cur_phi);
	swing_foot_trajectory_coronal.setKnotPosition(pos + 1, future_phi);

	if (ipm_needed() || ipm_active || (!user_specified_swing_foot_traj_xz_exist())) {
		if (!ipm_active) {
			Articulated_rigid_body* swing_foot = static_cast<Articulated_rigid_body*>(m_controlled_character->get_swing_foot());
			swing_foot_start_pos = swing_foot->get_cm_position();
			phase_start_pos = cur_phi;
			ipm_active = true;
		}


		//compute the ipm
		Point3d com_pos = m_controlled_character->get_com();
		Point3d com_vel = m_controlled_character->get_com_velocity();
		Vector3d step = ipm_compute_swing_foot_location(com_pos, phi);
		swing_foot_trajectory_coronal.setKnotValue(0, step.x);
		swing_foot_trajectory_sagittal.setKnotValue(0, step.z);



		step = ipm_compute_swing_foot_location(com_pos + Vector3d(com_vel) * SimGlobals::dt, future_phi);
		swing_foot_trajectory_coronal.setKnotValue(1, step.x);
		swing_foot_trajectory_sagittal.setKnotValue(1, step.z);
		//to give some gradient information, here's what the position will be a short time later...


	}
	else {
		//*
		swing_foot_trajectory_sagittal.setKnotPosition(0, 0);
		swing_foot_trajectory_sagittal.setKnotPosition(1, 1);

		swing_foot_trajectory_coronal.setKnotPosition(0, 0);
		swing_foot_trajectory_coronal.setKnotPosition(1, 1);
		//*/

		use_specified_swing_foot_location(cur_phi, future_phi);
	}


	//    std::ostringstream oss;
	//    oss<<"sagital traj count";
	//      <<swing_foot_trajectory_sagittal.getKnotValue(0)<<"  "<<
	//         swing_foot_trajectory_sagittal.getKnotValue(1);
	//    for (int i=pos;i<pos+2;++i){
	//        oss<<swing_foot_trajectory_sagittal.getKnotValue(i)<<"  ";
	//    }
	//    std::cout<<oss.str();

	//set some of these settings
	//the commented ones are used to modify the trajectories (but srly i don't care right now)
	//setUpperBodyPose(ubSagittalLean, ubCoronalLean, ubTwist);
	//setKneeBend(kneeBend);
	//setDuckWalkDegree((lowLCon->stance == LEFT_STANCE) ? (-duckWalk) : (duckWalk));
	//setDesiredHeading(desiredHeading);
	//this one may be usefull but I'll use constant speed for now
	//setVelocities(velDSagittal, velDCoronal);

	//adjust for panic mode or unplanned terrain...
	adjust_step_height();


	compute_swing_leg_target(SimGlobals::dt, desired_heading_pelvis);

	/*
	double sh_a,sh_a1,sh_a2;
	sh_a=(character->swing_hip()->child()->getOrientation()*
		  character->swing_hip()->parent()->getOrientation().getComplexConjugate()).getRotationAngle(Vector3d(0,0,0));
	sh_a1=(character->swing_hip()->child()->getOrientation()).getRotationAngle(Vector3d(0,0,0));
	sh_a2=(character->swing_hip()->parent()->getOrientation().getComplexConjugate()).getRotationAngle(Vector3d(0,0,0));


	std::ostringstream oss;
	oss<<sh_a<<"  "<<sh_a1<<"  "<<sh_a2;
	std::cout<<oss.str();

	//*/
}








bool Swing_foot_controller::ipm_needed() {
	if (SimGlobals::force_ipm) { return true; }

	if (_is_recovery_step) { return true; }

	if (v.y < 0) { return true; }

	if (is_early_step()) { return false; }

	return false;
}

bool Swing_foot_controller::is_early_step(double limit)
{
	if (phi < limit) { return true; }

	return false;
}

Vector3d Swing_foot_controller::ipm_compute_swing_foot_location(const Point3d &comPos, double phase)
{
	Vector3d step = ipm_compute_step_location();


	bool override_advanced_help = true;
	//applying the IP prediction would make the character stop, so take a smaller step if you want it to walk faster, or larger
	//if you want it to go backwards
	//step.z += -con->velDSagittal / 20;
	//    if (v.y < 0&& phase>0.2)
	if (override_advanced_help || velD_sagittal_current_heading * velD_sagittal >= 0) {
		step.z += -velD_sagittal_current_heading / 10;
		step.z += ipm_alt_sagittal * SimGlobals::ipm_alteration_effectiveness;
	}
	else {
		step.z *= 1.20;
	}
	//and adjust the stepping in the coronal plane in order to account for desired step width...
	if (override_advanced_help || velD_coronal_current_heading * velD_coronal >= 0) {
		step.x = ipm_adjust_coronal_step_location(step.x, phase);
	}
	else {
		step.x *= 1.20;
	}

	boundToRange(&step.z, -0.4 * leg_length, 0.4 * leg_length);
	boundToRange(&step.x, -0.4 * leg_length, 0.4 * leg_length);

	Vector3d result;
	Vector3d initialStep(comPos, swing_foot_start_pos);


	if (false) {
		std::cout << "ipm target: " << step.z << "   " << initialStep.z <<
			std::endl;
	}
	initialStep = m_controlled_character->get_heading().inverse_rotate(initialStep);
	//when phi is small, we want to be much closer to where the initial step is - so compute this quantity in character-relative coordinates
	//now interpolate between this position and initial foot position - but provide two estimates in order to provide some gradient information
	//double t = (phase - phase_start_pos);
	//t=1-t/(0.9-phase_start_pos);
	double t = std::max(1.0 - phase, 0.0) / 0.9;
	boundToRange(&t, 0, 1);

	//use a square law
	t = t * t;

	Vector3d suggestedViaPoint;
	//alternateFootTraj.clear();
	bool need_to_step_around_stance_ankle = false;

	//TODO integrate the leg intersection.
	//FOr now let's forget the legs intersections
	//if (phase < 0.8 && shouldPreventLegIntersections && getPanicLevel() < 0.5)
	//	needToStepAroundStanceAnkle = detectPossibleLegCrossing(step, &suggestedViaPoint);


	if (need_to_step_around_stance_ankle) {
		/*
		//use the via point...
		Vector3d currentSwingStepPos(comPos, con->getSwingFootPos());
		currentSwingStepPos = con->getCharacterFrame().inverseRotate(initialStep); currentSwingStepPos.y = 0;
		//compute the phase for the via point based on: d1/d2 = 1-x / x-phase, where d1 is the length of the vector from
		//the via point to the final location, and d2 is the length of the vector from the swing foot pos to the via point...
		double d1 = (step - suggestedViaPoint).length(); double d2 = (suggestedViaPoint - currentSwingStepPos).length(); if (d2 < 0.0001) d2 = d1 + 0.001;
		double c = d1 / d2;
		double viaPointPhase = (1 + phase*c) / (1 + c);
		//now create the trajectory...
		alternateFootTraj.addKnot(0, initialStep);
		alternateFootTraj.addKnot(viaPointPhase, suggestedViaPoint);
		alternateFootTraj.addKnot(1, step);
		//and see what the interpolated position is...
		result = alternateFootTraj.evaluate_catmull_rom(1.0 - t);
		//		std::cerr<<"t: %lf\n", 1-t);
		*/
	}
	else {
		result.add_scaled_vector(step, 1.0 - t);
		result.add_scaled_vector(initialStep, t);
	}

	result.y = 0;


	/*
	suggestedFootPosDebug = result;
	*/
	return result;
}

Vector3d Swing_foot_controller::ipm_compute_step_location() {
	Vector3d step;
	Point3d rotation_point = m_controlled_character->get_stance_foot()->get_cm_position();
	double h = fabs(m_controlled_character->get_com().y - rotation_point.y);
	step.x = v.x * sqrt(h / 9.8 + v.x * v.x / (4 * 9.8*9.8))*1.3;
	step.z = v.z * sqrt(h / 9.8 + v.z * v.z / (4 * 9.8*9.8))*1.3;
	step.y = 0;

	/*
	//original equations
	step.x = v.x * sqrt(h / 9.8 + v.x * v.x / (4 * 9.8*9.8)) * 1.3;
	step.z = v.z * sqrt(h / 9.8 + v.z * v.z / (4 * 9.8*9.8)) * 1.1;
	step.y = 0;
	//*/
	return step;
}


double Swing_foot_controller::ipm_adjust_coronal_step_location(double IPPrediction, double phase)
{

	double speed_control = v.x;


	//this addjust to the specifed step width
	double stepWidth = coronal_step_width / 2;
	stepWidth = (m_controlled_character->get_stance() == Character::left) ? (-stepWidth) : (stepWidth);

	//this help to diminish the fact that the caracter turn the leg when inside the water
	if (phase > 0.8) {
		if (stepWidth*speed_control < 0) {
			IPPrediction -= stepWidth;
		}
	}
	IPPrediction += stepWidth;

	//    if (v.y < 0 && phase>0.2)
	{
		IPPrediction -= velD_coronal_current_heading / 15; //techniquement 12 converge vers le truc parfait mais ya un peut d'overshot � la base
		double ipm_alt_coronal = 0;
		if (m_controlled_character->get_stance() == Character::left) {
			ipm_alt_coronal = ipm_alt_coronal_left;
		}
		else {
			ipm_alt_coronal = ipm_alt_coronal_right;
		}
		//IPPrediction += ipm_alt_coronal*SimGlobals::ipm_alteration_effectiveness;
	}


	//I'll disable all the panic system for now ...
	/*
	//now for the step in the coronal direction - figure out if the character is still doing well - panic = 0 is good, panic = 1 is bad...
	double panicLevel = 1;

	if (con->getStance() == LEFT_STANCE){
			panicLevel = getValueInFuzzyRange(con->get_d().x, 1.15 * stepWidth, 0.5 * stepWidth, 0.25 * stepWidth, -0.25 * stepWidth);
			panicLevel += getValueInFuzzyRange(con->get_v().x, 2 * stepWidth, stepWidth, -stepWidth, -stepWidth*1.5);
	}
	else{
			panicLevel = getValueInFuzzyRange(con->get_d().x, -0.25 * stepWidth, 0.25 * stepWidth, 0.5 * stepWidth, 1.15 * stepWidth);
			panicLevel += getValueInFuzzyRange(con->get_v().x, -stepWidth*1.5, -stepWidth, stepWidth, 2 * stepWidth);
	}
	boundToRange(&panicLevel, 0, 1);
	Trajectory1D offsetMultiplier;
	offsetMultiplier.addKnot(0.05, 0); offsetMultiplier.addKnot(0.075, 1 / 2.0);
	double offset = stepWidth * offsetMultiplier.evaluate_linear(fabs(stepWidth));
	//	if (IPPrediction * stepWidth < 0) offset = 0;
	//if it's doing well, use the desired step width...
	IPPrediction = panicLevel * (IPPrediction + offset) + (1 - panicLevel) * stepWidth;
	con->comOffsetCoronal = (1 - panicLevel) * stepWidth;

	//	if (panicLevel >= 1)
	//		std::cerr<<"panic level: %lf; d.x = %lf\n", panicLevel, lowLCon->d.x);
	//*/

	return IPPrediction;
}

void Swing_foot_controller::adjust_step_height()
{
	
	//con->unplannedForHeight = 0;
	//srly fu the oracle for now
	/*
	if (wo != NULL){
			//the trajectory of the foot was generated without taking the environment into account, so check to see if there are any un-planned bumps (at somepoint in the near future)
			con->unplannedForHeight = wo->getWorldHeightAt(con->getSwingFootPos() + con->swingFoot->getCMVelocity() * 0.1) * 1.5;
	}//*/

	//if the foot is high enough, we shouldn't do much about it... also, if we're close to the start or end of the
	//walk cycle, we don't need to do anything... the thing below is a quadratic that is 1 at 0.5, 0 at 0 and 1...
	/*
	double panicIntensity = -4 * con->getPhase() * con->getPhase() + 4 * con->getPhase();
	panicIntensity *= getPanicLevel();
	con->panicHeight = panicIntensity * 0.05;
	*/
}

void Swing_foot_controller::use_specified_swing_foot_location(double cur_phi, double future_phi) {

	//we know that the points what we want to modify are the 2 last that were defined
	//since we specify the position relative to the hip knot and that the system is based on positions relatives to the mass center
	//we need to add the bifference before giving the values to the system
	Point3d com = m_controlled_character->get_com();
	Point3d hip = m_controlled_character->get_swing_hip()->get_pos_in_global_coords();
	double dx = hip.x - com.x;
	double dz = hip.z - com.z;

	Vector3d to_hip = m_controlled_character->get_heading().get_conjugate().rotate((Vector3d(dx, 0, dz)));
	dx = to_hip.x;
	dz = to_hip.z;

	float val, valf;
	Vector3d z_axis(0, 0, 1);
	int point_count = swing_foot_trajectory_sagittal.get_knot_count();
	val = swing_foot_traj->components[2]->base_trj.evaluate_catmull_rom(cur_phi);
	valf = swing_foot_traj->components[2]->base_trj.evaluate_catmull_rom(future_phi);
	swing_foot_trajectory_sagittal.setKnotValue(point_count - 2, val + dz);
	swing_foot_trajectory_sagittal.setKnotValue(point_count - 1, valf + dz);


	Vector3d x_axis(1, 0, 0);
	double sign = (m_controlled_character->get_stance() == Character::left) ? -1.0 : 1.0;
	point_count = swing_foot_trajectory_coronal.get_knot_count();
	val = sign * swing_foot_traj->components[0]->base_trj.evaluate_catmull_rom(cur_phi);
	valf = sign * swing_foot_traj->components[0]->base_trj.evaluate_catmull_rom(future_phi);
	swing_foot_trajectory_coronal.setKnotValue(point_count - 2, val + dx);
	swing_foot_trajectory_coronal.setKnotValue(point_count - 1, valf + dx);


	//the height will be handled somewhere else( because it has to be always generated)


}

bool Swing_foot_controller::user_specified_swing_foot_traj_xz_exist()
{
	Vector3d x_axis(1, 0, 0);
	if(!swing_foot_traj)
	{
		return false;
	}
	Trajectory_component* comp_x = swing_foot_traj->get_component(x_axis);
	if ((comp_x == nullptr) || (comp_x->is_implicit())) {
		return false;
	}

	Vector3d z_axis(0, 0, 1);
	Trajectory_component *comp_z = swing_foot_traj->get_component(z_axis);
	if ((comp_z == nullptr) || (comp_z->is_implicit())) {
		return false;
	}


	return true;
}

void Swing_foot_controller::compute_swing_leg_target(double dt, Quaternion desired_heading_pelvis) {
	Point3d pNow, pFuture;

	Point3d com_pos = m_controlled_character->get_com();

	Quaternion current_heading = m_controlled_character->get_heading();
	current_heading = desired_heading_pelvis;
	pNow = swing_foot_target_location(phi, com_pos, current_heading);
	pFuture = swing_foot_target_location((MIN(phi + dt / cur_state_time, 1)), com_pos + m_controlled_character->get_com_velocity() * dt, current_heading);



	if (false) {
		Articulated_rigid_body* swing_foot = static_cast<Articulated_rigid_body*>(m_controlled_character->get_swing_foot());
		Point3d pos = swing_foot->get_cm_position();

		Point3d hip_pos = m_controlled_character->get_swing_hip()->get_pos_in_global_coords();
		std::cout << "test: " << pos.z - swing_foot_start_pos.z << "   " <<
			(pNow.z) - swing_foot_start_pos.z << "   " <<
			swing_foot_start_pos.z <<
			std::endl;
	}

	desired_position = pNow;

	Joint* hip = m_controlled_character->get_swing_hip();
	Joint* knee = hip->get_child_arb()->get_children_joints()[0];
	Joint* ankle = knee->get_child_arb()->get_children_joints()[0];
	Vector3d parentAxis(hip->get_joint_position_in_child(), knee->get_joint_position_in_parent());
	Vector3d childAxis(knee->get_joint_position_in_child(), ankle->get_joint_position_in_parent());

	//    if (!_is_recovery_step)
	{
		//*
		Point3d hip_pos = knee->get_parent_arb()->get_parent_joint()->get_pos_in_global_coords();
		double max_leg_length = (parentAxis.length() + childAxis.length())*0.993;
		Vector3d vect_step = pNow - hip_pos;
		if (vect_step.length() > max_leg_length&&ipm_needed()) {
			double height = vect_step.y;
			vect_step.y = 0;
			if (std::abs(height) > max_leg_length*0.999) {
				vect_step *= 0;
			}
			else {
				vect_step.toUnit();
				vect_step *= std::sqrt(max_leg_length*max_leg_length - height * height);
			}
			vect_step.y = height;
			pNow = Vector3d(vect_step + Vector3d(hip_pos));
			//std::cout<<"pNow too far, vector length reduced" <<vect_step.length()-max_leg_length;
		}
		//*/
		//*
		vect_step = pFuture - hip_pos;
		if (vect_step.length() > max_leg_length&&ipm_needed()) {
			double height = vect_step.y;
			vect_step.y = 0;
			if (std::abs(height) > max_leg_length*0.999) {
				vect_step *= 0;
			}
			else {
				vect_step.toUnit();
				vect_step *= std::sqrt(max_leg_length*max_leg_length - height * height);
			}
			vect_step.y = height;
			pFuture = Vector3d(vect_step + Vector3d(hip_pos));
			//        std::cout<<"pFuture   too far, vector length reduced";
		}
		//*/
	}

	//    std::ostringstream oss;
	//oss<<"ipm pos: "<<(pNow-com_pos).x<<"  "<<(pNow-com_pos).z;
	//    Point3d ankle_pos=character->swing_foot()->getWorldCoordinates(ankle->child_joint_position());
	//    oss<<std::endl<<"ankle pos: "<<(ankle_pos-pNow).x<<"  "<<(ankle_pos-pNow).z;
	//    std::cout<<oss.str();


	//first we set no orientation the hip base
	//meaning that the computation will be some as if we want the hip rotation to be done
	//in the y-z plane
	Vector3d swingLegPlaneOfRotation = Vector3d(-1, 0, 0);

	//compute_inversed_kinematics(hip->get_af_id(), knee->get_af_id(), parentAxis, swingLegPlaneOfRotation, Vector3d(-1, 0, 0),
	//	childAxis, pNow, true, pFuture, dt);


}

Point3d Swing_foot_controller::swing_foot_target_location(double t, const Point3d &com, const Quaternion &charFrameToWorld) {
	Vector3d step;
	step.z = swing_foot_trajectory_sagittal.evaluate_catmull_rom(t);
	step.x = swing_foot_trajectory_coronal.evaluate_catmull_rom(t);

	//now transform this vector into world coordinates
	step = charFrameToWorld.rotate(step);
	//add it to the com location
	step = Vector3d(com) + step;

	//finally, set the desired height of the foot
	///TODO reactivate the panicheight an the unplanned height
	//*
	//version relative to the hip
	Vector3d y_axis(0, 1, 0);
	Point3d hip_pos = m_controlled_character->get_swing_hip()->get_pos_in_global_coords();
	double delta_y = -pow(phi - 0.5,2) + 0.25/*swing_foot_traj->get_component(y_axis)->base_trj.evaluate_catmull_rom(t)*/;
	step.y = hip_pos.y - delta_y; // + panicHeight + unplannedForHeight;
	//*/

	//    std::cout<<step.y<<std::endl;
	/*
	//version relative to the ground
	double delta_y=swing_foot_traj->get_component(y_axis)->baseTraj.evaluate_catmull_rom(t);
	step.y = delta_y; // + panicHeight + unplannedForHeight;

	//*/


	///TODO check if the delta is realy necessary
	//step += computeSwingFootDelta(t);

	return step;
}

//void Swing_foot_controller::compute_inversed_kinematics(int parentJIndex, int childJIndex, const Vector3d &parentAxis,
//	const Vector3d &parentNormal, const Vector3d &childNormal,
//	const Vector3d &childEndEffector, const Point3d &wP,
//	bool computeAngVelocities, const Point3d &futureWP, double dt) {
//	//this is the joint between the grandparent RB and the parent
//	auto parentJoint = m_controlled_character->get_joint_by_id(parentJIndex);
//	//this is the grandparent - most calculations will be done in its coordinate frame
//	Articulated_rigid_body* gParent = parentJoint->get_parent_arb();
//	//this is the reduced character space where we will be setting the desired orientations and ang vels.
//	Reduced_character_state rs(Pose_controller::desired_pose);
//	//the desired relative orientation between parent and grandparent
//	Quaternion qParent;
//	//and the desired relative orientation between child and parent
//	Quaternion qChild;
//
//
//	Two_link_ik::getIKOrientations(parentJoint->get_joint_position_in_parent(), gParent->get_local_coordinates(wP), parentNormal, parentAxis,
//		childNormal, childEndEffector, &qParent, &qChild);
//
//
//
//	rs.set_joint_relative_orientation(qChild, childJIndex);
//	rs.set_joint_relative_orientation(qParent, parentJIndex);
//
//	{
//		Vector3d t = qChild.v;
//		if ((t.x != t.x) || (t.y != t.y) || (t.z != t.z)) {
//			std::cout << "Swing_foot_controller::compute_inversed_kinematics nan when compution orientation target";
//		}
//	}
//	{
//		Vector3d t = qParent.v;
//		if ((t.x != t.x) || (t.y != t.y) || (t.z != t.z)) {
//			std::cout << "Swing_foot_controller::compute_inversed_kinematics nan when compution orientation target";
//		}
//	}
//
//
//
//	Vector3d wParentD(0, 0, 0);
//	Vector3d wChildD(0, 0, 0);
//
//	if (computeAngVelocities) {
//		//the joint's origin will also move, so take that into account, by subbing the offset by which it moves to the
//		//futureTarget (to get the same relative position to the hip)
//		Vector3d velOffset = gParent->get_absolute_velocity_for_local_point(parentJoint->get_joint_position_in_parent());
//
//		Quaternion qParentF;
//		Quaternion qChildF;
//		Two_link_ik::getIKOrientations(parentJoint->get_joint_position_in_parent(),
//			gParent->get_local_coordinates(futureWP + velOffset * -dt), parentNormal,
//			parentAxis, childNormal, childEndEffector, &qParentF, &qChildF);
//
//
//		//the comment is the old calculation
//		//it is based on the fact that if hte angles are small sinn(theta)=~= theta
//		//but I call BS on that... it is way to dangerous when trying to haver bigger timesteps
//		Quaternion qDiff = qParentF * qParent.get_conjugate();
//		//wParentD = (qDiff.v / dt) *2;
//		if (qDiff.v.length() > 0) {
//			wParentD = (qDiff.v / (qDiff.v.length()))*(safeACOS(qDiff.s) * 2 / dt);
//		}
//		else {
//			wParentD = Vector3d(0, 0, 0);
//		}
//
//		//the above is the desired angular velocity, were the parent not rotating already - but it usually is, so we'll account for that
//		///TODO check if realy needed... I don't see why we should do that...
//		wParentD -= gParent->get_local_coordinates(gParent->get_angular_velocity_avg());
//
//
//		qDiff = qChildF * qChild.get_conjugate();
//
//		if (qDiff.v.length() > 0) {
//			wChildD = (qDiff.v / (qDiff.v.length()))*(safeACOS(qDiff.s) * 2 / dt);
//		}
//		else {
//			wChildD = Vector3d(0, 0, 0);
//		}
//		//        wChildD = (qDiff.v / dt) *2;
//
//		{
//			Vector3d t = wChildD;
//			if ((t.x != t.x) || (t.y != t.y) || (t.z != t.z)) {
//				std::cout << "Swing_foot_controller::compute_inverted_kinematics nan when computation velocity target";
//			}
//		}
//		{
//			Vector3d t = wParentD;
//			if ((t.x != t.x) || (t.y != t.y) || (t.z != t.z)) {
//				std::cout << "Swing_foot_controller::compute_inverted_kinematics nan when computation velocity target";
//			}
//		}
//
//		//make sure we don't go overboard with the estimates, in case there are discontinuities in the trajectories...
//		boundToRange(&wChildD.x, -5, 5); boundToRange(&wChildD.y, -5, 5); boundToRange(&wChildD.z, -5, 5);
//		boundToRange(&wParentD.x, -5, 5); boundToRange(&wParentD.y, -5, 5); boundToRange(&wParentD.z, -5, 5);
//
//	}
//
//
//	/*
//		Vector3d test=wChildD;
//		Vector3d test2=wParentD;
//		std::ostringstream oss;
//		oss<<"roottorque "<<std::fixed<<test.x<<" "<<test.y<<" "<<test.z<<" "<<test2.x<<" "<<test2.y<<" "<<test2.z;
//		std::cout<<oss.str();
////*/
//
//
//	rs.set_joint_relative_ang_velocity(wChildD, childJIndex);
//	rs.set_joint_relative_ang_velocity(wParentD, parentJIndex);
//}

void Swing_foot_controller::compute_ipm_alteration(Velocity_controller* vel_control)
{
	static int count = 0;
	count++;
	if (count < 10) return;


	if (vel_control == NULL) {
		return;
	}

	steps_left_sag--;
	if (steps_left_sag < 1) {
		evolve_sagittal = false;
		steps_left_sag = 0;
	};

	/*
	{
		std::ostringstream oss;
		oss<< "X";
		write_to_report_file(oss.str(),false);
	}//*/

	static Vector3d old_avg_buff = Vector3d(0, 0, 0);
	Vector3d old_avg_speed = old_avg_buff;
	old_avg_buff = SimGlobals::avg_speed;
	//we don't try to adapt the speed control if we are rotating (even if the system consider itself stable)
	//same reasoning if we are in a recovery step
	if (_is_recovery_step || vel_control->is_currently_rotating() ||
		vel_control->get_count_steps() < 3 /*|| SimGlobals::evolution_mode*/ || false) {
		//if (!SimGlobals::evolution_mode) {
		//	std::cout << "ipm alt not allowed" << std::endl;
		//}

		/*
		{
			std::ostringstream oss;
			oss<< "K ";
			write_to_report_file(oss.str(),false);
		}//*/
		return;
	}

	//first I'll deternime if I should adatp the IPM
	//I I detect that it should be adapted then i'll adapt it during the 3 next step before checking if I should continue
	//everytime I detect I should evolve the value the nbr of step left before stoping to evolve will be set to 3



	bool evo_sag = false;
	bool evo_cor = false;
	bool evo_sag_inf = false;
	bool evo_cor_inf = false;

	double limit_sup = 0.8;
	if (evolve_sagittal) {
		limit_sup = 0.5;
	}

	vel_control->virtual_force_near_limit(evo_cor, evo_sag, limit_sup);
	vel_control->virtual_force_near_limit(evo_cor_inf, evo_sag_inf, 0.1);


	//here i degrade the ipm alt if it opose the virtual force for two steps
	if (evo_sag_inf) {
		if (vel_control->last_virt_force_signed.z*ipm_alt_sagittal > 0 &&
			vel_control->previous_virt_force_signed.z*ipm_alt_sagittal > 0) {
			//if the ipm alteration is opposing the virt force can lower the IPM alteration without any worry
			//just as a reminder a negative ipm alt is equivalent to a forward force
			ipm_alt_sagittal -= ipm_alt_sagittal * 0.1;
			/*
			{
				std::ostringstream oss;
				oss<< "H";
				write_to_report_file(oss.str(),false);
			}//*/
		}
	}

	//only trigger here if it was not triggered before
	if (!evolve_sagittal&&evo_sag && (!_is_recovery_step)) {
		evolve_sagittal = true;
		steps_left_sag = 3;
	}

	//we check that the desired velocity is currently blocked before activating the IPM
	if (old_avg_speed.z*SimGlobals::avg_speed.z > 0) {
		if (SimGlobals::avg_speed.z < old_avg_speed.z - 0.1 || old_avg_speed.z + 0.1 < SimGlobals::avg_speed.z) {
			/*
			{
				std::ostringstream oss;
				oss<< "M ";
				write_to_report_file(oss.str(),false);
			}//*/
			return;
		}
	}

	//also we only activate the IPM alteration if the velocity tunning value has not changed
	bool has_evolved_sag = false;
	bool has_evolved_cor = false;
	vel_control->has_evolved_during_last_step(has_evolved_cor, has_evolved_sag, 0.1);
	if (has_evolved_sag) {
		/*
		{
			std::ostringstream oss;
			oss<< "L ";
			write_to_report_file(oss.str(),false);
		}//*/
		return;
	}

	//triggerhere in any case
	if (evo_sag && (!_is_recovery_step)) {
		evolve_sagittal = true;
		steps_left_sag = 3;
	}


	if (evolve_sagittal) {
		//this mean we have to evolve

		//Adapt the IPM alteration on sagittal results
		double eff_speed = vel_control->prev_step_sag_speed();
		try {
			double buff = vel_control->prev_step_sag_speed(1);
			eff_speed = eff_speed * 0.75 + buff * 0.25;
		}
		catch (...) {
			//an exception here mean we tryed to acces an incorrect step idx
			//so i just ignore it it doesn't matter
		}

		double d_v = velD_sagittal - eff_speed;

		if (std::abs(d_v) > 0.3) {
			d_v = 0.3*(d_v / std::abs(d_v));
		}
		ipm_alt_sagittal -= (d_v)*0.1;

		if (ipm_alt_sagittal > 0.045) {
			ipm_alt_sagittal = 0.045;
		}
		else if (ipm_alt_sagittal < -0.090) {
			ipm_alt_sagittal = -0.090;
		}

		/*
		std::ostringstream oss;
		oss<<"ipm_alteration:"<<ipm_alt_sagittal;
		std::cout<<oss.str();
		//*/

		//and now we have to ask the vel control to try to lower the virtual force
		//so that the main part of the velocity is compensated by the IPM
		vel_control->lower_force_intensity(0.3);

		/*
		{
			std::ostringstream oss;
			oss<< "O";
			write_to_report_file(oss.str(),false);
		}//*/
	}





	/*
	{
		std::ostringstream oss;
		oss<< " ";
		write_to_report_file(oss.str(),false);
	}//*/

	/*

	//Adapt IPM alteration on coronal results though the coronal ain't implemented yet
	double cor_top_limit = 0.03;
	double cor_top_inf = -0.03;

	static bool evolve_coronal_left = false;
	static bool evolve_coronal_right = false;
	static int steps_left_cor_left = 0;
	static int steps_left_cor_right = 0;


	if (evo_cor){
		if (character->stance() == RIGHT_STANCE){
			evolve_coronal_right = true;
			steps_left_cor_right = 3;
		}
		else{
			evolve_coronal_left = true;
			steps_left_cor_left = 3;
		}
	}

	d_v =virt_force_avg.x/1000;
	if (evolve_coronal_right && getController()->getStance() == RIGHT_STANCE){
		if (std::abs(d_v) > 0.3){
			d_v = 0.3*(d_v / std::abs(d_v));
		}
		ipm_alt_coronal_right -= (d_v)*0.01;

		if (ipm_alt_coronal_right > cor_top_limit){
			ipm_alt_coronal_right = cor_top_limit;
		}
		else if (ipm_alt_coronal_right < cor_top_inf){
			ipm_alt_coronal_right = cor_top_inf;
		}

		steps_left_cor_right--;
		if (steps_left_cor_right < 1){
			evolve_coronal_right = false;
		}
	}
	else if (evolve_coronal_left && getController()->getStance() == LEFT_STANCE){
		if (std::abs(d_v) > 0.3){
			d_v = 0.3*(d_v / std::abs(d_v));
		}
		ipm_alt_coronal_left -= (d_v)*0.1;

		if (ipm_alt_coronal_left > cor_top_limit){
			ipm_alt_coronal_left = cor_top_limit;
		}
		else if (ipm_alt_coronal_left < cor_top_inf){
			ipm_alt_coronal_left = cor_top_inf;
		}

		steps_left_cor_left--;
		if (steps_left_cor_left < 1){
			evolve_coronal_left = false;
		}
	}
	//*/
}

void Swing_foot_controller::pre_simulation_step_phase_1()
{
	preprocess_simulation_step();
	step_location = ipm_compute_step_location();
}

void Swing_foot_controller::pre_simulation_step_phase_2()
{

}

void Swing_foot_controller::pre_simulation_step_phase_3()
{
}

void Swing_foot_controller::post_simulation_step()
{
}

Controller_interface::type Swing_foot_controller::get_type()
{
	return Controller_interface::swing_foot_controller;
}
