#include <cmath>
#include "Ode_world.h"
#include "SimGlobals.h"

Ode_world::Ode_world()
{
	init();
}

Ode_world::Ode_world(const Ode_world& other)
{
	init();

	//rigid bodies
	Abstract_rb_engine::load_rbs_from_other(other);

	//af
	Abstract_rb_engine::load_af_from_other(other);
}

Ode_world::~Ode_world()
{
	//destroy the ODE physical world, simulation space and joint group
	delete m_cps;
	dJointGroupDestroy(contactGroupID);
	dSpaceDestroy(m_space_id);
	dWorldSetStepThreadingImplementation(m_world_id, nullptr, nullptr);
	dThreadingFreeImplementation(m_thread_id);
	dWorldDestroy(m_world_id);
	dCloseODE();
}

void Ode_world::load_rbs_from_file(char* f_name)
{
	Abstract_rb_engine::load_rbs_from_file(f_name);
}

void Ode_world::load_rbs_from_other(const Ode_world& other)
{
	load_rbs_from_other(other);
}

void Ode_world::load_af_from_struct(const Global& input)
{
	Abstract_rb_engine::load_af_from_struct(input);
}

void Ode_world::load_af_from_other(const Ode_world& other)
{
	Abstract_rb_engine::load_af_from_other(other);
}

void Ode_world::link_simbicon_with_ode()
{
	//now we'll make sure that the joint constraints are satisfied
	for (auto& rigid_body : m_rigid_bodies)
	{
		//CREATE AND LINK THE ODE BODY WITH OUR RIGID BODY

		//if the body is fixed, we'll only create the collision detection primitives
		if (rigid_body->is_locked())
		{
			//push in a dummy - never to be used - mapping!!!
			rigid_body->set_ode_id(dBodyID(nullptr));
		}
		else
		{
			rigid_body->set_ode_id(dBodyCreate(m_world_id));
		}

		create_ode_collision_primitives(*rigid_body);

		if (!rigid_body->is_locked())
		{
			dMass m;

			//set the mass and principal moments of inertia for this object
			m.setZero();
			const auto principal_moments = rigid_body->get_moi();
			m.setParameters(rigid_body->get_mass(), 0, 0, 0,
				principal_moments.x,
				principal_moments.y,
				principal_moments.z,
				0, 0, 0);

			set_engine_state_from_rb(*rigid_body);
		}
	}

	auto joints = m_character->get_joints();

	for (auto& joint : joints)
	{
		//connect the joint to the two bodies
		switch (joint->get_type())
		{
		case Joint::ball_in_socket_os:
			setup_ode_ball_and_socket_joint_opensim(*std::dynamic_pointer_cast<Custom_joint>(joint));
			break;
		case Joint::hinge_os:
			setup_ode_hinge_joint_opensim(*std::dynamic_pointer_cast<Custom_joint>(joint));
			break;
		case Joint::ball_in_socket:
			setup_ode_ball_and_socket_joint(*std::dynamic_pointer_cast<BallInSocketJoint>(joint));
			break;
		case Joint::hinge:
			setup_ode_hinge_joint(*std::dynamic_pointer_cast<HingeJoint>(joint));
			break;
		case Joint::universal:
			setup_ode_universal_joint(*std::dynamic_pointer_cast<UniversalJoint>(joint));
			break;
		case Joint::stiff_os:
			setup_ode_stiff_joint_opensim(*std::dynamic_pointer_cast<StiffJoint>(joint));
			break;
		default:
			throw std::logic_error("Ooops.... Only BallAndSocket, Hinge are currently supported.\n");
		}
	}
}

dGeomID Ode_world::get_plane_geom(Plane_cdp* p, Rigid_body& parent) const
{
	//and create the ground plane
	const auto n = p->get_normal();
	auto o = Vector3d(p->get_point_on_plane());
	const auto g = dCreatePlane(m_space_id, n.x, n.y, n.z, o.dotProductWith(n));
	return g;
}

dGeomID Ode_world::get_sphere_geom(SphereCDP* s) const
{
	const auto g = dCreateSphere(nullptr, s->get_radius());
	dSpaceAdd(m_space_id, g);
	const auto c = s->get_center();
	return g;
}

dGeomID Ode_world::get_box_geom(BoxCDP* b) const
{
	const auto g = dCreateBox(nullptr, b->getXLen(), b->getYLen(), b->getZLen());
	dSpaceAdd(m_space_id, g);
	return g;
}

dGeomID Ode_world::get_capsule_geom(Capsule_cdp* c) const
{
	const auto a = c->get_a();
	const auto b = c->get_b();
	Vector3d ab(a, b);
	const auto g = dCreateCCylinder(nullptr, c->get_radius(), ab.length());
	dSpaceAdd(m_space_id, g);
	const auto cen = a + ab / 2.0;
	dGeomSetPosition(g, cen.x, cen.y, cen.z);


	//now, the default arb_orientation for this is along the z-axis. We need to rotate this to make it match the direction
	//of ab, so we need an angle and an axis...
	Vector3d def_a(0, 0, 1);

	auto axis = def_a.cross_product_with(ab);
	axis.toUnit();
	const auto rot_angle = def_a.angle_with(ab);

	const Quaternion rel_orientation = Quaternion::get_rotation_quaternion(rot_angle, axis);

	dQuaternion q;
	q[0] = rel_orientation.s;
	q[1] = rel_orientation.v.x;
	q[2] = rel_orientation.v.y;
	q[3] = rel_orientation.v.z;

	dGeomSetQuaternion(g, q);

	return g;
}

dGeomID Ode_world::get_tri_mesh_geom(Tri_mesh_cdp* mesh) const
{
	const auto data = dGeomTriMeshDataCreate();

	dGeomTriMeshDataBuildSingle(data, mesh->vertices.data(), 3 * sizeof(float), int(mesh->vertices.size() / 3), mesh->indices.data(), int(mesh->indices.size()), 3 * sizeof(dTriIndex));

	const auto g = dCreateTriMesh(m_space_id, data, nullptr, nullptr, nullptr);

	return g;
}


void Ode_world::setup_ode_hinge_joint(HingeJoint& hj)
{
	const auto j = dJointCreateHinge(m_world_id, nullptr);
	dJointSetFeedback(j, &(m_joints_feedback[hj.get_af_id()]));

	ode_to_joints.emplace_back(j, hj);
	dJointSetData(j, reinterpret_cast<void*>(hj.get_af_id()));
	hj.id_ode_world = j;
	dJointAttach(j, hj.get_child_arb()->get_ode_id(), hj.get_parent_arb()->get_ode_id());
	const auto p = hj.get_child_arb()->get_point_world_coordinates(hj.get_joint_position_in_child());
	dJointSetHingeAnchor(j, p.x, p.y, p.z);
	const auto a = hj.get_parent_arb()->get_vector_world_coordinates(hj.get_rotation_axes_in_local_coords()[0]);
	dJointSetHingeAxis(j, a.x, a.y, a.z);
	//now set the joint limits
	if (!hj.is_using_joint_limits())
		return;

	dJointSetHingeParam(j, dParamLoStop, hj.get_rotation_limits()[0][0]);
	dJointSetHingeParam(j, dParamHiStop, hj.get_rotation_limits()[0][1]);
}

void Ode_world::setup_ode_hinge_joint_opensim(Custom_joint& cj)
{
	dJointID j = dJointCreateHinge(m_world_id, nullptr);
	dJointSetFeedback(j, &(m_joints_feedback[cj.get_af_id()]));

	ode_to_joints.emplace_back(j, cj);
	dJointSetData(j, reinterpret_cast<void*>(cj.get_af_id()));
	cj.id_ode_world = j;
	dJointAttach(j, cj.get_child_arb()->get_ode_id(), cj.get_parent_arb()->get_ode_id());
	const Point3d p = cj.get_parent_arb()->get_point_world_coordinates(cj.get_joint_position_in_parent());
	dJointSetHingeAnchor(j, p.x, p.y, p.z);
	const Vector3d a = cj.get_parent_arb()->get_vector_world_coordinates(cj.get_rotation_axes_in_local_coords()[0]);
	dJointSetHingeAxis(j, a.x, a.y, a.z);

	//now set the joint limits
	if (!cj.is_using_joint_limits())
		return;

	dJointSetHingeParam(j, dParamLoStop, cj.get_rotation_limits()[0][0]);
	dJointSetHingeParam(j, dParamHiStop, cj.get_rotation_limits()[0][1]);
}

void Ode_world::setup_ode_universal_joint(UniversalJoint& uj)
{
	dJointID j = dJointCreateUniversal(m_world_id, nullptr);
	dJointSetFeedback(j, &(m_joints_feedback[uj.get_af_id()]));

	ode_to_joints.emplace_back(j, uj);
	dJointSetData(j, reinterpret_cast<void*>(uj.get_af_id()));
	uj.id_ode_world = j;

	dJointAttach(j, uj.get_child_arb()->get_ode_id(), uj.get_parent_arb()->get_ode_id());
	const Point3d p = uj.get_child_arb()->get_point_world_coordinates(uj.get_joint_position_in_child());
	dJointSetUniversalAnchor(j, p.x, p.y, p.z);

	const Vector3d a = uj.get_parent_arb()->get_vector_world_coordinates(uj.get_rotation_axes_in_local_coords()[0]);
	const Vector3d b = uj.get_child_arb()->get_vector_world_coordinates(uj.get_rotation_axes_in_local_coords()[1]);

	dJointSetUniversalAxis1(j, a.x, a.y, a.z);
	dJointSetUniversalAxis2(j, b.x, b.y, b.z);

	//now set the joint limits
	if (!uj.is_using_joint_limits())
		return;

	dJointSetUniversalParam(j, dParamLoStop, uj.get_rotation_limits()[0][0]);
	dJointSetUniversalParam(j, dParamHiStop, uj.get_rotation_limits()[0][1]);
	dJointSetUniversalParam(j, dParamLoStop2, uj.get_rotation_limits()[1][0]);
	dJointSetUniversalParam(j, dParamHiStop2, uj.get_rotation_limits()[1][1]);
}

void Ode_world::setup_ode_ball_and_socket_joint(BallInSocketJoint& basj)
{
	dJointID j = dJointCreateBall(m_world_id, nullptr);
	ode_to_joints.emplace_back(j, basj);
	dJointSetData(j, reinterpret_cast<void*>(basj.get_af_id()));
	dJointAttach(j, basj.get_child_arb()->get_ode_id(), basj.get_parent_arb()->get_ode_id());
	const Point3d p = basj.get_child_arb()->get_point_world_coordinates(basj.get_joint_position_in_child());
	//now we'll set the world position of the ball-and-socket joint. It is important that the bodies are placed in the world
	//properly at this point
	dJointSetBallAnchor(j, p.x, p.y, p.z);

	//now deal with the joint limits
	if (!basj.is_using_joint_limits())
		return;

	const Vector3d a = basj.get_parent_arb()->get_vector_world_coordinates(basj.get_rotation_axes_in_local_coords()[0]);
	const Vector3d b = basj.get_child_arb()->get_vector_world_coordinates(basj.get_rotation_axes_in_local_coords()[1]);

	//we'll assume that:
	//b is the twisting axis of the joint, and the joint limits will be (in magnitude) less than 90 degrees, otherwise
	//the simulation will go unstable!!!


	const auto a_motor = dJointCreateAMotor(m_world_id, nullptr);
	ode_to_joints.emplace_back(a_motor, basj);
	dJointSetData(a_motor, reinterpret_cast<void*>(basj.get_af_id()));
	basj.id_ode_world = a_motor;
	dJointAttach(a_motor, basj.get_parent_arb()->get_ode_id(), basj.get_child_arb()->get_ode_id());
	dJointSetAMotorMode(a_motor, dAMotorEuler);

	dJointSetAMotorParam(a_motor, dParamStopCFM, 0.1);
	dJointSetAMotorParam(a_motor, dParamStopCFM2, 0.1);
	dJointSetAMotorParam(a_motor, dParamStopCFM3, 0.1);


	dJointSetAMotorAxis(a_motor, 0, 1, a.x, a.y, a.z);
	dJointSetAMotorAxis(a_motor, 2, 2, b.x, b.y, b.z);
	dJointSetAMotorParam(a_motor, dParamLoStop, basj.get_rotation_limits()[0][0]);
	dJointSetAMotorParam(a_motor, dParamHiStop, basj.get_rotation_limits()[0][1]);

	dJointSetAMotorParam(a_motor, dParamLoStop2, basj.get_rotation_limits()[1][0]);
	dJointSetAMotorParam(a_motor, dParamHiStop2, basj.get_rotation_limits()[1][1]);

	dJointSetAMotorParam(a_motor, dParamLoStop3, basj.get_rotation_limits()[2][0]);
	dJointSetAMotorParam(a_motor, dParamHiStop3, basj.get_rotation_limits()[2][1]);
}

void Ode_world::setup_ode_ball_and_socket_joint_opensim(Custom_joint& cj)
{
	dJointID j = dJointCreateBall(m_world_id, nullptr);
	dJointSetFeedback(j, &(m_joints_feedback[cj.get_af_id()]));

	ode_to_joints.emplace_back(j, cj);
	dJointSetData(j, reinterpret_cast<void*>(cj.get_af_id()));
	dJointAttach(j, cj.get_child_arb()->get_ode_id(), cj.get_parent_arb()->get_ode_id());
	const Point3d p = cj.get_parent_arb()->get_point_world_coordinates(cj.get_joint_position_in_parent());
	//now we'll set the world position of the ball-and-socket joint. It is important that the bodies are placed in the world
	//properly at this point
	dJointSetBallAnchor(j, p.x, p.y, p.z);

	//now deal with the joint limits
	if (!cj.is_using_joint_limits())
		return;

	auto rotation_axis = cj.get_rotation_axes_in_local_coords();

	const auto a = cj.get_parent_arb()->get_vector_world_coordinates(rotation_axis[0]);
	const auto b = cj.get_child_arb()->get_vector_world_coordinates(rotation_axis[2]);


	//watch Angular Motor in ODE doc for more details
	const auto a_motor = dJointCreateAMotor(m_world_id, nullptr);
	ode_to_joints.emplace_back(a_motor, cj);
	dJointSetData(a_motor, reinterpret_cast<void*>(cj.get_af_id()));
	cj.id_ode_world = a_motor;
	dJointAttach(a_motor, cj.get_parent_arb()->get_ode_id(), cj.get_child_arb()->get_ode_id());
	dJointSetAMotorMode(a_motor, dAMotorEuler);

	dJointSetAMotorParam(a_motor, dParamStopCFM, 0.1);
	dJointSetAMotorParam(a_motor, dParamStopCFM2, 0.1);
	dJointSetAMotorParam(a_motor, dParamStopCFM3, 0.1);


	dJointSetAMotorAxis(a_motor, 0, 1, a.x, a.y, a.z);
	dJointSetAMotorAxis(a_motor, 2, 2, b.x, b.y, b.z);

	dJointSetAMotorParam(a_motor, dParamLoStop, cj.get_rotation_limits()[0][0]);
	dJointSetAMotorParam(a_motor, dParamHiStop, cj.get_rotation_limits()[0][1]);

	dJointSetAMotorParam(a_motor, dParamLoStop2, cj.get_rotation_limits()[1][0]);
	dJointSetAMotorParam(a_motor, dParamHiStop2, cj.get_rotation_limits()[1][1]);

	dJointSetAMotorParam(a_motor, dParamLoStop3, cj.get_rotation_limits()[2][0]);
	dJointSetAMotorParam(a_motor, dParamHiStop3, cj.get_rotation_limits()[2][1]);
}

void Ode_world::setup_ode_stiff_joint(StiffJoint& sj)
{
	dJointID j = dJointCreateFixed(m_world_id, nullptr);
	dJointSetFeedback(j, &(m_joints_feedback[sj.get_af_id()]));

	ode_to_joints.emplace_back(j, sj);
	dJointSetData(j, reinterpret_cast<void*>(sj.m_af_id));
	dJointAttach(j, sj.m_child_arb->get_ode_id(), sj.m_parent_arb->get_ode_id());
	dJointSetFixed(j);
}

void Ode_world::setup_ode_stiff_joint_opensim(StiffJoint& sj)
{
	dJointID j = dJointCreateFixed(m_world_id, nullptr);
	dJointSetFeedback(j, &(m_joints_feedback[sj.get_af_id()]));

	ode_to_joints.emplace_back(j, sj);
	dJointSetData(j, reinterpret_cast<void*>(sj.m_af_id));
	dJointAttach(j, sj.m_child_arb->get_ode_id(), sj.m_parent_arb->get_ode_id());
	dJointSetFixed(j);
}

void Ode_world::create_ode_collision_primitives(Rigid_body& body) const
{
	//now we'll set up the body's collision detection primitives
	for (uint j = 0; j < body.get_cdps().size(); j++)
	{
		//depending on the type of collision primitive, we'll now create g.
		dGeomID g;
		switch (body.get_cdps()[j]->get_type())
		{
		case CollisionDetectionPrimitive::sphere_cdp:
		{
			auto s = dynamic_cast<SphereCDP*>(body.get_cdps()[j].get());
			g = get_sphere_geom(s);
			dGeomSetBody(g, body.get_ode_id());
			const auto c = s->get_center();
			dGeomSetOffsetPosition(g, c.x, c.y, c.z);
			break;
		}
		case CollisionDetectionPrimitive::capsule_cdp:
			g = get_capsule_geom(dynamic_cast<Capsule_cdp*>(body.get_cdps()[j].get()));
			dGeomSetBody(g, body.get_ode_id());
			break;
		case CollisionDetectionPrimitive::box_cdp:
		{
			auto b = dynamic_cast<BoxCDP*>(body.get_cdps()[j].get());
			g = get_box_geom(b);
			dGeomSetBody(g, body.get_ode_id());
			const auto c = b->getCenter();
			dGeomSetOffsetPosition(g, c.x, c.y, c.z);
			break;
		}
		case CollisionDetectionPrimitive::plane_cdp:
			//NOTE: only static objects can have planes as their collision primitives - if this isn't static, force it!!
			g = get_plane_geom(dynamic_cast<Plane_cdp*>(body.get_cdps()[j].get()), body);
			dGeomSetBody(g, body.get_ode_id());
			break;
		case CollisionDetectionPrimitive::tri_mesh_cdp:
			g = get_tri_mesh_geom(dynamic_cast<Tri_mesh_cdp*>(body.get_cdps()[j].get()));
			dGeomSetBody(g, body.get_ode_id());
			break;
		case CollisionDetectionPrimitive::unknown_cdp:
		default:
			throw std::logic_error(
				"Ooppps... No collision detection primitive was created for: " + body.get_name() + " cdp: " + std::
				to_string(j));
		}

		//now associate the geom to the rigid body that it belongs to, so that we can look up the properties we need later...
		dGeomSetData(g, &body);
	}
}

void Ode_world::process_collisions(dGeomID o1, dGeomID o2)
{
	const auto b1 = dGeomGetBody(o1);
	const auto b2 = dGeomGetBody(o2);
	const auto rb1 = reinterpret_cast<Rigid_body*>(dGeomGetData(o1));
	const auto rb2 = reinterpret_cast<Rigid_body*>(dGeomGetData(o2));

	auto joined = b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact);
	if (rb1->get_name() == "calcn_l" && rb2->get_name() == "tibia_l")
		joined = true;
	if (rb1->get_name() == "calcn_r" && rb2->get_name() == "tibia_r")
		joined = true;

	if (pcQuery)
	{
		if (!pcQuery->shouldCheckForCollisions(*rb1, *rb2, joined))
			return;
	}

	//we'll use the minimum of the two coefficients of friction of the two bodies.
	const auto mu1 = rb1->get_friction_coefficient();
	const auto mu2 = rb2->get_friction_coefficient();
	const auto mu_to_use = std::min(mu1, mu2);
	//const auto eps1 = rb1->get_restitution_coefficient();
	//const auto eps2 = rb2->get_restitution_coefficient();
	//const auto eps_to_use = std::min(eps1, eps2);
	const auto groundSoftness1 = rb1->get_ground_softness();
	const auto groundSoftness2 = rb2->get_ground_softness();
	const auto groundSoftness = std::min(groundSoftness1, groundSoftness2);
	const auto groundPenalty1 = rb1->get_ground_penalty();
	const auto groundPenalty2 = rb2->get_ground_penalty();
	const auto groundPenalty = std::min(groundPenalty1, groundPenalty2);
	//std::cout << rb1->get_cm_velocity().projectionOn(SimGlobals::up).length() << std::endl;


	const auto nb_contacts = dCollide(o1, o2, m_max_contact_count, &(m_cps[0].geom), sizeof(dContact));

	if (nb_contacts)
	{

		//std::cout << rb1->get_name() << " " << rb1->get_cm_velocity() << std::endl;
		//std::cout << rb2->get_name() << " " << rb2->get_cm_velocity() << std::endl;

		//const auto f = 100;
		//const auto mass_eff = (9.81 * 70 * SimGlobals::dt) / abs((rb1->get_cm_velocity() - rb2->get_cm_velocity()).dotProductWith(SimGlobals::up));
		//const auto c = 2 * mass_eff * 100 * f;
		//const auto k = pow(f, 2) * mass_eff;
		//const auto dtk = (SimGlobals::dt*k);

		//const auto erp = 1 / (c + dtk);
		//const auto cfm = dtk / (c + dtk);

		//std::cout << "m_eff " << mass_eff << std::endl;
		//std::cout << "k " << k << std::endl;
		//std::cout << "c " << c << std::endl;
		//std::cout << "dtk " << dtk << std::endl;
		//std::cout << "erp " << erp << std::endl;
		//std::cout << "cfm " << cfm << std::endl;



		//fill in the missing properties for the contact points
		for (auto i = 0; i < nb_contacts; i++)
		{
			//std::cout << rb1->get_name() << " with " << rb2->get_name() << " : " << m_cps[i].geom.arb_position[0] << " " << m_cps[i].geom.arb_position[1] << " " << m_cps[i].geom.arb_position[2] << std::endl;


			m_cps[i].surface.mode = dContactSoftERP | dContactSoftCFM | dContactRolling | dContactApprox1;
			m_cps[i].surface.mu = 1e1/*mu_to_use*/;
			m_cps[i].surface.rho = 0;
			m_cps[i].surface.rho2 = 0;
			m_cps[i].surface.rhoN = 0;
			//m_cps[i].surface.bounce = eps_to_use;
			//m_cps[i].surface.bounce_vel = 1e-5;

			m_cps[i].surface.soft_cfm = 9e-3;
			m_cps[i].surface.soft_erp = 0.2;
		}
	}

	// filter to list of m_cps in order to avoid double or triple definition of the some cp 
	// and now set the contact points to the simulation
	for (auto i = 0; i < nb_contacts; i++)
	{
		bool already_in = false;
		for (size_t j = 0; j < m_contact_joint_count; j++)
		{
			const auto err = 0.0001;

			const auto dist = Vector3d(m_cps[i].geom.pos[0] - m_contact_points[j]->cp.x, m_cps[i].geom.pos[1] - m_contact_points[j]->cp.y, m_cps[i].geom.pos[2] - m_contact_points[j]->cp.z).length();

			// here we check if there is already a contact point at short range
			if (dist < err)
			{
				already_in = true;
			}
		}

		if (!already_in)
		{
			//create a joint, and link the two geometries.
			const auto c = dJointCreateContact(m_world_id, contactGroupID, &m_cps[i]);
			dJointAttach(c, b1, b2);

			m_contact_points.push_back(std::make_shared<Contact_point>());
			//now we'll set up the feedback for this contact joint
			m_contact_points[m_contact_joint_count]->rb1 = rb1;
			m_contact_points[m_contact_joint_count]->rb2 = rb2;
			m_contact_points[m_contact_joint_count]->cp = Point3d(m_cps[i].geom.pos[0], m_cps[i].geom.pos[1],
				m_cps[i].geom.pos[2]);
			dJointSetFeedback(c, &(m_contact_joints_feedback[m_contact_joint_count]));


			if (m_contact_points[m_contact_joint_count]->rb1->is_articulated() || m_contact_points[m_contact_joint_count]->rb2->is_articulated())
			{
				m_character->add_contact_points(*m_contact_points[m_contact_joint_count]);
			}

			m_contact_joint_count++;
		}
	}
}

void collisionCallBack(void* odeWorld, dGeomID o1, dGeomID o2)
{
	static_cast<Ode_world*>(odeWorld)->process_collisions(o1, o2);
}

void Ode_world::engine_simulation_step()
{
	const auto delta_t = SimGlobals::dt;

	//make sure that the state of the RB's is synchronized with the engine...
	set_engine_state_from_rb();

	//restart the counter for the joint feedback terms
	m_character->clear_contact_points();
	m_contact_joint_count = 0;

	//go through all the joints in the world, and apply their torques to the parent and child rb's
	for (auto& a_joint : m_character->get_joints())
	{
		Vector3d torque_to_apply;
		if (a_joint->is_muscle_actuated() && SimGlobals::use_muscle_actuation)
		{
			torque_to_apply = a_joint->get_muscle_torque();
			//reset torques when used
			a_joint->set_muscle_torque(Vector3d());
			a_joint->set_torque(Vector3d());
		}
		else
		{
			torque_to_apply = a_joint->get_torque();
			//reset torques when used
			a_joint->set_muscle_torque(Vector3d());
			a_joint->set_torque(Vector3d());
		}


		//we will apply to the parent a positive torque, and to the child a negative torque
		dBodyAddTorque(a_joint->get_parent_arb()->get_ode_id(), torque_to_apply.x,
			torque_to_apply.y, torque_to_apply.z);
		dBodyAddTorque(a_joint->get_child_arb()->get_ode_id(), -torque_to_apply.x,
			-torque_to_apply.y, -torque_to_apply.z);
	}

	//we need to determine the contact points first - delete the previous contacts
	dJointGroupEmpty(contactGroupID);

	//initiate the collision detection
	dSpaceCollide(m_space_id, this, &collisionCallBack);

	//advance the simulation
	dWorldStep(m_world_id, delta_t);

	m_character->save_previous_state();

	//copy over the state of the ODE bodies to the rigid bodies...
	set_rb_state_from_engine();

	for (size_t i = 0; i < m_contact_joint_count; i++)
	{
		m_contact_points[i]->f = Vector3d(m_contact_joints_feedback[i].f1[0], m_contact_joints_feedback[i].f1[1],
			m_contact_joints_feedback[i].f1[2]);
		m_contact_points[i]->t1 = Vector3d(m_contact_joints_feedback[i].t1[0], m_contact_joints_feedback[i].t1[1],
			m_contact_joints_feedback[i].t1[2]);
	}
}

void Ode_world::sync_ode_and_state()
{
	set_rb_state_from_engine();
	set_engine_state_from_rb();
}

void Ode_world::set_rb_state_from_engine()
{
	//now update all the rigid bodies...
	for (auto& rigid_body : m_rigid_bodies)
	{
		set_rb_state_from_ode(*rigid_body);
	}
}

void Ode_world::set_rb_state_from_ode(Rigid_body& rigid_body)
{
	//if it is a locked object, we won't do anything about it
	if (rigid_body.is_locked())
		return;

	const dReal* tempData = dBodyGetPosition(rigid_body.get_ode_id());
	rigid_body.set_cm_position(Point3d(tempData[0], tempData[1], tempData[2]));


	tempData = dBodyGetQuaternion(rigid_body.get_ode_id());
	rigid_body.set_orientation(Quaternion(tempData[0], tempData[1], tempData[2], tempData[3]));

	tempData = dBodyGetLinearVel(rigid_body.get_ode_id());
	rigid_body.set_cm_velocity(Vector3d(tempData[0], tempData[1], tempData[2]));

	tempData = dBodyGetAngularVel(rigid_body.get_ode_id());
	rigid_body.set_angular_velocity(Vector3d(tempData[0], tempData[1], tempData[2]));
}

void Ode_world::set_engine_state_from_rb()
{
	//now update all the rigid bodies...
	for (auto& rigid_body : m_rigid_bodies)
	{
		set_engine_state_from_rb(*rigid_body);
	}
}

void Ode_world::set_engine_state_from_rb(Rigid_body& rigid_body)
{
	if (rigid_body.is_locked())
		return;

	dBodySetPosition(rigid_body.get_ode_id(), rigid_body.get_cm_position().x, rigid_body.get_cm_position().y, rigid_body.get_cm_position().z);

	dQuaternion tempQ;
	tempQ[0] = rigid_body.get_orientation().s;
	tempQ[1] = rigid_body.get_orientation().v.x;
	tempQ[2] = rigid_body.get_orientation().v.y;
	tempQ[3] = rigid_body.get_orientation().v.z;
	dBodySetQuaternion(rigid_body.get_ode_id(), tempQ);

	dBodySetLinearVel(rigid_body.get_ode_id(), rigid_body.get_cm_velocity().x,
		rigid_body.get_cm_velocity().y, rigid_body.get_cm_velocity().z);

	dBodySetAngularVel(rigid_body.get_ode_id(), rigid_body.get_angular_velocity().x,
		rigid_body.get_angular_velocity().y, rigid_body.get_angular_velocity().z);
}

void Ode_world::set_state(const std::vector<double>& state, int start)
{
	Abstract_rb_engine::set_state(state, start);
}

void Ode_world::reset_engine_state()
{
	for (auto& rigid_body : m_rigid_bodies)
	{
		reset_ode_state(*rigid_body);
	}
}

void Ode_world::reset_ode_state(Rigid_body& rigid_body)
{
	if (rigid_body.is_locked())
		return;

	dBodySetPosition(rigid_body.get_ode_id(), 0, 0, 0);

	dQuaternion temp_quaternion;
	temp_quaternion[0] = 1;
	temp_quaternion[1] = 0;
	temp_quaternion[2] = 0;
	temp_quaternion[3] = 0;
	dBodySetQuaternion(rigid_body.get_ode_id(), temp_quaternion);

	dBodySetLinearVel(rigid_body.get_ode_id(), 0, 0, 0);

	dBodySetAngularVel(rigid_body.get_ode_id(), 0, 0, 0);

	dBodySetForce(rigid_body.get_ode_id(), 0, 0, 0);
	dBodySetTorque(rigid_body.get_ode_id(), 0, 0, 0);
}

void Ode_world::apply_rel_force_to(Rigid_body& rigid_body, const Vector3d& f, const Point3d& p)
{
	dBodyAddRelForceAtRelPos(rigid_body.get_ode_id(), f.x, f.y, f.z, p.x, p.y, p.z);
}

void Ode_world::apply_force_to(Rigid_body& rigid_body, const Vector3d& f, const Point3d& p)
{
	dBodyAddForceAtRelPos(rigid_body.get_ode_id(), f.x, f.y, f.z, p.x, p.y, p.z);
}

void Ode_world::apply_torque_to(Rigid_body& rigid_body, const Vector3d& t)
{
	dBodyAddTorque(rigid_body.get_ode_id(), t.x, t.y, t.z);
}

void Ode_world::init()
{
	const auto max_cont = 333;
	//Initialize the world, simulation space and joint groups
	dInitODE();
	m_world_id = dWorldCreate();
	m_thread_id = dThreadingAllocateSelfThreadedImplementation();
	dWorldSetStepThreadingImplementation(m_world_id, dThreadingImplementationGetFunctions(m_thread_id), m_thread_id);
	m_space_id = dHashSpaceCreate(nullptr);
	contactGroupID = dJointGroupCreate(0);

	dSetErrorHandler(my_error_handler);
	dSetMessageHandler(my_message_handler);
	dSetDebugHandler(my_error_handler);

	//set a few of the constants that ODE needs to be aware of
	//dWorldSetContactSurfaceLayer(m_world_id, 0.001); // the amount of interpenetration allowed between objects
	//dWorldSetContactMaxCorrectingVel(m_world_id, 1.0); // maximum velocity that contacts are allowed to generate  
	dWorldSetERP(m_world_id, 0.2);
	dWorldSetCFM(m_world_id, 9e-3);

	//set the gravity...
	const auto gravity = SimGlobals::up * SimGlobals::gravity;
	dWorldSetGravity(m_world_id, gravity.x, gravity.y, gravity.z);

	//allocate the space for the contacts;
	m_max_contact_count = max_cont;
	m_cps = new dContact[m_max_contact_count];

	pcQuery = nullptr;

	pcQuery = std::make_shared<PreCollisionQuery>();
}