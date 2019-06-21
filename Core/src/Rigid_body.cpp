#include "Rigid_body.h"
#include "RBUtils.h"
#include "CapsuleCDP.h"
#include "PlaneCDP.h"
#include "BoxCDP.h"
#include "SphereCDP.h"
#include "TriMeshCDP.h"
#include "Model.h"
#include <boost/algorithm/string.hpp>
#include <filesystem>
#include "Utils/src/Utils.h"
#include <fstream>

Rigid_body::Rigid_body(const Rigid_body& other)
{
	m_name = other.m_name;

	m_properties = other.m_properties;

	m_af_id = other.m_af_id;

	m_state = other.m_state;

	for (auto& model : other.m_models)
	{
		auto tmp_model = std::make_shared<Model>(*model);
		m_models.push_back(tmp_model);
	}

	for (auto& cdp : other.m_CDPs)
	{
		std::shared_ptr<CollisionDetectionPrimitive> tmp_cdp;
		switch(cdp->get_type())
		{
		case CollisionDetectionPrimitive::unknown_cdp: 
			break;
		case CollisionDetectionPrimitive::sphere_cdp:
			m_CDPs.push_back(std::make_shared<SphereCDP>(*std::dynamic_pointer_cast<SphereCDP>(cdp)));
			break;
		case CollisionDetectionPrimitive::capsule_cdp: 
			break;
		case CollisionDetectionPrimitive::plane_cdp:
			m_CDPs.push_back(std::make_shared<Plane_cdp>(*std::dynamic_pointer_cast<Plane_cdp>(cdp)));
			break;
		case CollisionDetectionPrimitive::box_cdp:
			m_CDPs.push_back(std::make_shared<BoxCDP>(*std::dynamic_pointer_cast<BoxCDP>(cdp)));
			break;
		case CollisionDetectionPrimitive::tri_mesh_cdp:
			m_CDPs.push_back(std::make_shared<Tri_mesh_cdp>(*std::dynamic_pointer_cast<Tri_mesh_cdp>(cdp)));
			break;
		default: 
			break;
		}
	}
	
	if (m_properties.mu < 0)
		throw std::logic_error("Incorrect rigid body input file - Friction coefficient should be >= 0");

	if (m_properties.epsilon < 0 || m_properties.epsilon > 1)
		throw std::logic_error("Incorrect rigid body input file - restitution coefficient should be between 0 and 1");

}

void Rigid_body::load_rb_from_struct(const A_RigidBody& a_rigid_body)
{
	m_name = a_rigid_body.name;

	set_mass(a_rigid_body.mass);

	set_moi(a_rigid_body.moi[0], a_rigid_body.moi[1], a_rigid_body.moi[2]);

	for (auto& mesh : a_rigid_body.meshs)
	{
		auto tmp_mesh = std::make_shared<Model>();
		tmp_mesh->load_from_file(mesh.file_name);
		tmp_mesh->scale_meshes(Vector3d(a_rigid_body.scale_factor[0], a_rigid_body.scale_factor[1],
			a_rigid_body.scale_factor[2]));
		tmp_mesh->translate_vertex(Vector3d(a_rigid_body.center_of_mass[0], a_rigid_body.center_of_mass[1],
			a_rigid_body.center_of_mass[2]));
		tmp_mesh->set_color(std::array<float, 3>({ float(mesh.color[0]), float(mesh.color[1]), float(mesh.color[2]) }));
		m_models.push_back(tmp_mesh);

		auto cdp_file_obj = boost::algorithm::replace_last_copy(mesh.file_name, ".obj", "_cdp.obj");
		auto cdp_file_txt = boost::algorithm::replace_last_copy(mesh.file_name, ".obj", "_cdp.txt");
		if (std::filesystem::exists(cdp_file_obj))
		{
			m_CDPs.push_back(std::make_shared<Tri_mesh_cdp>(cdp_file_obj, a_rigid_body));
		}
		else if (std::filesystem::exists(cdp_file_txt))
		{
			std::ifstream ifs;
			ifs.open(cdp_file_txt);

			// read to the end
			while (!ifs.eof())
			{
				std::string line;
				getline(ifs, line);
				std::vector<std::string> words;
				boost::algorithm::trim(line);
				if (!line.empty() && line.front() != '#')
				{
					split(words, line, boost::is_space(), boost::algorithm::token_compress_on);
					switch (m_attribute_id.find(words[0])->second)
					{
					case att_id::sphere:
						{
						Point3d center(std::stod(words[1]), std::stod(words[2]), std::stod(words[3]));
						double radius(std::stod(words[4]));	
						m_CDPs.push_back(std::make_shared<SphereCDP>(center, radius));
						break;
						}
					default:
						break;
					}
				}
			}
		}
		else
		{
			generate_cdp_from_meshes();
		}

	}

	m_properties.mu = a_rigid_body.frictionCoef;

	if (m_properties.mu < 0)
		throw std::logic_error("Incorrect rigid body input file - Friction coefficient should be >= 0");


	m_properties.epsilon = a_rigid_body.restitutionCoef;
	if (m_properties.epsilon < 0 || m_properties.epsilon > 1)
		throw std::logic_error("Incorrect rigid body input file - restitution coefficient should be between 0 and 1");
}


double Rigid_body::get_ground_softness() const
{
	return m_properties.ground_softness;
}

double Rigid_body::get_ground_penalty() const
{
	return m_properties.ground_penalty;
}

//This method returns the coordinates of the point that is passed in as a parameter(expressed in local coordinates), in world coordinates.
Point3d Rigid_body::get_point_world_coordinates(const Point3d& localPoint) const
{
	return this->m_state.position + get_vector_world_coordinates(Vector3d(localPoint));
}

//This method returns the vector that is passed in as a parameter(expressed in local coordinates), in world coordinates.
Vector3d Rigid_body::get_vector_world_coordinates(const Vector3d& localVector) const
{
	//the rigid body's arb_orientation is a unit quaternion. Using this, we can obtain the global coordinates of a local vector
	return m_state.orientation.rotate(localVector);
}

//This method is used to return the local coordinates of the point that is passed in as a parameter (expressed in global coordinates)
Point3d Rigid_body::get_local_coordinates(const Point3d& globalPoint) const
{
	const auto v = get_local_coordinates(Vector3d(globalPoint)) - get_local_coordinates(
		Vector3d(this->m_state.position));
	return Point3d(0, 0, 0) + v;
}

//This method is used to return the local coordinates of the vector that is passed in as a parameter (expressed in global coordinates)
Vector3d Rigid_body::get_local_coordinates(const Vector3d& globalVector) const
{
	//the rigid body's arb_orientation is a unit quaternion. Using this, we can obtain the global coordinates of a local vector
	return this->m_state.orientation.get_conjugate().rotate(globalVector);
}

//This method returns the absolute velocity of a point that is passed in as a parameter. The point is expressed in local coordinates, and the
//resulting velocity will be expressed in world coordinates.
Vector3d Rigid_body::get_absolute_velocity_for_local_point(const Point3d& localPoint) const
{
	//we need to compute the vector r, from the origin of the body to the point of interest
	Vector3d r(Point3d(), localPoint);
	//the velocity is given by omega x r + v. omega and v are already expressed in world coordinates, so we need to express r in world coordinates first.
	return m_state.angularVelocity.cross_product_with(get_vector_world_coordinates(r)) + m_state.velocity;
}

//This method returns the absolute velocity of a point that is passed in as a parameter. The point is expressed in global coordinates, and the
//resulting velocity will be expressed in world coordinates.
Vector3d Rigid_body::get_absolute_velocity_for_global_point(const Point3d& globalPoint) const
{
	//we need to compute the vector r, from the origin of the body to the point of interest
	Vector3d r(m_state.position, globalPoint);
	//the velocity is given by omega x r + v. omega and v are already expressed in world coordinates, so we need to express r in world coordinates first.
	return m_state.angularVelocity.cross_product_with(r) + m_state.velocity;
}


//Take min and max of every meshes in order to create a CD Box for the mesh, setDataFromPosition the resulting box the CDP array.
void Rigid_body::generate_cdp_from_meshes()
{
	for (auto& model : m_models)
	{
		Point3d p1, p2;
		std::vector<double> v_x, v_y, v_z;

		for (auto& mesh : model->meshes)
		{
			for (auto& vertex : mesh.vertices)
			{
				v_x.push_back(vertex.position[0]);
				v_y.push_back(vertex.position[1]);
				v_z.push_back(vertex.position[2]);
			}
		}

		auto result_x = std::minmax_element(v_x.begin(), v_x.end());
		auto result_y = std::minmax_element(v_y.begin(), v_y.end());
		auto result_z = std::minmax_element(v_z.begin(), v_z.end());

		p1.setValues(*result_x.first, *result_y.first, *result_z.first);
		p2.setValues(*result_x.second, *result_y.second, *result_z.second);

		Vector3d p1p2 = p2 - p1;

		p1 += p1p2 * (0.2);
		p2 += p1p2 * (-0.2);


		m_CDPs.push_back(std::make_shared<BoxCDP>(p1, p2));
	}
}


//This method loads all the pertinent information regarding the rigid body from a file.
void Rigid_body::load_from_file(FILE* f)
{
	if (f == nullptr)
		throw std::logic_error("Invalid file pointer.");

	//have a temporary buffer used to read the file line by line...
	char buffer[200];
	char mesh_path[200];

	//temporary variables that we may end up populating
	float r, g, b, a;
	Point3d p1, p2;
	Vector3d n;
	double radius, t1, t2, t3;
	double t;


	//this is where it happens.
	while (!feof(f))
	{
		//get a line from the file...
		fgets(buffer, 200, f);
		if (strlen(buffer) > 195)
			throw std::logic_error("The input file contains a line that is longer than ~200 characters - not allowed");
		char* line = lTrim(buffer);
		switch (getRBLineType(line))
		{
		case rb_utils::rb_name:
			m_name = trim(line);
			break;
		case rb_utils::rb_mesh_name:
		{
			sscanf(line, "%s", mesh_path);
			auto tmp_mesh = std::shared_ptr<Model>();
			tmp_mesh->load_from_file(mesh_path);
			m_models.push_back(tmp_mesh);
			break;
		}
		case rb_utils::rb_mass:
			if (sscanf_s(line, "%lf", &t) != 1)
				throw std::logic_error(
					"Incorrect rigid body input file - a mass needs to be specified if the 'mass' keyword is used.");
			set_mass(t);
			break;
		case rb_utils::rb_moi:
			if (sscanf_s(line, "%lf %lf %lf", &t1, &t2, &t3) != 3)
				throw std::logic_error(
					"Incorrect rigid body input file - the three principal moments of inertia need to be specified if the 'moi' keyword is used.");
			if (t1 <= 0 || t2 <= 0 || t3 <= 0)
				throw std::logic_error("Incorrect values for the principal moments of inertia.");
			set_moi(t1, t2, t3);
			break;
		case rb_utils::rb_end_rb:
			return;
		case rb_utils::rb_color:
			if (sscanf_s(line, "%f %f %f %f", &r, &g, &b, &a) != 4)
				throw std::logic_error(
					"Incorrect rigid body input file - color parameter expects 4 arguments " + std::string(line));
			if (!m_models.empty())
				m_models.back()->set_color(std::array<float, 3>({ r, g, b}));
			break;
		case rb_utils::rb_sphere:
			if (sscanf_s(line, "%lf %lf %lf %lf", &p1.x, &p1.y, &p1.z, &radius) != 4)
				throw std::logic_error(
					"Incorrect rigid body input file - 4 arguments are required to specify a sphere collision detection primitive\n");
			m_CDPs.push_back(std::make_shared<SphereCDP>(p1, r));
			break;
		case rb_utils::rb_capsule:
			if (sscanf_s(line, "%lf %lf %lf %lf %lf %lf %lf", &p1.x, &p1.y, &p1.z, &p2.x, &p2.y, &p2.z, &radius) != 7)
				throw std::logic_error(
					"Incorrect rigid body input file - 7 arguments are required to specify a capsule collision detection primitive\n");
			m_CDPs.push_back(std::make_shared<Capsule_cdp>(p1, p2, r));
			break;
		case rb_utils::rb_box:
			if (sscanf_s(line, "%lf %lf %lf %lf %lf %lf", &p1.x, &p1.y, &p1.z, &p2.x, &p2.y, &p2.z) != 6)
				throw std::logic_error(
					"Incorrect rigid body input file - 6 arguments are required to specify a box collision detection primitive\n");
			m_CDPs.push_back(std::make_shared<BoxCDP>(p1, p2));
			break;
		case rb_utils::rb_plane:
			if (sscanf_s(line, "%lf %lf %lf %lf %lf %lf", &n.x, &n.y, &n.z, &p1.x, &p1.y, &p1.z) != 6)
				throw std::logic_error(
					"Incorrect rigid body input file - 6 arguments are required to specify a plane collision detection primitive\n");
			m_CDPs.push_back(std::make_shared<Plane_cdp>(n, p1));
			m_state.position = { p1.x, p1.y, p1.z };
			break;
		case rb_utils::rb_not_important:
			break;
		case rb_utils::rb_locked:
			lock();
			break;
		case rb_utils::rb_position:
			if (sscanf_s(line, "%lf %lf %lf", &m_state.position.x, &m_state.position.y, &m_state.position.z) != 3)
				throw std::logic_error(
					"Incorrect rigid body input file - 3 arguments are required to specify the world coordinates position of a rigid body\n");
			break;
		case rb_utils::rb_orientation:
			if (sscanf_s(line, "%lf %lf %lf %lf", &t, &t1, &t2, &t3) != 4)
				throw std::logic_error(
					"Incorrect rigid body input file - 4 arguments are required to specify the world coordinates orientation of a rigid body\n");
			m_state.orientation = Quaternion::get_rotation_quaternion(t, Vector3d(t1, t2, t3).toUnit()) * m_state.
				orientation;
			break;
		case rb_utils::rb_velocity:
			if (sscanf_s(line, "%lf %lf %lf", &m_state.velocity.x, &m_state.velocity.y, &m_state.velocity.z) != 3)
				throw std::logic_error(
					"Incorrect rigid body input file - 3 arguments are required to specify the world coordinates velocity of a rigid body\n");
			break;
		case rb_utils::rb_angular_velocity:
			if (sscanf_s(line, "%lf %lf %lf", &m_state.angularVelocity.x, &m_state.angularVelocity.y,
				&m_state.angularVelocity.z) != 3)
				throw std::logic_error(
					"Incorrect rigid body input file - 3 arguments are required to specify the world coordinates angular velocity of a rigid body\n");
			break;
		case rb_utils::rb_friction_coeff:
			if (sscanf_s(line, "%lf", &m_properties.mu) != 1)
				throw std::logic_error(
					"Incorrect rigid body input file - Expecting a value for the friction coefficient");
			if (m_properties.mu < 0)
				throw std::logic_error("Incorrect rigid body input file - Friction coefficient should be >= 0");
			break;
		case rb_utils::rb_restitution_coeff:
			if (sscanf_s(line, "%lf", &m_properties.epsilon) != 1)
				throw std::logic_error(
					"Incorrect rigid body input file - Expecting a value for the restitution coefficient");
			if (m_properties.epsilon < 0 || m_properties.epsilon > 1)
				throw std::logic_error(
					"Incorrect rigid body input file - restitution coefficient should be between 0 and 1");
			break;
		case rb_utils::rb_ode_ground_coeff:
			if (sscanf_s(line, "%lf %lf", &t1, &t2) != 2)
				throw std::logic_error("Two parameters need to be provided for the ODE ground parameter settings");
			m_properties.ground_softness = t1;
			m_properties.ground_penalty = t2;
			break;
		case rb_utils::rb_planar:
			m_properties.planar = true;
			break;
		default:
			throw std::logic_error(
				"Incorrect rigid body input file in RB: " + std::string(buffer) + "- unexpected line.");
		}
	}
	throw std::logic_error("Incorrect articulated body input file! No /End found");
}

void Rigid_body::update_angular_velocity_avg()
{
	m_previous_angular_velocity_avg.push_back(m_angular_velocity_avg);

	if (m_previous_angular_velocity_avg.size() > m_previous_angular_vel_avg_max_size) {
		m_previous_angular_velocity_avg.erase(m_previous_angular_velocity_avg.begin());
	}

	Vector3d wRel_avg = Vector3d(0, 0, 0);
	for (int j = 0; j < m_previous_angular_velocity_avg.size(); ++j) {
		wRel_avg += m_previous_angular_velocity_avg[j];
	}
	wRel_avg = wRel_avg / double(m_previous_angular_velocity_avg.size());
	m_angular_velocity_avg = wRel_avg;
}
