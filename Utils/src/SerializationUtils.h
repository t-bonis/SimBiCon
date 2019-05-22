#pragma once

#include "boost/archive/xml_oarchive.hpp"
#include "boost/archive/xml_iarchive.hpp"
#include "boost/serialization/vector.hpp"
#include <vector>


struct viaPoint
{
	std::string name;
	std::vector<double> position;
	std::string refBody;

	template <class Archive>
	void serialize(Archive& archive, const unsigned int version)
	{
		archive & BOOST_SERIALIZATION_NVP(name);
		archive & BOOST_SERIALIZATION_NVP(position);
		archive & BOOST_SERIALIZATION_NVP(refBody);
	}
};

struct Muscle_from_xml
{
	std::string name;
	double maxForce{};
	double optimalLength{};
	double slackLength{};
	std::vector<viaPoint> viaPoints;

	template <class Archive>
	void serialize(Archive& archive, const unsigned int version)
	{
		archive & BOOST_SERIALIZATION_NVP(name);
		archive & BOOST_SERIALIZATION_NVP(maxForce);
		archive & BOOST_SERIALIZATION_NVP(optimalLength);
		archive & BOOST_SERIALIZATION_NVP(slackLength);
		archive & BOOST_SERIALIZATION_NVP(viaPoints);
	}
};

struct Coordinate_OpenSim
{
	std::string name;
	std::vector<double> range;

	template <class Archive>
	void serialize(Archive& archive, const unsigned int version)
	{
		archive & BOOST_SERIALIZATION_NVP(name);
		archive & BOOST_SERIALIZATION_NVP(range);
	}
};

struct TransformAxis_from_xml
{
	std::string motion_type;
	std::string coordinate_name;
	std::string function;
	std::vector<double> axis;
	std::vector<double> spline_x;
	std::vector<double> spline_y;
	double value{};
	std::vector<double> coef;

	template <class Archive>
	void serialize(Archive& archive, const unsigned int version)
	{
		archive & BOOST_SERIALIZATION_NVP(motion_type);
		archive & BOOST_SERIALIZATION_NVP(coordinate_name);
		archive & BOOST_SERIALIZATION_NVP(function);
		archive & BOOST_SERIALIZATION_NVP(axis);
		archive & BOOST_SERIALIZATION_NVP(spline_x);
		archive & BOOST_SERIALIZATION_NVP(spline_y);
		archive & BOOST_SERIALIZATION_NVP(value);
		archive & BOOST_SERIALIZATION_NVP(coef);
	}
};


struct Joint_from_xml
{
	std::vector<Coordinate_OpenSim> coordinates;
	std::vector<TransformAxis_from_xml> transAxis;
	int type{};
	std::string name;
	std::string parent_body_name;
	std::string child_body_name;
	std::vector<double> parent_RB_Pos;
	std::vector<double> child_RB_pos;

	template <class Archive>
	void serialize(Archive& archive, const unsigned int version)
	{
		archive & BOOST_SERIALIZATION_NVP(coordinates);
		archive & BOOST_SERIALIZATION_NVP(transAxis);
		archive & BOOST_SERIALIZATION_NVP(type);
		archive & BOOST_SERIALIZATION_NVP(name);
		archive & BOOST_SERIALIZATION_NVP(parent_body_name);
		archive & BOOST_SERIALIZATION_NVP(child_body_name);
		archive & BOOST_SERIALIZATION_NVP(parent_RB_Pos);
		archive & BOOST_SERIALIZATION_NVP(child_RB_pos);
	}
};

struct Mesh_from_xml
{
	std::vector<double> color;
	std::string file_name;

	template <class Archive>
	void serialize(Archive& archive, const unsigned int version)
	{
		archive & BOOST_SERIALIZATION_NVP(color);
		archive & BOOST_SERIALIZATION_NVP(file_name);
	}
};

struct A_RigidBody
{
	std::string name;
	double mass{};
	int articulated{0};
	std::vector<Mesh_from_xml> meshs;
	std::vector<double> center_of_mass;
	std::vector<double> moi;
	std::vector<double> position;
	std::vector<double> scale_factor;
	double frictionCoef{1e30};
	double restitutionCoef{0};
	double ODEGroundParam{0};

	template <class Archive>
	void serialize(Archive& archive, const unsigned int version)
	{
		archive & BOOST_SERIALIZATION_NVP(name);
		archive & BOOST_SERIALIZATION_NVP(mass);
		archive & BOOST_SERIALIZATION_NVP(articulated);
		archive & BOOST_SERIALIZATION_NVP(meshs);
		archive & BOOST_SERIALIZATION_NVP(center_of_mass);
		archive & BOOST_SERIALIZATION_NVP(moi);
		archive & BOOST_SERIALIZATION_NVP(position);
		archive & BOOST_SERIALIZATION_NVP(scale_factor);
		archive & BOOST_SERIALIZATION_NVP(frictionCoef);
		archive & BOOST_SERIALIZATION_NVP(restitutionCoef);
		archive & BOOST_SERIALIZATION_NVP(ODEGroundParam);
	}
};

struct Global
{
	std::string modelName;
	std::vector<A_RigidBody> rigidBodies;
	std::vector<Muscle_from_xml> muscles;
	std::vector<Joint_from_xml> joints;

	template <class Archive>
	void serialize(Archive& archive, const unsigned int version)
	{
		archive & BOOST_SERIALIZATION_NVP(modelName);
		archive & BOOST_SERIALIZATION_NVP(rigidBodies);
		archive & BOOST_SERIALIZATION_NVP(muscles);
		archive & BOOST_SERIALIZATION_NVP(joints);
	}
};

void save(Global& parsed_osim, const std::string& filename);

Global load(const std::string& filename);

Coordinate_OpenSim find_coordinate_by_name(const std::string& in_name, Joint_from_xml input);
