#include "SerializationUtils.h"
#include <fstream>


void save(Global& parsed_osim, const std::string& filename)
{
	std::ofstream file(filename);
	boost::archive::xml_oarchive oa(file);
	oa & BOOST_SERIALIZATION_NVP(parsed_osim);
}

Global load(const std::string& filename)
{
	std::ifstream file(filename);
	boost::archive::xml_iarchive ia(file);
	Global input;
	ia >> BOOST_SERIALIZATION_NVP(input);
	return input;
}

Coordinate_OpenSim find_coordinate_by_name(const std::string& in_name, Joint_from_xml input)
{
	for (auto& coordinate : input.coordinates)
	{
		if (coordinate.name == in_name)
		{
			return coordinate;
		}
	}
	throw std::logic_error("coordinate name not found");
}
