#pragma once
#include <vector>
#include <MathLib/src/Quaternion.h>
#include <iostream>
#include <iomanip>
#include <ctime>

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sinks/text_file_backend.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sources/record_ostream.hpp>

namespace logging = boost::log;
namespace src = boost::log::sources;
namespace sinks = boost::log::sinks;
namespace keywords = boost::log::keywords;

//This method throws an error with a specified text and arguments 
//This method reads all the doubles from the given file and stores them in the array of doubles that is passed in
inline void readDoublesFromFile(FILE* f, std::vector<double>* d)
{
	double temp;
	while (fscanf_s(f, "%lf\n", &temp) == 1)
		d->push_back(temp);
}


//This method returns a pointer to the first non-white space character location in the provided buffer
inline char* lTrim(char* buffer)
{
	while (*buffer == ' ' || *buffer == '\t' || *buffer == '\n' || *buffer == '\r')
		buffer++;
	return buffer;
}

inline char* rTrim(char* buffer)
{
	int index = int(strlen(buffer)) - 1;
	while (index >= 0)
	{
		if (buffer[index] == ' ' || buffer[index] == '\t' || buffer[index] == '\n' || buffer[index] == '\r')
		{
			buffer[index] = '\0';
			index--;
		}
		else
			break;
	}
	return buffer;
}

inline char* trim(char* buffer)
{
	return rTrim(lTrim(buffer));
}

//This method reads a line from a file. It does not return empty lines or ones that start with a pound key - those are assumed to be comments.
//This method returns true if a line is read, false otherwise (for instance the end of file is met).
inline bool readValidLine(char* line, FILE* fp)
{
	while (!feof(fp))
	{
		fgets(line, 100, fp);
		char* tmp = trim(line);
		if (tmp[0] != '#' && tmp[0] != '\0')
			return true;
	}

	return false;
}

inline void init()
{
	std::time_t t = std::time(nullptr);
	char mbstr[100];
	std::strftime(mbstr, sizeof(mbstr), "%m%d_%H%M", std::localtime(&t));
	std::string time = mbstr;
	time += ".log";
	logging::add_file_log(time);
}


inline std::ostream& operator<< (std::ostream& in, const Vector3d& value)
{
	in << std::setprecision(std::numeric_limits<double>::digits10 + 1);
	in << value.x << " " << value.y << " " << value.z ;
	return in;
}

inline std::ostream& operator<< (std::ostream& in, const Point3d& value)
{
	in << std::setprecision(std::numeric_limits<double>::digits10 + 1);
	in << value.x << " " << value.y << " " << value.z ;
	return in;
}

inline std::ostream& operator<< (std::ostream& in, const Quaternion& value)
{
	in << std::setprecision(std::numeric_limits<double>::digits10 + 1);
	in << value.s << " " << value.v.x << " " << value.v.y << " " << value.v.z ;
	return in;
}

inline std::ostream& operator<< (std::ostream& in, const std::vector<std::vector<Vector3d>>& matrix)
{
	in << std::setprecision(std::numeric_limits<double>::digits10 + 1);
	for (auto& lines : matrix)
	{
		for (auto& value : lines)
		{
			in << value << " ; ";
		}
		in << std::endl;
	}
	return in;
}

inline std::ostream& operator<< (std::ostream& in, const std::vector<std::vector<Point3d>>& matrix)
{
	in << std::setprecision(std::numeric_limits<double>::digits10 + 1);
	for (auto& lines : matrix)
	{
		for (auto& value : lines)
		{
			in << value << " ; ";
		}
		in << std::endl;
	}
	return in;
}


inline std::ostream& operator<< (std::ostream& in, const std::vector<double>& line)
{
	in << std::setprecision(std::numeric_limits<double>::digits10 + 1);
	for (auto& value : line)
	{
		in << value << " ";
	}
	return in;
}
