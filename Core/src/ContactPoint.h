#pragma once
#include "Utils/src/Utils.h"
class Rigid_body;

class Contact_point
{
public:
	Contact_point()
	{
		rb1 = rb2 = nullptr;
	}
	Contact_point(const Contact_point& other)
	{
		this->cp = other.cp;
		this->f = other.f;
		this->n = other.n;
		this->d = other.d;
		this->rb1 = other.rb1;
		this->rb2 = other.rb2;
	}

	~Contact_point() = default;

	Contact_point(Contact_point&& other) = delete;
	Contact_point& operator=(const Contact_point& other) = delete;
	Contact_point& operator=(Contact_point&& other) = delete;


	//this is the world coordinate of the origin of the contact force...
	Point3d cp;
	//this is the normal at the contact point
	Vector3d n;
	//and this is the penetration depth
	double d{};
	//this is the first rigid body that participated in the contact
	Rigid_body* rb1;
	//and this is the second...
	Rigid_body* rb2;
	//and this is the force applied (with f being applied to rb1, and -f to rb2)
	Vector3d f;

	Vector3d t1;
	Vector3d t2;

};
