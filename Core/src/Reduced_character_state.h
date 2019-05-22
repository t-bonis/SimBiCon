#pragma once
#include "Utils/src/Utils.h"

//This class is used to map the array of doubles that is used to define the state of the character to an
//easier to use and understand meaning of the state. This should be the only place in the code where we need to know
//that the first 13 numbers in the array represent the position, orientation, velocity and angular velocity, etc.
//For the state, we consider the 13-dimensional state of the root, and then only
//the relative orientation and angular velocity (as measured in parent coordinates) for
//every other link. The velocities of the CM are derived from this information,
//using the velocity propagation technique (if that's what it is called).
//The order in which the bodies appear is given by the array of joints.
//This works under the assumption that in the joint
//sequence, the parent of any rigid body appears before its children (assuming that for each joint
//we read the parent first and then the child).
//Muscle edit: the activation and excitation of each muscle are also stored in the array.
//Here, the order is that of the Character's muscles array.
class Reduced_character_state
{
public:
	//a useful structure that holds information regarding which joints are relevant for the distance measure, and how much they're worth
	struct Relevant_joint
	{
		//this is the index of the joint, as it appears in the reduced state
		int j_index;
		//this is how much the difference in the relative orientations should be worth
		double worth_q;
		//and this is how much the difference in the relative angular velocity should be worth
		const double worth_v;

		Relevant_joint() = delete;
	};

public:
	Reduced_character_state()
	{
		this->m_start_index = 0;
		m_state = std::vector<double>(200);
		//TODO : initialize using number of joint 
	}
	explicit Reduced_character_state(const std::vector<double>& s, const size_t index = 0)
	{
		set_state(s, index);
	}
	explicit Reduced_character_state(std::vector<double>&& s, const size_t index = 0)
	{
		set_state(std::move(s), index);
	}

	~Reduced_character_state() = default;

	Reduced_character_state(const Reduced_character_state& other) = delete;
	Reduced_character_state(Reduced_character_state&& other)  = delete;
	Reduced_character_state& operator=(const Reduced_character_state& other) = delete;
	Reduced_character_state& operator=(Reduced_character_state&& other)  = delete;
	
	std::vector<double> get_state() const { return m_state; }

	void set_state(const std::vector<double>& s, const size_t index = 0)
	{
		m_state = std::vector<double>(s);
		m_start_index = index;
	}

	void set_state(std::vector<double>&& s, const size_t index = 0)
	{
		m_state = std::move(s);
		m_start_index = index;
	}

	Point3d get_root_position() const
	{
		return Point3d(m_state[0 + m_start_index], m_state[1 + m_start_index], m_state[2 + m_start_index]);
	}

	void set_root_position(const Vector3d& p)
	{
		m_state[0 + m_start_index] = p.x;
		m_state[1 + m_start_index] = p.y;
		m_state[2 + m_start_index] = p.z;
	}

	void set_root_position(const Point3d& p)
	{
		m_state[0 + m_start_index] = p.x;
		m_state[1 + m_start_index] = p.y;
		m_state[2 + m_start_index] = p.z;
	}

	Quaternion get_root_orientation() const
	{
		return Quaternion(m_state[3 + m_start_index], m_state[4 + m_start_index], m_state[5 + m_start_index],
		                  m_state[6 + m_start_index]);
	}

	void set_root_orientation(const QQuaternion& q)
	{
		set_root_orientation(Quaternion(q.scalar(), q.x(), q.y(), q.z()));
	}

	void set_root_orientation(const Quaternion& q)
	{
		m_state[3 + m_start_index] = q.s;
		m_state[4 + m_start_index] = q.v.x;
		m_state[5 + m_start_index] = q.v.y;
		m_state[6 + m_start_index] = q.v.z;
	}

	Vector3d get_root_velocity() const
	{
		return Vector3d(m_state[7 + m_start_index], m_state[8 + m_start_index], m_state[9 + m_start_index]);
	}

	void set_root_velocity(const Vector3d& v)
	{
		m_state[7 + m_start_index] = v.x;
		m_state[8 + m_start_index] = v.y;
		m_state[9 + m_start_index] = v.z;
	}


	Vector3d get_root_angular_velocity() const
	{
		return Vector3d(m_state[10 + m_start_index], m_state[11 + m_start_index], m_state[12 + m_start_index]);
	}


	void set_root_angular_velocity(const Vector3d& v)
	{
		m_state[10 + m_start_index] = v.x;
		m_state[11 + m_start_index] = v.y;
		m_state[12 + m_start_index] = v.z;
	}

	Quaternion get_joint_relative_orientation(const size_t j_index) const
	{
		const auto offset = m_start_index + 13 + 7 * j_index;
		return Quaternion(m_state[0 + offset], m_state[1 + offset], m_state[2 + offset], m_state[3 + offset]);
	}

	void set_joint_relative_orientation(const QQuaternion& q, const size_t j_index)
	{
		set_joint_relative_orientation(Quaternion(q.scalar(), q.x(), q.y(), q.z()), j_index);
	}

	void set_joint_relative_orientation(const Quaternion& q, const size_t j_index)
	{
		const auto offset = m_start_index + 13 + 7 * j_index;
		m_state[0 + offset] = q.s;
		m_state[1 + offset] = q.v.x;
		m_state[2 + offset] = q.v.y;
		m_state[3 + offset] = q.v.z;
	}

	Vector3d get_joint_relative_ang_velocity(const size_t j_index) const
	{
		const auto offset = m_start_index + 13 + 7 * j_index;
		return Vector3d(m_state[4 + offset], m_state[5 + offset], m_state[6 + offset]);
	}

	void set_joint_relative_ang_velocity(const Vector3d& w, const size_t j_index)
	{
		const auto offset = m_start_index + 13 + 7 * j_index;
		m_state[4 + offset] = w.x;
		m_state[5 + offset] = w.y;
		m_state[6 + offset] = w.z;
	}


	double compute_distance_to(const std::shared_ptr<Reduced_character_state>& other) const
	{
		Relevant_joint joints[] = {
			{0, 1.5, 1}, {1, 1.5, 1}, {2, 1.5, 1}, {3, 1.5, 1}, {4, 1.5, 1}, {5, 1.5, 1},
			{6, 1.5, 1}, {7, 1.5, 1}, {8, 1.5, 1}, {9, 1.5, 1}, {10, 1.5, 1}
		};
		double result = 0;

		auto va = this->get_root_orientation().get_conjugate().rotate(this->get_root_velocity());
		auto vb = other->get_root_orientation().get_conjugate().rotate(other->get_root_velocity());

		result += 3 * (va - vb).dotProductWith(va - vb);

		//and now penalize the difference in root angular velocities
		va = this->get_root_orientation().get_conjugate().rotate(this->get_root_angular_velocity());
		vb = other->get_root_orientation().get_conjugate().rotate(other->get_root_angular_velocity());
		result += 3 * (va - vb).dotProductWith(va - vb);

		//now go through all the joints that are relevant, and penalize the differences in the orientations and angular velocities
		for (auto& joint : joints)
		{
			va = this->get_joint_relative_ang_velocity(joint.j_index);
			vb = other->get_joint_relative_ang_velocity(joint.j_index);
			result += joint.worth_v * (va - vb).dotProductWith(va - vb);
		}

		return result;
	}

	std::vector<double>* get_state_ptr()
	{
		return &m_state;
	}

private:

	std::vector<double> m_state;

	size_t m_start_index{0};
};
