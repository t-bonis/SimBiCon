#pragma once
#include "Utils/src/Utils.h"

class Full_character_state
{
public:
	Full_character_state()
	{
		this->m_start_index = 0;
		m_state = std::vector<double>(200);
		//TODO : initialize using number of rigid_body 
	}
	explicit Full_character_state(const std::vector<double>& s, const size_t index = 0)
	{
		set_state(s, index);
	}
	explicit Full_character_state(std::vector<double>&& s, const size_t index = 0)
	{
		set_state(std::move(s), index);
	}
	~Full_character_state() = default;

	Full_character_state(const Full_character_state& other) = delete;
	Full_character_state(Full_character_state&& other) = delete;
	Full_character_state& operator=(const Full_character_state& other) = delete;
	Full_character_state& operator=(Full_character_state&& other) = delete;

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

	Quaternion get_arb_orientation(const size_t j_index) const
	{
		const auto offset = m_start_index + 13 + 13 * j_index;
		return Quaternion(m_state[0 + offset], m_state[1 + offset], m_state[2 + offset], m_state[3 + offset]);
	}

	void set_arb_orientation(const QQuaternion& q, const size_t j_index)
	{
		set_arb_orientation(Quaternion(q.scalar(), q.x(), q.y(), q.z()), j_index);
	}

	void set_arb_orientation(const Quaternion& q, const size_t j_index)
	{
		const auto offset = m_start_index + 13 + 13 * j_index;
		m_state[0 + offset] = q.s;
		m_state[1 + offset] = q.v.x;
		m_state[2 + offset] = q.v.y;
		m_state[3 + offset] = q.v.z;
	}

	Vector3d get_arb_position(const size_t j_index) const
	{
		const auto offset = m_start_index + 13 + 13 * j_index;
		return Vector3d(m_state[4 + offset], m_state[5 + offset], m_state[6 + offset]);
	}

	void set_arb_position(const Vector3d& w, const size_t j_index)
	{
		const auto offset = m_start_index + 13 + 13 * j_index;
		m_state[4 + offset] = w.x;
		m_state[5 + offset] = w.y;
		m_state[6 + offset] = w.z;
	}

	Vector3d get_arb_lin_velocity(const size_t j_index) const
	{
		const auto offset = m_start_index + 13 + 13 * j_index;
		return Vector3d(m_state[7 + offset], m_state[8 + offset], m_state[9 + offset]);
	}

	void set_arb_lin_velocity(const Vector3d& w, const size_t j_index)
	{
		const auto offset = m_start_index + 13 + 13 * j_index;
		m_state[7 + offset] = w.x;
		m_state[8 + offset] = w.y;
		m_state[9 + offset] = w.z;
	}

	Vector3d get_arb_ang_velocity(const size_t j_index) const
	{
		const auto offset = m_start_index + 13 + 13 * j_index;
		return Vector3d(m_state[10 + offset], m_state[11 + offset], m_state[12 + offset]);
	}

	void set_arb_ang_velocity(const Vector3d& w, const size_t j_index)
	{
		const auto offset = m_start_index + 13 + 13 * j_index;
		m_state[10 + offset] = w.x;
		m_state[11 + offset] = w.y;
		m_state[12 + offset] = w.z;
	}

	std::vector<double>* get_state_ptr()
	{
		return &m_state;
	}

private:

	std::vector<double> m_state;

	size_t m_start_index{ 0 };

};
