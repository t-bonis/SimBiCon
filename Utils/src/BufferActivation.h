#pragma once
#include <queue>

//This class enables the storing (fifo) and reuse of muscle activations from a certain number of contiguous previous time steps
class Buffer_activation
{
public:
	explicit Buffer_activation(const size_t nb = 1)
	{
		m_size = nb;
		m_buffer = std::queue<double>();
	};

	// reset the buffer according to the number of needed values
	void set_size(const size_t nb)
	{
		m_size = nb;
	};

	// setDataFromPosition the newest value to the buffer
	void add(double a)
	{
		if (m_buffer.size() >= m_size && !m_buffer.empty())
		{
			m_buffer.pop();
		}
		m_buffer.push(a);
	};

	// access the value used for the update of the excitation (leaving it in the buffer)
	double get()
	{
		return m_buffer.front();
	};

	// access the most recently added value (leaving it in the buffer)
	double get_newest()
	{
		if (!m_buffer.empty()) return m_buffer.back();
		return -1;
	};

	// know whether the buffer already contains previous values
	bool is_empty() const { return m_buffer.empty(); }; //changed from 1 to 0

	void reset()
	{
		if (is_empty())
			return;
		for (size_t i = 0; i < m_size; ++i)
		{
			m_buffer.pop();
			m_buffer.push(0.0);
		}
	}

private:
	size_t m_size; // amount of stored values
	std::queue<double> m_buffer;
};
