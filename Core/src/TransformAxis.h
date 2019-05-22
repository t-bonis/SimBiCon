#pragma once
#include "Utils/src/Utils.h"
#include "Coordinate.h"
#include "Function.h"
#include "Utils/src/SerializationUtils.h"

struct TransformAxis_from_xml;

class TransformAxis
{
public :
	enum transformation_type { rotation, translation };

public :

	TransformAxis();

	~TransformAxis();

	TransformAxis(const TransformAxis& aTransformAxis);

	void set_coordinate(Coordinate* p_coordinate);

	void set_axis(const Vector3d& axis);

	Vector3d get_axis() const { return m_axis; }

	void set_axis(double x, double y, double z);

	void set_function(TransformAxis_from_xml input_axe);

	void set_function();

	double get_value() const
	{
		if (!m_coordinate_dependance)
		{
			return m_relation->calc_value(0.0);
		}
		return m_relation->calc_value(m_coordinate_dependance->coords_value);
	}

	int is_free() const
	{
		if (m_coordinate_dependance == nullptr)
		{
			return 0;
		}
		return 1;
	}

	Coordinate* get_coordinate();

private:

	Vector3d m_axis{0, 0, 0};

	Coordinate* m_coordinate_dependance{};

	std::shared_ptr<Function> m_relation;
};
